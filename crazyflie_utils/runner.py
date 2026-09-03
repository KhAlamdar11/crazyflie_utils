"""Run one stress scenario end to end.

Responsibilities, in order:

1. build the fleet client and the link monitor,
2. spin them on a background executor so scenario code can be written as
   straight-line blocking calls,
3. pre-flight: wait for the crazyflie_server services and for telemetry from
   every drone, and refuse to fly if something is missing,
4. run the scenario,
5. **always** bring the fleet down -- Ctrl-C, a scenario exception and a
   telemetry watchdog trip all land through the same path,
6. render and write the report.

The executor is multi-threaded with reentrant callbacks so a scenario can have
several service calls in flight at once (which is the whole point of the
parallel dispatch mode).
"""

from __future__ import annotations

import threading
import time
import traceback
from typing import Any, Dict, List, Optional

from rclpy.executors import MultiThreadedExecutor

from crazyflie_utils.cf_fleet import CrazyflieFleet
from crazyflie_utils.colors import bold, cyan, green, red, yellow
from crazyflie_utils.config import arming_mode, cfg_get, resolve_namespaces
from crazyflie_utils.link_monitor import LinkMonitor
from crazyflie_utils.metrics import MetricsCollector
from crazyflie_utils.report import format_console_report, write_reports
from crazyflie_utils.scenarios import AbortRequested, get_scenario

#: Fallback for ``safety.required_services``: what the flight scenarios
#: actually command. ``emergency`` is deliberately not in here -- plenty of
#: servers never advertise it, and it is only needed when it is the configured
#: abort action.
REQUIRED_SERVICES = ('takeoff', 'land')


class StressTestRunner:
    """Owns the node, the executor thread and the abort path for one run."""

    def __init__(self, cfg: Dict[str, Any]):
        self.cfg = cfg
        self.namespaces: List[str] = resolve_namespaces(cfg)
        self.scenario_name = str(cfg_get(cfg, 'test.type'))
        self.scenario_cls = get_scenario(self.scenario_name)
        self.metrics = MetricsCollector(self.scenario_name, cfg)
        self.abort_event = threading.Event()

        self.fleet: Optional[CrazyflieFleet] = None
        self.monitor: Optional[LinkMonitor] = None
        self.executor: Optional[MultiThreadedExecutor] = None
        self._spin_thread: Optional[threading.Thread] = None

    # -- lifecycle ---------------------------------------------------------

    def request_abort(self, reason: str = 'signal') -> None:
        """Ask the running scenario to stop and hand over to the abort path."""
        if not self.abort_event.is_set():
            self.metrics.add_event('abort_requested', reason=reason)
            self.abort_event.set()
            if self.fleet is not None:
                self.fleet.get_logger().warn(
                    yellow(f"Abort requested ({reason}) -- stopping the scenario "
                           f"and bringing the fleet down"))

    def _start(self) -> None:
        self.fleet = CrazyflieFleet(self.cfg, self.namespaces, self.metrics)
        self.monitor = LinkMonitor(
            self.fleet, self.namespaces, self.cfg.get('telemetry', {}))
        self.executor = MultiThreadedExecutor()
        self.executor.add_node(self.fleet)
        self._spin_thread = threading.Thread(
            target=self.executor.spin, name='cf_utils_executor', daemon=True)
        self._spin_thread.start()

    def _stop(self) -> None:
        if self.monitor is not None:
            self.monitor.destroy()
        if self.executor is not None:
            self.executor.shutdown()
        if self._spin_thread is not None:
            self._spin_thread.join(timeout=2.0)
        if self.fleet is not None:
            self.fleet.destroy_node()

    # -- pre-flight --------------------------------------------------------

    def _preflight(self, requires_flight: bool) -> bool:
        """Verify the server and the drones are there. False = do not fly."""
        assert self.fleet is not None and self.monitor is not None
        log = self.fleet.get_logger()
        require_all = bool(cfg_get(self.cfg, 'safety.require_all_services', True))
        dry_run = bool(cfg_get(self.cfg, 'safety.dry_run', False))

        required = [str(s) for s in (cfg_get(self.cfg, 'safety.required_services')
                                     or REQUIRED_SERVICES)]
        # The abort path is only as good as the service it calls, so require
        # whichever one this run would use to bring the fleet down.
        if bool(cfg_get(self.cfg, 'safety.emergency_on_abort', False)):
            required.append('emergency')

        timeout = float(cfg_get(self.cfg, 'timing.service_ready_timeout', 4.0))
        if dry_run:
            # A rehearsal should not sit through the full discovery timeout;
            # still probe briefly so the report says what was missing.
            timeout = min(timeout, 1.0)
        log.info(cyan(f"Pre-flight: waiting up to {timeout:.0f}s for "
                      f"{', '.join(required)} on {len(self.namespaces)} "
                      f"drone(s)"))
        missing = self.fleet.wait_for_services(required, timeout)
        if missing:
            detail = '; '.join(f"{ns}: {', '.join(actions)}"
                               for ns, actions in sorted(missing.items()))
            self.metrics.add_event('missing_services', detail=missing)
            if require_all and not dry_run:
                log.error(red(f"Missing services -- {detail}"))
                log.error(red('Is crazyflie_server running, and do fleet.namespaces '
                              'match its robot names? Set '
                              'safety.require_all_services: false to run anyway.'))
                self.metrics.add_finding(f"ABORTED: missing services -- {detail}")
                return False
            log.warn(yellow(f"Missing services (continuing anyway) -- {detail}"))
        else:
            log.info(green('All expected services are available'))

        if self.monitor.enabled and not dry_run:
            silent = self.monitor.wait_for_telemetry(timeout=float(
                cfg_get(self.cfg, 'timing.telemetry_ready_timeout', 4.0)))
            if silent:
                message = (f"No telemetry from {', '.join(silent)} -- those drones "
                           f"are not reporting pose")
                self.metrics.add_event('silent_drones', namespaces=silent)
                if require_all and requires_flight:
                    log.error(red(message))
                    self.metrics.add_finding(f"ABORTED: {message}")
                    return False
                log.warn(yellow(message))
            else:
                log.info(green('Telemetry flowing from every drone'))

        return self._check_arming(requires_flight)

    def _check_arming(self, requires_flight: bool) -> bool:
        """Verify arming is possible. False = do not fly.

        Nothing is armed here: each drone is armed immediately before its own
        takeoff, in :meth:`crazyflie_utils.scenarios.base.StressScenario.arm_for_takeoff`.
        Arming the whole fleet at startup would leave drones sitting armed
        through the pre-flight pauses, and the supervisor disarms on landing
        detection anyway, so the state would not survive to the second cycle.

        ``safety.arm_before_flight`` is tri-state:
          ``auto`` (default) arm every drone whose ``<ns>/arm`` service is
              advertised, and simply skip the ones without it -- firmware
              older than the supervisor does not need arming and servers
              that predate the Arm service never offer it,
          ``true``/``always`` arming is mandatory; a missing service aborts
              the run here rather than mid-flight,
          ``false``/``never`` never arm.
        """
        assert self.fleet is not None
        log = self.fleet.get_logger()
        mode = arming_mode(self.cfg)

        if not requires_flight or mode == 'never':
            return True
        strict = mode == 'always'

        if not self.fleet.arm_supported():
            message = ('crazyflie_interfaces/srv/Arm is not available in this '
                       'workspace, so the fleet cannot be armed')
            if strict:
                log.error(red(message))
                self.metrics.add_finding(f"ABORTED: {message}")
                return False
            log.info(cyan(f"Skipping arming: {message}"))
            return True

        armable = self.fleet.armable(self.namespaces)
        unarmable = [ns for ns in self.namespaces if ns not in armable]

        if unarmable:
            message = f"no arm service for {', '.join(unarmable)}"
            if strict:
                log.error(red(message))
                self.metrics.add_finding(f"ABORTED: {message}")
                return False
            log.info(cyan(f"Arming skipped where unsupported: {message}"))
        if not armable:
            return True

        delay = float(cfg_get(self.cfg, 'timing.arm_delay', 0.5))
        self.metrics.add_event('arming_plan', namespaces=armable, arm_delay=delay)
        log.info(green(
            f"Arm service ready for {len(armable)} drone(s): {', '.join(armable)} "
            f"-- each is armed {delay:.2f} s before its own takeoff"))
        return True

    # -- abort / shutdown path --------------------------------------------

    def _bring_fleet_down(self, requires_flight: bool) -> None:
        """Land (or cut) the fleet. Safe to call more than once."""
        assert self.fleet is not None
        if not requires_flight:
            return
        log = self.fleet.get_logger()

        if bool(cfg_get(self.cfg, 'safety.emergency_on_abort', False)):
            log.warn(red('EMERGENCY STOP -- cutting motors'))
            self.fleet.broadcast_emergency()
            for ns in self.namespaces:
                self.fleet.emergency(ns)
            return

        if not bool(cfg_get(self.cfg, 'safety.land_on_abort', True)):
            log.warn(yellow('safety.land_on_abort is false -- leaving the fleet '
                            'as it is'))
            return

        height = float(cfg_get(self.cfg, 'safety.abort_land_height', 0.0))
        duration = float(cfg_get(self.cfg, 'safety.abort_land_duration', 2.0))
        log.warn(yellow(f"Landing fleet ({duration:.1f}s)"))

        # Release any streaming-priority lock first, otherwise the Land is a
        # no-op for a drone that was being fed setpoints.
        for ns in self.namespaces:
            self.fleet.notify_setpoints_stop(ns, 100, phase='shutdown')

        record = self.fleet.broadcast_land(height, duration, phase='shutdown')
        if not record.success:
            log.warn(yellow('Broadcast land failed -- falling back to per-drone '
                            'land commands'))
            for ns in self.namespaces:
                self.fleet.land(ns, height, duration, phase='shutdown')
        # Give the trajectory time to finish before the process exits.
        time.sleep(min(5.0, duration + 1.0))

        if (cfg_get(self.cfg, 'safety.disarm_after_flight', False)
                and self.fleet.arm_supported()):
            for ns in self.namespaces:
                self.fleet.arm(ns, False, phase='shutdown')

    # -- main entry --------------------------------------------------------

    def run(self) -> int:
        """Run the configured scenario. Returns a process exit code."""
        self._start()
        assert self.fleet is not None and self.monitor is not None
        log = self.fleet.get_logger()
        exit_code = 0
        requires_flight = bool(self.scenario_cls.requires_flight)

        if bool(cfg_get(self.cfg, 'safety.dry_run', False)):
            self.metrics.add_finding(
                'DRY RUN: no commands were sent, so latency, altitude and skew '
                'numbers in this report are placeholders')

        try:
            if not self._preflight(requires_flight):
                self.metrics.aborted = True
                return 2

            scenario = self.scenario_cls(
                self.fleet, self.monitor, self.metrics, self.cfg, self.abort_event)
            log.info(bold(f"Running scenario '{self.scenario_name}'"))
            self.metrics.add_event('scenario_start', scenario=self.scenario_name)

            try:
                scenario.run()
                self.metrics.add_event('scenario_end', status='completed')
                log.info(green(f"Scenario '{self.scenario_name}' completed"))
            except AbortRequested as exc:
                self.metrics.aborted = True
                self.metrics.add_event('scenario_end', status='aborted',
                                       reason=str(exc))
                self.metrics.add_finding(
                    f"ABORTED: scenario stopped early "
                    f"({str(exc) or 'user request'}) -- the numbers below cover only "
                    f"the part that ran")
                log.warn(yellow(f"Scenario aborted: {str(exc) or 'user request'}"))
                exit_code = 1
            except Exception as exc:  # scenario bug or unexpected ROS failure
                self.metrics.aborted = True
                self.metrics.add_event('scenario_end', status='error',
                                       reason=f"{type(exc).__name__}: {exc}")
                self.metrics.add_finding(
                    f"ABORTED: scenario raised {type(exc).__name__}: {exc}")
                log.error(red(f"Scenario failed: {type(exc).__name__}: {exc}"))
                log.error(red(traceback.format_exc()))
                exit_code = 1
            finally:
                self._bring_fleet_down(requires_flight)
        finally:
            self.metrics.finished_wall = time.time()
            self._publish_report()
            self._stop()

        return exit_code

    def _publish_report(self) -> None:
        summary = self.metrics.summary()
        report_cfg = self.cfg.get('report', {}) or {}

        if report_cfg.get('console', True):
            print(format_console_report(summary))

        try:
            written = write_reports(summary, self.metrics.call_rows(), report_cfg)
        except OSError as exc:
            print(red(f"Could not write report artefacts: {exc}"))
            return
        for path in written:
            print(cyan(f"wrote {path}"))


def run_config(cfg: Dict[str, Any]) -> int:
    """Convenience wrapper: run one config, assuming rclpy is initialised."""
    return StressTestRunner(cfg).run()
