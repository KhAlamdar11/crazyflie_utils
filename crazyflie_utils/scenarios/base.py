"""Common machinery shared by every stress scenario.

A scenario is a plain object with a :meth:`StressScenario.run` method that
issues commands through :class:`~crazyflie_utils.cf_fleet.CrazyflieFleet` and
records observations through
:class:`~crazyflie_utils.metrics.MetricsCollector`. Everything it needs to
decide *what* to do comes from ``self.params``, which is
``DEFAULT_PARAMS`` deep-merged with the ``test.params`` block of the YAML
config -- scenarios must not hardcode heights, durations or counts.

The base class provides the pieces every scenario needs: abort-aware sleeping,
telemetry measurement windows, and the three fleet-command dispatch styles
(broadcast / parallel unicast / sequential unicast) that the sync-vs-async
comparison is built on.
"""

from __future__ import annotations

import time
from contextlib import contextmanager
from typing import Any, Callable, Dict, List, Optional, Sequence

from crazyflie_utils.cf_fleet import CrazyflieFleet, PendingCall
from crazyflie_utils.config import arming_mode, cfg_get, clamp_height, deep_merge
from crazyflie_utils.link_monitor import LinkMonitor
from crazyflie_utils.metrics import CallRecord, MetricsCollector

#: Fleet-command dispatch styles.
#:   broadcast  - one /all/<action> call, server emits one broadcast packet
#:   parallel   - one unicast call per drone, all in flight simultaneously
#:   sequential - one unicast call per drone, each awaited before the next
DISPATCH_MODES = ('broadcast', 'parallel', 'sequential')


class AbortRequested(Exception):
    """Raised inside a scenario when Ctrl-C (or a safety trip) fires."""


class StressScenario:
    """Base class for all stress scenarios."""

    #: Name used in ``test.type`` and in the scenario registry.
    name: str = 'base'
    #: One-line description shown by ``--list``.
    description: str = ''
    #: Scenario defaults; YAML ``test.params`` is merged on top of this.
    DEFAULT_PARAMS: Dict[str, Any] = {}
    #: False for ground-only scenarios (skips the pre-flight arm/abort-land).
    requires_flight: bool = True

    def __init__(self, fleet: CrazyflieFleet, monitor: LinkMonitor,
                 metrics: MetricsCollector, cfg: Dict[str, Any], abort_event):
        self.fleet = fleet
        self.monitor = monitor
        self.metrics = metrics
        self.cfg = cfg
        self.abort_event = abort_event
        self.namespaces: List[str] = list(fleet.namespaces)
        self.params: Dict[str, Any] = deep_merge(
            self.DEFAULT_PARAMS, cfg_get(cfg, 'test.params', {}) or {})
        self.service_timeout = float(cfg_get(cfg, 'timing.service_timeout', 3.0))
        self.settle_time = float(cfg_get(cfg, 'timing.settle_time', 2.0))
        # 'auto' | 'always' | 'never' -- consulted before every takeoff.
        self.arm_mode = arming_mode(cfg)
        # Pause between arming a drone and commanding it to take off.
        self.arm_delay = float(cfg_get(cfg, 'timing.arm_delay', 0.5))

    # -- to implement ------------------------------------------------------

    def run(self) -> None:
        raise NotImplementedError

    # -- logging -----------------------------------------------------------

    def info(self, text: str) -> None:
        self.fleet.get_logger().info(text)

    def warn(self, text: str) -> None:
        self.fleet.get_logger().warn(text)

    def error(self, text: str) -> None:
        self.fleet.get_logger().error(text)

    # -- abort-aware timing ------------------------------------------------

    def check_abort(self) -> None:
        if self.abort_event.is_set():
            raise AbortRequested()

    def sleep(self, seconds: float) -> None:
        """Sleep, waking early (and raising) if an abort is requested."""
        if seconds <= 0:
            self.check_abort()
            return
        if self.abort_event.wait(seconds):
            raise AbortRequested()

    def check_link_health(self) -> None:
        """Trip the abort path when a drone's telemetry has gone silent.

        Controlled by ``safety.telemetry_stall_abort`` (seconds, 0 disables).
        A drone that has stopped reporting is one we can no longer command
        reliably, so continuing to stress it is not useful -- and it is
        airborne.
        """
        threshold = float(cfg_get(self.cfg, 'safety.telemetry_stall_abort', 0.0))
        if threshold <= 0 or not self.monitor.enabled:
            return
        for ns in self.namespaces:
            age = self.monitor.seconds_since_pose(ns)
            if age is not None and age > threshold:
                message = (f"Telemetry from {ns} silent for {age:.1f} s "
                           f"(limit {threshold:.1f} s) -- aborting")
                self.error(message)
                self.metrics.add_finding(f"ABORTED: {message}")
                self.abort_event.set()
                raise AbortRequested(message)

    # -- telemetry windows -------------------------------------------------

    @contextmanager
    def measure(self, label: str):
        """Record a downlink measurement window around a block of commands."""
        t_start = self.monitor.begin_window()
        self.metrics.add_event('window_start', label=label)
        try:
            yield
        finally:
            for window in self.monitor.close_window(label, t_start):
                self.metrics.add_telemetry(window)
            self.metrics.add_event('window_end', label=label)

    # -- fleet commands ----------------------------------------------------

    def dispatch_fleet_async(self, kind: str, mode: str, phase: str,
                             targets: Optional[Sequence[str]] = None,
                             stagger: float = 0.0,
                             **kwargs: Any) -> List[PendingCall]:
        """Send a fleet command without waiting for any response.

        The caller is responsible for passing the returned handles to
        :meth:`crazyflie_utils.cf_fleet.CrazyflieFleet.gather` -- until then
        the calls are in flight, which is what a fire-and-forget flood looks
        like. ``sequential`` degenerates to ``parallel`` here (there is
        nothing to serialise on), but its ``stagger`` still applies.
        """
        if mode not in DISPATCH_MODES:
            raise ValueError(
                f"Unknown dispatch mode '{mode}', expected one of {DISPATCH_MODES}")
        targets = list(targets if targets is not None else self.namespaces)

        if mode == 'broadcast':
            broadcast = {
                'takeoff': self.fleet.broadcast_takeoff_async,
                'land': self.fleet.broadcast_land_async,
                'go_to': self.fleet.broadcast_go_to_async,
            }[kind]
            return [broadcast(phase=phase, **kwargs)]

        async_fn = {
            'takeoff': self.fleet.takeoff_async,
            'land': self.fleet.land_async,
            'go_to': self.fleet.go_to_async,
        }[kind]

        pending: List[PendingCall] = []
        for ns in targets:
            pending.append(async_fn(ns, phase=phase, **kwargs))
            if stagger > 0:
                self.sleep(stagger)
        return pending

    def _dispatch_fleet(self, kind: str, mode: str, phase: str,
                        targets: Optional[Sequence[str]] = None,
                        stagger: float = 0.0,
                        before_each: Optional[Callable[[str], None]] = None,
                        **kwargs: Any) -> List[CallRecord]:
        """Issue ``kind`` to the fleet using dispatch style ``mode``.

        ``before_each`` runs immediately before each drone's own command and
        only applies to ``sequential`` -- it is how per-drone arming is
        interleaved into a one-at-a-time takeoff.
        """
        if mode == 'sequential':
            targets = list(targets if targets is not None else self.namespaces)
            async_fn = {
                'takeoff': self.fleet.takeoff_async,
                'land': self.fleet.land_async,
                'go_to': self.fleet.go_to_async,
            }[kind]
            records: List[CallRecord] = []
            for ns in targets:
                if before_each is not None:
                    before_each(ns)
                records.append(self.fleet.wait(async_fn(ns, phase=phase, **kwargs)))
                if stagger > 0:
                    self.sleep(stagger)
            return records

        pending = self.dispatch_fleet_async(
            kind, mode, phase, targets, stagger, **kwargs)
        return self.fleet.gather(pending, timeout=self.service_timeout)

    def arm_for_takeoff(self, targets: Sequence[str], phase: str,
                        mode: str = 'parallel') -> List[CallRecord]:
        """Arm ``targets`` and pause ``timing.arm_delay`` before returning.

        Called from :meth:`takeoff_fleet`, so **every** takeoff in every
        scenario is preceded by a fresh arm: firmware with the supervisor
        disarms itself on landing detection, so each takeoff in a cycle test
        needs its own arm -- and an unarmed takeoff is indistinguishable from
        a lost command in the report.

        In ``sequential`` mode this is called once per drone with a single
        target, so each drone is armed just before its own takeoff rather than
        the whole fleet being armed up front. In the simultaneous modes the
        takeoff itself is a single event, so the group is armed together.
        """
        if self.arm_mode == 'never':
            return []
        armable = self.fleet.armable(targets)
        if not armable:
            if self.arm_mode == 'always':
                raise AbortRequested(
                    f"arming required but no arm service for "
                    f"{', '.join(targets)}")
            return []

        # Keep a broadcast takeoff a single radio transaction end to end by
        # arming through /all/arm when the server offers it.
        if mode == 'broadcast' and self.fleet.armable([self.fleet.broadcast_ns]):
            records = self.fleet.arm_all([self.fleet.broadcast_ns], True,
                                         phase=f"{phase}/arm")
        else:
            records = self.fleet.arm_all(armable, True, phase=f"{phase}/arm")
        failed = [r.target for r in records if not r.success]
        if failed:
            message = f"arming failed for {', '.join(failed)} before takeoff"
            if self.arm_mode == 'always':
                raise AbortRequested(message)
            self.warn(message)

        # Let the supervisor latch the armed state before the takeoff lands on
        # it. Skipped in dry-run, where nothing was actually armed.
        if not self.fleet.dry_run:
            self.sleep(self.arm_delay)
        return records

    def takeoff_fleet(self, mode: str, height: float, duration: float,
                      phase: str, targets: Optional[Sequence[str]] = None,
                      stagger: float = 0.0) -> List[CallRecord]:
        targets = list(targets if targets is not None else self.namespaces)

        if mode == 'sequential':
            # arm cf1 -> wait -> take off cf1 -> stagger -> arm cf2 -> ...
            def before_each(ns: str) -> None:
                self.arm_for_takeoff([ns], phase, 'parallel')
        else:
            # takeoff is one simultaneous event here, so arming has to be too.
            self.arm_for_takeoff(targets, phase, mode)
            before_each = None  # type: ignore[assignment]

        return self._dispatch_fleet('takeoff', mode, phase, targets, stagger,
                                    before_each=before_each,
                                    height=float(height), duration=float(duration))

    def land_fleet(self, mode: str, height: float, duration: float,
                   phase: str, targets: Optional[Sequence[str]] = None,
                   stagger: float = 0.0) -> List[CallRecord]:
        return self._dispatch_fleet('land', mode, phase, targets, stagger,
                                    height=float(height), duration=float(duration))

    def go_to_fleet(self, mode: str, x: float, y: float, z: float, yaw: float,
                    duration: float, relative: bool, phase: str,
                    targets: Optional[Sequence[str]] = None,
                    stagger: float = 0.0) -> List[CallRecord]:
        return self._dispatch_fleet('go_to', mode, phase, targets, stagger,
                                    x=float(x), y=float(y), z=float(z),
                                    yaw=float(yaw), duration=float(duration),
                                    relative=bool(relative))

    # -- flight helpers ----------------------------------------------------

    def clamp(self, height: float) -> float:
        return clamp_height(self.cfg, height)

    def wait_for_altitude(self, z_threshold: float, since: float,
                          timeout: float,
                          targets: Optional[Sequence[str]] = None
                          ) -> Dict[str, Optional[float]]:
        """Wait until every drone's reported ``z`` crosses ``z_threshold``.

        Returns ``{namespace: seconds after 'since' when it crossed}`` with
        ``None`` for drones that never made it (a strong signal that the
        takeoff command was lost).
        """
        targets = list(targets if targets is not None else self.namespaces)
        crossings: Dict[str, Optional[float]] = {ns: None for ns in targets}
        if self.fleet.dry_run:
            # Nothing was commanded, so nothing will ever climb -- do not burn
            # the timeout on every cycle of a rehearsal.
            return crossings
        deadline = time.perf_counter() + timeout

        while time.perf_counter() < deadline:
            self.check_abort()
            for ns in targets:
                if crossings[ns] is None:
                    stamp = self.monitor.first_time_above(ns, z_threshold, since)
                    if stamp is not None:
                        crossings[ns] = stamp - since
            if all(value is not None for value in crossings.values()):
                break
            time.sleep(0.01)
        return crossings

    def report_altitudes(self, phase: str) -> Dict[str, Optional[float]]:
        """Snapshot every drone's last reported altitude."""
        heights = {ns: self.monitor.latest_z(ns) for ns in self.namespaces}
        self.metrics.add_event('altitudes', phase=phase, heights=heights)
        return heights

    def summarise_records(self, records: Sequence[CallRecord]) -> str:
        """Compact ``ok/total (max latency)`` string for log lines."""
        ok = sum(1 for r in records if r.success)
        worst = max((r.latency for r in records), default=0.0)
        return f"{ok}/{len(records)} ok, worst {worst * 1000:.0f} ms"
