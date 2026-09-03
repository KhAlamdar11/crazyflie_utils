"""Streaming-setpoint bandwidth test.

High-level commands (takeoff/go_to) are occasional; streaming setpoints
(``cmd_hover``, ``cmd_full_state``) are continuous, one packet per drone per
tick, and they are what actually saturates a Crazyradio. This scenario walks
the stream rate up through a list of stages and, at each stage, measures what
the *downlink* is still able to deliver.

A radio that is busy carrying 5 drones x 100 Hz of uplink has little left for
pose telemetry, and the collapse of ``rate_ratio`` in the report is that
effect made visible. Compare stages against each other to find the rate your
setup can actually sustain.

Safety notes:

* The fleet is taken off with the high-level commander first, because
  streaming a hover setpoint at a grounded drone is how you get a drone
  scrambling across the floor.
* ``notify_setpoints_stop`` is called on every drone before landing --
  without it the streaming-priority lock makes the Land command a no-op.
* The drones are commanded to hover in place (zero velocity) unless ``vx`` /
  ``vy`` / ``yaw_rate`` are set. Note that ``cmd_hover`` is a *velocity*
  setpoint: zero means "hold this velocity", not "hold this position", so
  expect drift over long stages.
"""

from __future__ import annotations

import time
from typing import Any, Dict, List

from crazyflie_utils.scenarios.base import StressScenario


class SetpointStreamScenario(StressScenario):
    """Stream setpoints at increasing rates and watch telemetry degrade."""

    name = 'setpoint_stream'
    description = ('Stream cmd_hover/cmd_full_state at increasing rates and '
                   'measure downlink degradation')

    DEFAULT_PARAMS: Dict[str, Any] = {
        # hover | full_state | position
        'setpoint_type': 'hover',
        # Per-drone publish rates to walk through.
        'rates_hz': [10.0, 25.0, 50.0, 100.0],
        'stage_duration': 15.0,
        'recover_time': 3.0,
        'takeoff_height': 0.5,
        'takeoff_duration': 2.0,
        'takeoff_mode': 'broadcast',
        'settle_time': 3.0,
        # Streamed setpoint contents.
        'hover_z': 0.5,
        'vx': 0.0,
        'vy': 0.0,
        'yaw_rate': 0.0,
        # Grace window handed to notify_setpoints_stop before landing, so the
        # last setpoint stays valid across the handover.
        'notify_stop_ms': 300,
        'land_height': 0.0,
        'land_duration': 2.0,
        'land_mode': 'broadcast',
        'airborne_fraction': 0.6,
        'airborne_timeout': 6.0,
    }

    def run(self) -> None:
        setpoint_type = str(self.params['setpoint_type'])
        supported = self.fleet.setpoint_types()
        if setpoint_type not in supported:
            raise ValueError(
                f"setpoint_type '{setpoint_type}' not available; this workspace "
                f"supports {supported}")

        rates = self.params['rates_hz']
        rates = [float(r) for r in (rates if isinstance(rates, (list, tuple)) else [rates])]
        height = self.clamp(float(self.params['takeoff_height']))
        hover_z = self.clamp(float(self.params['hover_z']))

        self.info(
            f"setpoint_stream: {setpoint_type} at {rates} Hz x "
            f"{self.params['stage_duration']} s to {len(self.namespaces)} drone(s)")

        t_command = time.perf_counter()
        with self.measure('takeoff'):
            self.takeoff_fleet(
                str(self.params['takeoff_mode']), height,
                float(self.params['takeoff_duration']), phase='takeoff')
            crossings = self.wait_for_altitude(
                height * float(self.params['airborne_fraction']), t_command,
                float(self.params['airborne_timeout']))
        missing = sorted(ns for ns, t in crossings.items() if t is None)
        if missing:
            self.warn(f"Not airborne before streaming: {', '.join(missing)}")
        self.sleep(float(self.params['settle_time']))

        stages: List[Dict[str, Any]] = []
        try:
            for rate in rates:
                self.check_abort()
                self.check_link_health()
                phase = f"stream_{rate:g}hz"
                t_stage = self.monitor.begin_window()
                stage = self._stream_stage(rate, setpoint_type, hover_z, phase)
                windows = self.monitor.close_window(phase, t_stage)
                for window in windows:
                    self.metrics.add_telemetry(window)
                stage['telemetry'] = {w.namespace: {
                    'pose_rate': w.pose_rate,
                    'rate_ratio': w.rate_ratio,
                    'max_gap': w.max_gap,
                    'stalls': w.stalls,
                    'rssi_mean': w.rssi_mean,
                } for w in windows}
                stages.append(stage)

                worst_ratio = min((w.rate_ratio for w in windows), default=float('nan'))
                worst_gap = max((w.max_gap for w in windows), default=float('nan'))
                self.info(
                    f"  {rate:g} Hz/drone ({stage['offered_msgs_per_s']:.0f} msg/s "
                    f"fleet-wide): achieved {stage['achieved_rate_hz']:.1f} Hz, "
                    f"downlink {worst_ratio * 100:.0f}% of expected, worst gap "
                    f"{worst_gap * 1000:.0f} ms")
                self.sleep(float(self.params['recover_time']))
        finally:
            # Always release the streaming lock, even on abort, or the land
            # below (and the runner's abort land) silently does nothing.
            for ns in self.namespaces:
                self.fleet.notify_setpoints_stop(
                    ns, int(self.params['notify_stop_ms']), phase='handover')
            with self.measure('land'):
                self.land_fleet(
                    str(self.params['land_mode']),
                    float(self.params['land_height']),
                    float(self.params['land_duration']), phase='land')

        self.metrics.set_metric('stages', stages)
        self.metrics.set_metric('setpoint_type', setpoint_type)
        self._add_findings(stages)

    # -- one stage ---------------------------------------------------------

    def _stream_stage(self, rate: float, setpoint_type: str, hover_z: float,
                      phase: str) -> Dict[str, Any]:
        duration = float(self.params['stage_duration'])
        period = 1.0 / rate if rate > 0 else 0.0
        vx = float(self.params['vx'])
        vy = float(self.params['vy'])
        yaw_rate = float(self.params['yaw_rate'])

        # Absolute setpoint types need a position to aim at; hold whatever the
        # drone reported when the stage started.
        anchors = {ns: (self.monitor.latest_position(ns) or (0.0, 0.0, hover_z))
                   for ns in self.namespaces}

        start = time.perf_counter()
        tick = 0
        published = 0
        overruns = 0
        worst_tick = 0.0

        while time.perf_counter() - start < duration:
            self.check_abort()
            tick_start = time.perf_counter()
            for ns in self.namespaces:
                if setpoint_type == 'hover':
                    self.fleet.publish_hover(ns, vx, vy, yaw_rate, hover_z)
                elif setpoint_type == 'full_state':
                    x, y, _ = anchors[ns]
                    self.fleet.publish_full_state(ns, x, y, hover_z)
                else:
                    x, y, _ = anchors[ns]
                    self.fleet.publish_position(ns, x, y, hover_z)
                published += 1
            worst_tick = max(worst_tick, time.perf_counter() - tick_start)
            tick += 1

            if period:
                slack = (start + tick * period) - time.perf_counter()
                if slack > 0:
                    self.sleep(slack)
                else:
                    overruns += 1

        elapsed = time.perf_counter() - start
        self.metrics.add_event('stream_stage', phase=phase, rate_hz=rate,
                               published=published, duration=elapsed)
        return {
            'rate_hz': rate,
            'ticks': tick,
            'published': published,
            'duration_s': elapsed,
            'achieved_rate_hz': (tick / elapsed) if elapsed > 0 else 0.0,
            'offered_msgs_per_s': (published / elapsed) if elapsed > 0 else 0.0,
            'pacing_overruns': overruns,
            'worst_tick_s': worst_tick,
        }

    # -- verdict -----------------------------------------------------------

    def _add_findings(self, stages: List[Dict[str, Any]]) -> None:
        if not stages:
            return
        for stage in stages:
            telemetry = stage.get('telemetry', {})
            offered = (f"{stage['rate_hz']:g} Hz/drone "
                       f"({stage['offered_msgs_per_s']:.0f} msg/s fleet-wide)")
            if not telemetry:
                self.metrics.add_finding(f"{offered}: no telemetry recorded")
                continue
            worst_ratio = min(t['rate_ratio'] for t in telemetry.values())
            worst_gap = max(t['max_gap'] for t in telemetry.values())
            self.metrics.add_finding(
                f"{offered}: downlink at {worst_ratio * 100:.0f}% of expected, "
                f"worst gap {worst_gap * 1000:.0f} ms")

        degraded = [s for s in stages
                    if any(t['rate_ratio'] < 0.8
                           for t in s.get('telemetry', {}).values())]
        if degraded:
            first = degraded[0]
            self.metrics.add_finding(
                f"Downlink drops below 80% of the expected pose rate from "
                f"{first['rate_hz']:g} Hz per drone "
                f"({first['offered_msgs_per_s']:.0f} msg/s fleet-wide) upwards")
        else:
            self.metrics.add_finding(
                'Downlink held up at every streamed rate tested')

        unsustainable = [s for s in stages
                         if s['pacing_overruns'] > 0.1 * max(1, s['ticks'])]
        if unsustainable:
            rates = ', '.join('{:g} Hz'.format(s['rate_hz']) for s in unsustainable)
            self.metrics.add_finding(
                f"The publisher itself could not keep up at {rates} -- treat those "
                f"stages as publisher-limited, not radio-limited")
