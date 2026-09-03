"""Sustained high-level command flood while the fleet is airborne.

Takes the fleet up, then fires ``go_to`` at a configurable rate for a
configurable duration. Each tick sends one command per drone (or one
broadcast), so the offered uplink load is ``rate_hz * num_uavs`` service calls
per second -- the same shape of traffic a swarm controller produces, only
without the controller.

What it exposes:

* the rate at which service calls start timing out (the point where the radio
  or the server can no longer keep up),
* whether downlink telemetry degrades while the uplink is saturated,
* whether the drones stay where they are supposed to be.

The default ``hold`` pattern commands a *relative* ``(0, 0, 0)`` move, so the
drones hover in place and the test isolates pure command throughput. Switch to
``square`` or ``circle`` when you want the flight controller loaded too.
"""

from __future__ import annotations

import math
import time
from typing import Any, Dict, List, Tuple

from crazyflie_utils.metrics import latency_stats
from crazyflie_utils.scenarios.base import StressScenario


class GoToFloodScenario(StressScenario):
    """Flood the fleet with go_to commands and watch the link give out."""

    name = 'goto_flood'
    description = ('Saturate the uplink with go_to commands while flying and '
                   'measure timeouts and telemetry degradation')

    DEFAULT_PARAMS: Dict[str, Any] = {
        'takeoff_height': 0.5,
        'takeoff_duration': 2.0,
        'takeoff_mode': 'broadcast',
        'settle_time': 3.0,
        # Command rate per drone. A list runs consecutive stages, which is the
        # useful way to find the breaking point: [2, 5, 10, 20].
        'rate_hz': [2.0, 5.0, 10.0],
        'stage_duration': 15.0,
        # hold | square | circle
        'pattern': 'hold',
        'radius': 0.25,
        'goto_duration': 1.0,
        # parallel | sequential | broadcast (broadcast forces relative moves)
        'mode': 'parallel',
        'relative': True,
        # True  : each tick waits for its responses, so the achieved rate is
        #         capped by round-trip latency (a well-behaved client).
        # False : fire and forget -- keep sending at the requested rate and
        #         collect the responses at the end of the stage. This is the
        #         harsher load and the one that actually saturates the radio.
        'await_responses': True,
        'recover_time': 3.0,
        'land_height': 0.0,
        'land_duration': 2.0,
        'land_mode': 'broadcast',
        'airborne_fraction': 0.6,
        'airborne_timeout': 6.0,
        'log_every': 25,
    }

    def run(self) -> None:
        height = self.clamp(float(self.params['takeoff_height']))
        rates = self.params['rate_hz']
        rates = [float(r) for r in (rates if isinstance(rates, (list, tuple)) else [rates])]
        pattern = str(self.params['pattern'])
        mode = str(self.params['mode'])
        relative = bool(self.params['relative'])

        if mode == 'broadcast' and not relative:
            raise ValueError(
                'goto_flood: broadcast go_to sends the same goal to every drone; '
                'set relative: true (or use mode: parallel)')
        if pattern not in ('hold', 'square', 'circle'):
            raise ValueError(f"Unknown pattern '{pattern}'")

        self.info(
            f"goto_flood: takeoff to {height:.2f} m, then stages {rates} Hz x "
            f"{self.params['stage_duration']} s, pattern={pattern}, mode={mode}")

        # -- get airborne --------------------------------------------------
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
            self.warn(f"Not airborne before flood: {', '.join(missing)}")
        self.sleep(float(self.params['settle_time']))

        # -- flood stages --------------------------------------------------
        stages: List[Dict[str, Any]] = []
        try:
            for rate in rates:
                self.check_abort()
                self.check_link_health()
                phase = f"flood_{rate:g}hz"
                with self.measure(phase):
                    stage = self._flood_stage(rate, phase, height, pattern, mode,
                                              relative)
                stages.append(stage)
                self.info(
                    f"  {rate:g} Hz: sent {stage['calls']} call(s), "
                    f"{stage['failures']} failed ({stage['failure_rate'] * 100:.1f}%), "
                    f"median {stage['latency']['median_ms']:.1f} ms, achieved "
                    f"{stage['achieved_rate_hz']:.1f} Hz")
                # Let the fleet (and the radio) recover between stages.
                self.sleep(float(self.params['recover_time']))
        finally:
            with self.measure('land'):
                self.land_fleet(
                    str(self.params['land_mode']),
                    float(self.params['land_height']),
                    float(self.params['land_duration']), phase='land')

        self.metrics.set_metric('stages', stages)
        self._add_findings(stages)

    # -- one stage ---------------------------------------------------------

    def _flood_stage(self, rate: float, phase: str, height: float, pattern: str,
                     mode: str, relative: bool) -> Dict[str, Any]:
        duration = float(self.params['stage_duration'])
        goto_duration = float(self.params['goto_duration'])
        log_every = max(1, int(self.params['log_every']))
        period = 1.0 / rate if rate > 0 else 0.0

        await_responses = bool(self.params['await_responses'])
        start = time.perf_counter()
        tick = 0
        overruns = 0
        records = []
        in_flight = []

        while time.perf_counter() - start < duration:
            self.check_abort()
            x, y, z = self._waypoint(tick, pattern, height, relative)
            if await_responses:
                records.extend(self.go_to_fleet(
                    mode, x, y, z, yaw=0.0, duration=goto_duration,
                    relative=relative, phase=phase))
            else:
                in_flight.extend(self.dispatch_fleet_async(
                    'go_to', mode, phase, x=x, y=y, z=z, yaw=0.0,
                    duration=goto_duration, relative=relative))
            tick += 1

            if tick % log_every == 0:
                self.check_link_health()

            if period:
                next_tick = start + tick * period
                slack = next_tick - time.perf_counter()
                if slack > 0:
                    self.sleep(slack)
                else:
                    # We could not keep up with the requested rate: the calls
                    # themselves are taking longer than the period.
                    overruns += 1

        elapsed = time.perf_counter() - start
        if in_flight:
            # Fire-and-forget mode: collect everything now. Each call keeps
            # its own deadline, so a response that arrived during the stage is
            # still timed correctly.
            records.extend(self.fleet.gather(in_flight, timeout=self.service_timeout))
        ok = [r for r in records if r.success]
        failures = len(records) - len(ok)

        return {
            'rate_hz': rate,
            'ticks': tick,
            'await_responses': await_responses,
            'calls': len(records),
            'failures': failures,
            'failure_rate': (failures / len(records)) if records else 0.0,
            'errors': self._error_counts(records),
            'latency': latency_stats([r.latency for r in ok]),
            'requested_rate_hz': rate,
            'achieved_rate_hz': (tick / elapsed) if elapsed > 0 else 0.0,
            'offered_calls_per_s': (len(records) / elapsed) if elapsed > 0 else 0.0,
            'pacing_overruns': overruns,
            'duration_s': elapsed,
        }

    def _waypoint(self, tick: int, pattern: str, height: float,
                  relative: bool) -> Tuple[float, float, float]:
        """Goal for this tick. Relative goals are offsets from where the drone is."""
        radius = float(self.params['radius'])
        if pattern == 'hold':
            return (0.0, 0.0, 0.0) if relative else (0.0, 0.0, height)
        if pattern == 'square':
            # Offsets that sum to zero every four ticks, so a relative square
            # keeps the fleet around its starting point.
            offsets = [(radius, 0.0), (0.0, radius), (-radius, 0.0), (0.0, -radius)]
            dx, dy = offsets[tick % 4]
            return (dx, dy, 0.0) if relative else (dx, dy, height)
        angle = (tick % 36) * (2.0 * math.pi / 36.0)
        dx, dy = radius * math.cos(angle), radius * math.sin(angle)
        if relative:
            # Chord between consecutive points, so the drone traces the circle
            # instead of jumping back to the same offset every tick.
            prev = ((tick - 1) % 36) * (2.0 * math.pi / 36.0)
            dx -= radius * math.cos(prev)
            dy -= radius * math.sin(prev)
            return dx, dy, 0.0
        return dx, dy, height

    @staticmethod
    def _error_counts(records) -> Dict[str, int]:
        counts: Dict[str, int] = {}
        for record in records:
            if not record.success:
                key = record.error or 'unknown'
                counts[key] = counts.get(key, 0) + 1
        return counts

    # -- verdict -----------------------------------------------------------

    def _add_findings(self, stages: List[Dict[str, Any]]) -> None:
        if not stages:
            return
        for stage in stages:
            self.metrics.add_finding(
                f"{stage['rate_hz']:g} Hz/drone ({stage['offered_calls_per_s']:.1f} "
                f"calls/s offered): {stage['failure_rate'] * 100:.1f}% failed, "
                f"median {stage['latency']['median_ms']:.1f} ms, p95 "
                f"{stage['latency']['p95_ms']:.1f} ms")

        breaking = next((s for s in stages if s['failure_rate'] > 0.02), None)
        if breaking:
            self.metrics.add_finding(
                f"Link starts dropping commands at {breaking['rate_hz']:g} Hz per "
                f"drone ({breaking['offered_calls_per_s']:.1f} calls/s across the "
                f"fleet)")
        else:
            self.metrics.add_finding(
                f"No stage exceeded a 2% failure rate; the link kept up with all "
                f"tested rates (max {max(s['offered_calls_per_s'] for s in stages):.1f} "
                f"calls/s)")

        saturated = [s for s in stages if s['pacing_overruns'] > 0.1 * max(1, s['ticks'])]
        if saturated:
            rates = ', '.join('{:g} Hz'.format(s['rate_hz']) for s in saturated)
            self.metrics.add_finding(
                f"Requested rate could not be sustained at {rates} -- the calls "
                f"themselves took longer than the tick period")
