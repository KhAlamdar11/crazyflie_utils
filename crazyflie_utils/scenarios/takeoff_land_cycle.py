"""Back-to-back takeoff/land cycles -- the bread-and-butter endurance test.

Runs ``cycles`` iterations of *takeoff -> hover -> land -> sit on the ground*
across the whole fleet and, for each one, records:

* the round-trip latency of every takeoff and land call,
* whether every drone actually left the ground (from ``<ns>/pose``), which is
  how a silently dropped command shows up,
* how far apart the drones started moving,
* the downlink rate, dropouts, RSSI and battery sag during the cycle.

Consecutive cycles are where link problems accumulate: radio congestion, a
server whose per-drone thread wedges, or a drone that stops answering after
the fifth landing. The per-cycle breakdown in the report is there so you can
see *when* it started degrading, not just that it did.
"""

from __future__ import annotations

import time
from typing import Any, Dict, List

from crazyflie_utils.metrics import latency_stats
from crazyflie_utils.scenarios.base import StressScenario


class TakeoffLandCycleScenario(StressScenario):
    """N consecutive takeoff/land cycles with per-cycle comms metrics."""

    name = 'takeoff_land_cycle'
    description = ('Consecutive takeoff/hover/land cycles with per-cycle latency, '
                   'command-loss and telemetry metrics')

    DEFAULT_PARAMS: Dict[str, Any] = {
        'cycles': 10,
        # Dispatch style for the fleet commands:
        # broadcast | parallel | sequential
        'mode': 'broadcast',
        # Same modes; 'same' reuses whatever `mode` is.
        'land_mode': 'same',
        'takeoff_height': 0.5,
        'takeoff_duration': 2.0,
        # Cycle through this list of heights instead of takeoff_height when
        # non-empty, e.g. [0.4, 0.8, 0.4, 1.0].
        'height_sequence': [],
        'hover_time': 3.0,
        'land_height': 0.0,
        'land_duration': 2.0,
        'ground_time': 3.0,
        # Delay between per-drone calls in parallel/sequential mode.
        'stagger': 0.0,
        # A drone counts as airborne once it reports this fraction of the
        # commanded height.
        'airborne_fraction': 0.6,
        'airborne_timeout': 6.0,
        # Below this altitude the drone counts as landed at the end of a cycle.
        'landed_height': 0.1,
    }

    def run(self) -> None:
        cycles = int(self.params['cycles'])
        mode = str(self.params['mode'])
        land_mode = str(self.params['land_mode'])
        land_mode = mode if land_mode == 'same' else land_mode
        heights = [float(h) for h in (self.params['height_sequence'] or [])]
        stagger = float(self.params['stagger'])

        self.info(
            f"takeoff_land_cycle: {cycles} cycles, mode={mode}/{land_mode}, "
            f"{len(self.namespaces)} drone(s)")

        cycle_rows: List[Dict[str, Any]] = []
        for cycle in range(1, cycles + 1):
            self.check_abort()
            self.check_link_health()
            height = self.clamp(
                heights[(cycle - 1) % len(heights)] if heights
                else float(self.params['takeoff_height']))
            phase = f"cycle_{cycle}"

            with self.measure(phase):
                row = self._one_cycle(cycle, phase, mode, land_mode, height, stagger)
            cycle_rows.append(row)

            self.info(
                f"  cycle {cycle}/{cycles}: takeoff {row['takeoff_summary']}, "
                f"airborne {row['airborne_count']}/{len(self.namespaces)}, "
                f"land {row['land_summary']}")

        self.metrics.set_metric('cycles', cycle_rows)
        self._add_findings(cycle_rows)

    # -- one cycle ---------------------------------------------------------

    def _one_cycle(self, cycle: int, phase: str, mode: str, land_mode: str,
                   height: float, stagger: float) -> Dict[str, Any]:
        # Arming is handled by takeoff_fleet: the supervisor disarms on
        # landing detection, so every cycle re-arms before its takeoff -- in
        # sequential mode, one drone at a time (arm -> arm_delay -> takeoff).
        t_command = time.perf_counter()
        takeoff_records = self.takeoff_fleet(
            mode, height, float(self.params['takeoff_duration']),
            phase=phase, stagger=stagger)

        # Did they actually leave the ground? This is the command-loss check:
        # a takeoff service can return happily while the radio packet never
        # reached the drone.
        threshold = height * float(self.params['airborne_fraction'])
        crossings = self.wait_for_altitude(
            threshold, t_command, float(self.params['airborne_timeout']))
        reached = {ns: t for ns, t in crossings.items() if t is not None}
        missing = sorted(ns for ns, t in crossings.items() if t is None)
        spread = (max(reached.values()) - min(reached.values())) if len(reached) > 1 else 0.0

        remaining_hover = float(self.params['hover_time'])
        self.sleep(max(0.0, remaining_hover))
        hover_heights = self.report_altitudes(f"{phase}_hover")

        land_records = self.land_fleet(
            land_mode, float(self.params['land_height']),
            float(self.params['land_duration']), phase=phase, stagger=stagger)
        self.sleep(float(self.params['ground_time']))

        landed_limit = float(self.params['landed_height'])
        ground_heights = self.report_altitudes(f"{phase}_ground")
        still_up = sorted(
            ns for ns, z in ground_heights.items()
            if z is not None and z > landed_limit)

        if missing:
            self.warn(
                f"  cycle {cycle}: no altitude report above {threshold:.2f} m from "
                f"{', '.join(missing)} -- command lost or drone not connected")
        if still_up:
            self.warn(
                f"  cycle {cycle}: {', '.join(still_up)} still above "
                f"{landed_limit:.2f} m after landing")

        return {
            'cycle': cycle,
            'height': height,
            'takeoff': latency_stats([r.latency for r in takeoff_records if r.success]),
            'takeoff_failures': sum(1 for r in takeoff_records if not r.success),
            'takeoff_summary': self.summarise_records(takeoff_records),
            'land': latency_stats([r.latency for r in land_records if r.success]),
            'land_failures': sum(1 for r in land_records if not r.success),
            'land_summary': self.summarise_records(land_records),
            'airborne_count': len(reached),
            'airborne_missing': missing,
            'airborne_delay_s': {ns: t for ns, t in crossings.items()},
            'airborne_spread_s': spread,
            'hover_heights': hover_heights,
            'ground_heights': ground_heights,
            'not_landed': still_up,
        }

    # -- verdict -----------------------------------------------------------

    def _add_findings(self, rows: List[Dict[str, Any]]) -> None:
        if not rows:
            return
        fleet_size = len(self.namespaces)
        lost_cycles = [r['cycle'] for r in rows if r['airborne_missing']]
        stuck_cycles = [r['cycle'] for r in rows if r['not_landed']]
        call_failures = sum(r['takeoff_failures'] + r['land_failures'] for r in rows)

        self.metrics.add_finding(
            f"{len(rows)} cycles completed, {call_failures} failed service call(s)")

        if lost_cycles:
            self.metrics.add_finding(
                f"Drones failed to reach altitude in {len(lost_cycles)}/{len(rows)} "
                f"cycles (cycles {lost_cycles[:10]}) -- takeoff commands are being "
                f"lost or the drones are not connected")
        else:
            self.metrics.add_finding(
                f"All {fleet_size} drone(s) reached altitude in every cycle")

        if stuck_cycles:
            self.metrics.add_finding(
                f"Drones were still airborne after land in cycles {stuck_cycles[:10]}")

        # Degradation over time: compare the first and last third of the run.
        third = max(1, len(rows) // 3)
        early = [r['takeoff']['median_ms'] for r in rows[:third]
                 if r['takeoff']['count']]
        late = [r['takeoff']['median_ms'] for r in rows[-third:]
                if r['takeoff']['count']]
        if early and late:
            first = sum(early) / len(early)
            last = sum(late) / len(late)
            self.metrics.add_finding(
                f"Takeoff latency drift: {first:.1f} ms (first {third} cycles) -> "
                f"{last:.1f} ms (last {third} cycles)")
            if last > 2.0 * first and last - first > 20.0:
                self.metrics.add_finding(
                    'Latency more than doubled over the run -- the link degrades '
                    'with consecutive cycles')

        spreads = [r['airborne_spread_s'] for r in rows if r['airborne_spread_s']]
        if spreads:
            self.metrics.add_finding(
                f"Fleet motion-start spread: mean "
                f"{sum(spreads) / len(spreads) * 1000:.0f} ms, "
                f"worst {max(spreads) * 1000:.0f} ms")
