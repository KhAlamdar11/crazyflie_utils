"""How synchronised is a fleet command, really?

The same takeoff/land cycle is flown once per dispatch style, round after
round, and the styles are compared on two axes:

``uplink cost``
    Round-trip latency and failure rate of the service calls themselves. One
    broadcast call is one radio transaction for the whole fleet; N unicast
    calls are N transactions competing for the same radio.

``fleet skew``
    How far apart the drones actually started moving, measured from the
    ``<ns>/pose`` stream. This is the number that matters for formation
    flight, and it is the one that degrades as you add drones to unicast
    dispatch.

Dispatch styles (``modes`` param):
    ``broadcast``   one ``/all/takeoff`` -- the server sends a single
                    broadcast packet; nominally simultaneous.
    ``parallel``    one ``/cfN/takeoff`` per drone, all dispatched before any
                    response is awaited -- N transactions in flight at once.
    ``sequential``  one ``/cfN/takeoff`` per drone, each awaited before the
                    next is sent -- skew grows with fleet size.

Skew resolution is bounded by the pose rate (100 ms at the default 10 Hz), so
compare modes against each other rather than trusting an absolute number.
"""

from __future__ import annotations

import time
from typing import Any, Dict, List

from crazyflie_utils.metrics import latency_stats
from crazyflie_utils.scenarios.base import DISPATCH_MODES, StressScenario


class SyncVsAsyncScenario(StressScenario):
    """Compare broadcast, parallel-unicast and sequential-unicast dispatch."""

    name = 'sync_vs_async'
    description = ('Compare broadcast vs parallel vs sequential fleet commands '
                   'on latency and real fleet skew')

    DEFAULT_PARAMS: Dict[str, Any] = {
        'rounds': 3,
        'modes': ['broadcast', 'parallel', 'sequential'],
        'takeoff_height': 0.5,
        'takeoff_duration': 2.0,
        'hover_time': 3.0,
        'land_height': 0.0,
        'land_duration': 2.0,
        'ground_time': 4.0,
        'stagger': 0.0,
        'airborne_fraction': 0.6,
        'airborne_timeout': 6.0,
        # Land with the round's own dispatch mode ('same') or always the same
        # way (e.g. 'broadcast') so only the takeoff differs between modes.
        'land_mode': 'same',
    }

    def run(self) -> None:
        rounds = int(self.params['rounds'])
        modes = [str(m) for m in self.params['modes']]
        unknown = set(modes) - set(DISPATCH_MODES)
        if unknown:
            raise ValueError(
                f"Unknown dispatch mode(s) {sorted(unknown)}, expected "
                f"{list(DISPATCH_MODES)}")
        if len(self.namespaces) < 2:
            self.warn('sync_vs_async with a single drone measures latency only; '
                      'fleet skew needs at least two drones')

        height = self.clamp(float(self.params['takeoff_height']))
        self.info(
            f"sync_vs_async: {rounds} round(s) x {modes}, height {height:.2f} m, "
            f"{len(self.namespaces)} drone(s)")

        trials: List[Dict[str, Any]] = []
        for round_index in range(1, rounds + 1):
            for mode in modes:
                self.check_abort()
                self.check_link_health()
                phase = f"round_{round_index}/{mode}"
                with self.measure(phase):
                    trial = self._one_trial(round_index, mode, height, phase)
                trials.append(trial)
                self.info(
                    f"  round {round_index} {mode}: uplink "
                    f"{trial['takeoff_summary']}, skew "
                    f"{self._fmt_ms(trial['skew_s'])}, airborne "
                    f"{trial['airborne_count']}/{len(self.namespaces)}")

        self.metrics.set_metric('trials', trials)
        comparison = self._compare(modes, trials)
        self.metrics.set_metric('comparison', comparison)
        self._add_findings(modes, comparison)

    # -- one trial ---------------------------------------------------------

    def _one_trial(self, round_index: int, mode: str, height: float,
                   phase: str) -> Dict[str, Any]:
        stagger = float(self.params['stagger'])
        land_mode = str(self.params['land_mode'])
        land_mode = mode if land_mode == 'same' else land_mode

        t_command = time.perf_counter()
        takeoff_records = self.takeoff_fleet(
            mode, height, float(self.params['takeoff_duration']),
            phase=phase, stagger=stagger)

        # Uplink view: when did each call get its response?
        completions = [r.t_start + r.latency - t_command for r in takeoff_records]
        uplink_span = (max(completions) - min(completions)) if len(completions) > 1 else 0.0

        # Airframe view: when did each drone actually start climbing?
        crossings = self.wait_for_altitude(
            height * float(self.params['airborne_fraction']), t_command,
            float(self.params['airborne_timeout']))
        reached = {ns: t for ns, t in crossings.items() if t is not None}
        skew = (max(reached.values()) - min(reached.values())) if len(reached) > 1 else 0.0

        self.sleep(float(self.params['hover_time']))
        land_records = self.land_fleet(
            land_mode, float(self.params['land_height']),
            float(self.params['land_duration']), phase=phase, stagger=stagger)
        self.sleep(float(self.params['ground_time']))

        return {
            'round': round_index,
            'mode': mode,
            'calls': len(takeoff_records),
            'takeoff': latency_stats([r.latency for r in takeoff_records if r.success]),
            'takeoff_failures': sum(1 for r in takeoff_records if not r.success),
            'takeoff_summary': self.summarise_records(takeoff_records),
            'land_failures': sum(1 for r in land_records if not r.success),
            'uplink_span_s': uplink_span,
            'skew_s': skew,
            'airborne_count': len(reached),
            'airborne_missing': sorted(ns for ns, t in crossings.items() if t is None),
            'airborne_delay_s': crossings,
        }

    # -- comparison --------------------------------------------------------

    def _compare(self, modes: List[str],
                 trials: List[Dict[str, Any]]) -> Dict[str, Any]:
        comparison: Dict[str, Any] = {}
        for mode in modes:
            rows = [t for t in trials if t['mode'] == mode]
            if not rows:
                continue
            skews = [t['skew_s'] for t in rows if t['airborne_count'] > 1]
            spans = [t['uplink_span_s'] for t in rows]
            latencies = [t['takeoff']['median_ms'] for t in rows if t['takeoff']['count']]
            comparison[mode] = {
                'trials': len(rows),
                'calls_per_trial': rows[0]['calls'],
                'median_latency_ms': (sum(latencies) / len(latencies)) if latencies else float('nan'),
                'mean_uplink_span_ms': (sum(spans) / len(spans) * 1000.0) if spans else float('nan'),
                'mean_skew_ms': (sum(skews) / len(skews) * 1000.0) if skews else float('nan'),
                'worst_skew_ms': (max(skews) * 1000.0) if skews else float('nan'),
                'failures': sum(t['takeoff_failures'] + t['land_failures'] for t in rows),
                'airborne_misses': sum(len(t['airborne_missing']) for t in rows),
            }
        return comparison

    @staticmethod
    def _fmt_ms(seconds: float) -> str:
        return f"{seconds * 1000:.0f} ms"

    def _add_findings(self, modes: List[str], comparison: Dict[str, Any]) -> None:
        for mode in modes:
            row = comparison.get(mode)
            if not row:
                continue
            self.metrics.add_finding(
                f"{mode}: {row['calls_per_trial']} call(s)/trial, median latency "
                f"{row['median_latency_ms']:.1f} ms, uplink span "
                f"{row['mean_uplink_span_ms']:.0f} ms, fleet skew "
                f"{row['mean_skew_ms']:.0f} ms (worst {row['worst_skew_ms']:.0f} ms), "
                f"{row['failures']} failed call(s), {row['airborne_misses']} "
                f"missed takeoff(s)")

        skews = {m: comparison[m]['mean_skew_ms'] for m in comparison
                 if comparison[m]['mean_skew_ms'] == comparison[m]['mean_skew_ms']}
        if len(skews) > 1:
            best = min(skews, key=skews.get)
            worst = max(skews, key=skews.get)
            if skews[worst] > 0:
                self.metrics.add_finding(
                    f"Tightest fleet sync: {best} ({skews[best]:.0f} ms); loosest: "
                    f"{worst} ({skews[worst]:.0f} ms)")
