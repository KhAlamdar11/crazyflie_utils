"""Metric collection for the communication stress tests.

Two things are measured and everything else is derived from them:

* **Uplink** -- every service call made to the crazyflie_server is timed from
  just before ``call_async`` to the moment the response future completes, and
  tagged success / failure / timeout. Round-trip latency here covers the ROS 2
  service hop *plus* the radio round trip to the drone, which is what makes it
  a useful proxy for radio congestion.
* **Downlink** -- see :mod:`crazyflie_utils.link_monitor`, which feeds
  :class:`TelemetryWindow` snapshots into the same result object.

Latency is measured with :func:`time.perf_counter` (monotonic, high
resolution); wall-clock stamps are recorded separately for correlating with
the crazyflie_server log.
"""

from __future__ import annotations

import statistics
import threading
import time
from dataclasses import dataclass, field, asdict
from typing import Any, Dict, Iterable, List, Optional


@dataclass
class CallRecord:
    """One service call: who, what, how long, and did it come back."""

    action: str                 # 'takeoff', 'land', 'go_to', ...
    target: str                 # drone namespace, or the broadcast namespace
    phase: str                  # scenario-defined, e.g. 'cycle_3/sync'
    t_start: float              # perf_counter at dispatch
    latency: float              # seconds; time to the response future
    success: bool
    error: Optional[str] = None  # 'timeout' | 'unavailable' | 'rejected' | ...
    wall_time: float = 0.0      # time.time() at dispatch
    extra: Dict[str, Any] = field(default_factory=dict)

    def as_row(self) -> Dict[str, Any]:
        row = asdict(self)
        extra = row.pop('extra')
        for key, value in extra.items():
            row[f"extra_{key}"] = value
        return row


def percentile(values: List[float], pct: float) -> float:
    """Nearest-rank percentile; ``values`` need not be sorted."""
    if not values:
        return float('nan')
    ordered = sorted(values)
    rank = max(1, min(len(ordered), int(round(pct / 100.0 * len(ordered) + 0.5))))
    return ordered[rank - 1]


def latency_stats(values: Iterable[float]) -> Dict[str, float]:
    """Summary statistics for a list of latencies, in milliseconds."""
    vals = [v * 1000.0 for v in values]
    if not vals:
        return {
            'count': 0, 'min_ms': float('nan'), 'mean_ms': float('nan'),
            'median_ms': float('nan'), 'p95_ms': float('nan'),
            'max_ms': float('nan'), 'stdev_ms': float('nan'),
        }
    return {
        'count': len(vals),
        'min_ms': min(vals),
        'mean_ms': statistics.fmean(vals),
        'median_ms': statistics.median(vals),
        'p95_ms': percentile(vals, 95.0),
        'max_ms': max(vals),
        'stdev_ms': statistics.stdev(vals) if len(vals) > 1 else 0.0,
    }


@dataclass
class TelemetryWindow:
    """Downlink health for one drone over one measurement window."""

    namespace: str
    label: str
    duration: float
    pose_count: int
    pose_rate: float
    expected_rate: float
    rate_ratio: float           # observed / expected, 1.0 = no loss
    max_gap: float              # longest silence between pose messages [s]
    stalls: int                 # gaps longer than telemetry.stall_threshold
    stalled_time: float         # total time spent inside those gaps [s]
    status_count: int = 0
    rssi_mean: Optional[float] = None
    rssi_min: Optional[float] = None
    battery_start: Optional[float] = None
    battery_end: Optional[float] = None

    def to_dict(self) -> Dict[str, Any]:
        return asdict(self)


class MetricsCollector:
    """Thread-safe sink for call records, telemetry windows and events.

    The fleet wrapper writes :class:`CallRecord` s from whichever thread made
    the call, the link monitor writes :class:`TelemetryWindow` s from the
    scenario thread, and :meth:`summary` folds everything into the dict that
    the reporter renders and serialises.
    """

    def __init__(self, scenario: str, config: Dict[str, Any]):
        self.scenario = scenario
        self.config = config
        self.calls: List[CallRecord] = []
        self.telemetry: List[TelemetryWindow] = []
        self.events: List[Dict[str, Any]] = []
        self.findings: List[str] = []
        self.scenario_metrics: Dict[str, Any] = {}
        self.t0 = time.perf_counter()
        self.started_wall = time.time()
        self.finished_wall: Optional[float] = None
        self.aborted = False
        self._lock = threading.Lock()

    # -- ingest ------------------------------------------------------------

    def add_call(self, record: CallRecord) -> None:
        with self._lock:
            self.calls.append(record)

    def add_calls(self, records: Iterable[CallRecord]) -> None:
        with self._lock:
            self.calls.extend(records)

    def add_telemetry(self, window: TelemetryWindow) -> None:
        with self._lock:
            self.telemetry.append(window)

    def add_event(self, name: str, **data: Any) -> None:
        """Record a timestamped scenario event (phase boundaries, aborts...)."""
        with self._lock:
            self.events.append({
                'name': name,
                't': time.perf_counter() - self.t0,
                'wall_time': time.time(),
                **data,
            })

    def add_finding(self, text: str) -> None:
        """Record a human-readable observation for the report's verdict."""
        with self._lock:
            self.findings.append(text)

    def set_metric(self, key: str, value: Any) -> None:
        """Attach a scenario-specific aggregate (e.g. sync spread)."""
        with self._lock:
            self.scenario_metrics[key] = value

    # -- aggregation -------------------------------------------------------

    def _group(self, key) -> Dict[str, List[CallRecord]]:
        grouped: Dict[str, List[CallRecord]] = {}
        for record in self.calls:
            grouped.setdefault(key(record), []).append(record)
        return grouped

    @staticmethod
    def _bucket_summary(records: List[CallRecord]) -> Dict[str, Any]:
        ok = [r for r in records if r.success]
        failed = [r for r in records if not r.success]
        errors: Dict[str, int] = {}
        for record in failed:
            errors[record.error or 'unknown'] = errors.get(record.error or 'unknown', 0) + 1
        summary = {
            'total': len(records),
            'ok': len(ok),
            'failed': len(failed),
            'failure_rate': (len(failed) / len(records)) if records else 0.0,
            'errors': errors,
            'latency': latency_stats([r.latency for r in ok]),
        }
        return summary

    def summary(self) -> Dict[str, Any]:
        """Fold everything collected so far into a serialisable report dict."""
        with self._lock:
            calls = list(self.calls)
            telemetry = list(self.telemetry)
            events = list(self.events)
            findings = list(self.findings)
            scenario_metrics = dict(self.scenario_metrics)

        overall = self._bucket_summary(calls)
        by_action = {k: self._bucket_summary(v)
                     for k, v in self._group(lambda r: r.action).items()}
        by_target = {k: self._bucket_summary(v)
                     for k, v in self._group(lambda r: r.target).items()}
        by_phase = {k: self._bucket_summary(v)
                    for k, v in self._group(lambda r: r.phase).items()}

        telemetry_by_ns: Dict[str, List[Dict[str, Any]]] = {}
        for window in telemetry:
            telemetry_by_ns.setdefault(window.namespace, []).append(window.to_dict())

        duration = (self.finished_wall or time.time()) - self.started_wall

        return {
            'scenario': self.scenario,
            'started': self.started_wall,
            'duration_s': duration,
            'aborted': self.aborted,
            'config': {k: v for k, v in self.config.items() if k != '_meta'},
            'config_sources': (self.config.get('_meta') or {}).get('sources', []),
            'calls': {
                'overall': overall,
                'by_action': by_action,
                'by_target': by_target,
                'by_phase': by_phase,
            },
            'telemetry': telemetry_by_ns,
            'scenario_metrics': scenario_metrics,
            'events': events,
            'findings': findings,
        }

    def call_rows(self) -> List[Dict[str, Any]]:
        """Per-call rows for the CSV artefact."""
        with self._lock:
            return [record.as_row() for record in self.calls]
