"""Console, JSON and CSV rendering of a stress-test result.

The console report is the thing you read while standing next to the drones;
the JSON is the full result (every aggregate, every event, the config that
produced it) and the CSV holds one row per service call for plotting.
"""

from __future__ import annotations

import csv
import json
import math
import os
import time
from pathlib import Path
from typing import Any, Dict, Iterable, List, Sequence

from crazyflie_utils.colors import bold, cyan, dim, green, red, yellow


# --------------------------------------------------------------------------
# formatting helpers
# --------------------------------------------------------------------------

def _is_nan(value: Any) -> bool:
    return isinstance(value, float) and math.isnan(value)


def fmt(value: Any, spec: str = '.1f', missing: str = '-') -> str:
    """Format a number, rendering ``None``/NaN as ``-``."""
    if value is None or _is_nan(value):
        return missing
    try:
        return format(value, spec)
    except (TypeError, ValueError):
        return str(value)


def render_table(headers: Sequence[str], rows: Iterable[Sequence[str]],
                 indent: str = '  ') -> List[str]:
    """Render a fixed-width table as a list of lines."""
    headers = list(headers)
    rows = [[str(cell) for cell in row] for row in rows]
    if not rows:
        return [f"{indent}{dim('(no data)')}"]
    # Ragged rows are padded / the header grown so a formatting slip in a
    # scenario cannot crash the report at the end of a flight.
    width_count = max([len(headers)] + [len(row) for row in rows])
    headers += [''] * (width_count - len(headers))
    rows = [row + [''] * (width_count - len(row)) for row in rows]

    widths = [len(h) for h in headers]
    for row in rows:
        for i, cell in enumerate(row):
            widths[i] = max(widths[i], len(cell))

    def line(cells: Sequence[str]) -> str:
        padded = [cell.ljust(widths[i]) for i, cell in enumerate(cells)]
        return indent + '  '.join(padded).rstrip()

    out = [line(headers), indent + '  '.join('-' * w for w in widths)]
    out.extend(line(row) for row in rows)
    return out


def _health_colour(failure_rate: float, text: str) -> str:
    if failure_rate <= 0.0:
        return green(text)
    if failure_rate < 0.02:
        return yellow(text)
    return red(text)


# --------------------------------------------------------------------------
# console report
# --------------------------------------------------------------------------

def format_console_report(summary: Dict[str, Any]) -> str:
    """Build the full console report for a finished run."""
    lines: List[str] = []
    calls = summary.get('calls', {})
    overall = calls.get('overall', {})

    lines.append('')
    lines.append(bold('=' * 78))
    title = f"CRAZYFLIE STRESS TEST - {summary.get('scenario', '?')}"
    lines.append(bold(title))
    lines.append(bold('=' * 78))

    started = summary.get('started')
    started_text = (time.strftime('%Y-%m-%d %H:%M:%S', time.localtime(started))
                    if started else '?')
    fleet = (summary.get('config', {}).get('fleet') or {})
    lines.append(f"started   : {started_text}")
    lines.append(f"duration  : {fmt(summary.get('duration_s'), '.1f')} s")
    lines.append(f"drones    : {', '.join(_namespaces(summary)) or '-'}")
    lines.append(f"broadcast : /{fleet.get('broadcast_namespace', 'all')}")
    if (summary.get('config', {}).get('safety') or {}).get('dry_run'):
        lines.append(yellow('mode      : DRY RUN - no commands were sent'))
    if summary.get('aborted'):
        lines.append(red('status    : ABORTED'))
    sources = summary.get('config_sources') or []
    if sources:
        lines.append(f"config    : {'; '.join(sources)}")

    # -- uplink --------------------------------------------------------
    lines.append('')
    lines.append(bold('UPLINK (service calls)'))
    total = overall.get('total', 0)
    failed = overall.get('failed', 0)
    rate = overall.get('failure_rate', 0.0)
    latency = overall.get('latency', {})
    headline = (f"{total} calls, {failed} failed ({rate * 100:.2f}%), "
                f"median {fmt(latency.get('median_ms'))} ms, "
                f"p95 {fmt(latency.get('p95_ms'))} ms, "
                f"max {fmt(latency.get('max_ms'))} ms")
    lines.append('  ' + _health_colour(rate, headline))
    if overall.get('errors'):
        lines.append('  errors: ' + ', '.join(
            f"{k}={v}" for k, v in sorted(overall['errors'].items())))

    lines.append('')
    lines.append('  by action:')
    lines.extend(_bucket_table(calls.get('by_action', {}), 'action'))
    lines.append('')
    lines.append('  by target:')
    lines.extend(_bucket_table(calls.get('by_target', {}), 'target'))

    phases = calls.get('by_phase', {})
    if 1 < len(phases) <= 40:
        lines.append('')
        lines.append('  by phase:')
        lines.extend(_bucket_table(phases, 'phase'))

    # -- downlink ------------------------------------------------------
    telemetry = summary.get('telemetry') or {}
    if telemetry:
        lines.append('')
        lines.append(bold('DOWNLINK (telemetry per drone)'))
        lines.extend(_telemetry_table(telemetry))

    # -- scenario specifics --------------------------------------------
    metrics = summary.get('scenario_metrics') or {}
    scenario_lines = _scenario_section(metrics)
    if scenario_lines:
        lines.append('')
        lines.append(bold('SCENARIO METRICS'))
        lines.extend(scenario_lines)

    # -- verdict -------------------------------------------------------
    findings = summary.get('findings') or []
    if findings:
        lines.append('')
        lines.append(bold('FINDINGS'))
        for finding in findings:
            marker = red('  ! ') if finding.startswith('ABORTED') else cyan('  * ')
            lines.append(marker + finding)

    lines.append(bold('=' * 78))
    return '\n'.join(lines)


def _namespaces(summary: Dict[str, Any]) -> List[str]:
    targets = list((summary.get('calls', {}).get('by_target') or {}).keys())
    fleet = (summary.get('config', {}).get('fleet') or {})
    broadcast = fleet.get('broadcast_namespace', 'all')
    drones = [t for t in targets if t != broadcast]
    if drones:
        return sorted(drones)
    return sorted((summary.get('telemetry') or {}).keys())


def _bucket_table(buckets: Dict[str, Dict[str, Any]], label: str) -> List[str]:
    rows = []
    for key in sorted(buckets):
        bucket = buckets[key]
        latency = bucket.get('latency', {})
        errors = bucket.get('errors') or {}
        rows.append([
            key,
            str(bucket.get('total', 0)),
            str(bucket.get('failed', 0)),
            f"{bucket.get('failure_rate', 0.0) * 100:.1f}%",
            fmt(latency.get('median_ms')),
            fmt(latency.get('p95_ms')),
            fmt(latency.get('max_ms')),
            ', '.join(f"{k}={v}" for k, v in sorted(errors.items())) or '-',
        ])
    return render_table(
        [label, 'calls', 'fail', 'fail%', 'med ms', 'p95 ms', 'max ms', 'errors'],
        rows, indent='    ')


def _telemetry_table(telemetry: Dict[str, List[Dict[str, Any]]]) -> List[str]:
    rows = []
    for ns in sorted(telemetry):
        windows = telemetry[ns]
        if not windows:
            continue
        duration = sum(w['duration'] for w in windows)
        poses = sum(w['pose_count'] for w in windows)
        rate = poses / duration if duration > 0 else float('nan')
        expected = windows[0].get('expected_rate') or float('nan')
        ratio = (rate / expected) if expected and not _is_nan(expected) else float('nan')
        rssis = [w['rssi_mean'] for w in windows if w.get('rssi_mean') is not None]
        batteries = [w['battery_start'] for w in windows
                     if w.get('battery_start') is not None]
        batteries += [w['battery_end'] for w in windows
                      if w.get('battery_end') is not None]
        rows.append([
            ns,
            str(poses),
            fmt(rate, '.1f'),
            fmt(ratio * 100 if not _is_nan(ratio) else ratio, '.0f') + '%',
            fmt(max(w['max_gap'] for w in windows) * 1000, '.0f'),
            str(sum(w['stalls'] for w in windows)),
            fmt(sum(rssis) / len(rssis), '.0f') if rssis else '-',
            (f"{batteries[0]:.2f}->{batteries[-1]:.2f}" if len(batteries) >= 2 else '-'),
        ])
    return render_table(
        ['drone', 'poses', 'Hz', '%exp', 'max gap ms', 'stalls', 'rssi', 'battery V'],
        rows, indent='    ')


def _scenario_section(metrics: Dict[str, Any]) -> List[str]:
    """Render the scenario-specific aggregates we know how to tabulate."""
    lines: List[str] = []

    comparison = metrics.get('comparison')
    if isinstance(comparison, dict) and comparison:
        lines.append('  dispatch modes:')
        rows = []
        for mode in sorted(comparison):
            row = comparison[mode]
            rows.append([
                mode,
                str(row.get('trials', 0)),
                str(row.get('calls_per_trial', 0)),
                fmt(row.get('median_latency_ms')),
                fmt(row.get('mean_uplink_span_ms'), '.0f'),
                fmt(row.get('mean_skew_ms'), '.0f'),
                fmt(row.get('worst_skew_ms'), '.0f'),
                str(row.get('failures', 0)),
                str(row.get('airborne_misses', 0)),
            ])
        lines.extend(render_table(
            ['mode', 'trials', 'calls', 'med ms', 'span ms', 'skew ms',
             'worst skew', 'fails', 'missed'], rows, indent='    '))

    stages = metrics.get('stages')
    if isinstance(stages, list) and stages:
        lines.append('  stages:')
        rows = []
        for stage in stages:
            latency = stage.get('latency') or {}
            telemetry = stage.get('telemetry') or {}
            ratios = [t['rate_ratio'] for t in telemetry.values()] or [float('nan')]
            rows.append([
                fmt(stage.get('rate_hz'), 'g'),
                str(stage.get('calls', stage.get('published', 0))),
                fmt(stage.get('offered_msgs_per_s', stage.get('offered_calls_per_s')), '.0f'),
                fmt(stage.get('achieved_rate_hz'), '.1f'),
                f"{stage.get('failure_rate', 0.0) * 100:.1f}%" if 'failure_rate' in stage else '-',
                fmt(latency.get('median_ms')) if latency else '-',
                fmt(latency.get('p95_ms')) if latency else '-',
                fmt(min(ratios) * 100, '.0f') + '%' if telemetry else '-',
                str(stage.get('pacing_overruns', 0)),
            ])
        lines.extend(render_table(
            ['rate Hz', 'msgs', 'offered/s', 'achieved Hz', 'fail%', 'med ms',
             'p95 ms', 'downlink', 'overruns'], rows, indent='    '))

    cycles = metrics.get('cycles')
    if isinstance(cycles, list) and cycles:
        lines.append('  cycles:')
        rows = []
        for cycle in cycles:
            takeoff = cycle.get('takeoff') or {}
            land = cycle.get('land') or {}
            rows.append([
                str(cycle.get('cycle')),
                fmt(cycle.get('height'), '.2f'),
                fmt(takeoff.get('median_ms')),
                fmt(takeoff.get('max_ms')),
                str(cycle.get('takeoff_failures', 0)),
                fmt(land.get('median_ms')),
                str(cycle.get('land_failures', 0)),
                str(cycle.get('airborne_count', 0)),
                fmt(cycle.get('airborne_spread_s', 0.0) * 1000, '.0f'),
                ', '.join(cycle.get('airborne_missing') or []) or '-',
            ])
        lines.extend(render_table(
            ['#', 'height', 'TO med', 'TO max', 'TO fail', 'LD med', 'LD fail',
             'airborne', 'skew ms', 'missing'], rows, indent='    '))

    for key in ('ping_latency_ms', 'batch_span_ms'):
        stats = metrics.get(key)
        if isinstance(stats, dict) and stats.get('count'):
            lines.append(
                f"  {key}: n={stats['count']}, min {fmt(stats['min_ms'])}, "
                f"median {fmt(stats['median_ms'])}, p95 {fmt(stats['p95_ms'])}, "
                f"max {fmt(stats['max_ms'])}")

    return lines


# --------------------------------------------------------------------------
# artefacts
# --------------------------------------------------------------------------

def write_reports(summary: Dict[str, Any], call_rows: List[Dict[str, Any]],
                  report_cfg: Dict[str, Any]) -> List[Path]:
    """Write the configured artefacts, returning the paths written."""
    formats = report_cfg.get('formats') or []
    if not formats:
        return []

    out_dir = Path(os.path.expanduser(str(report_cfg.get('output_dir', 'stress_results'))))
    out_dir.mkdir(parents=True, exist_ok=True)

    tag = str(report_cfg.get('tag') or '').strip()
    stamp = time.strftime('%Y%m%d_%H%M%S', time.localtime(summary.get('started') or time.time()))
    base = f"{stamp}_{summary.get('scenario', 'run')}" + (f"_{tag}" if tag else '')
    written: List[Path] = []

    if 'json' in formats:
        path = out_dir / f"{base}.json"
        with open(path, 'w') as handle:
            # NaN is not valid JSON, so missing statistics become null.
            json.dump(_sanitize(summary), handle, indent=2,
                      default=_json_default, allow_nan=False)
        written.append(path)

    if 'csv' in formats and report_cfg.get('per_call_rows', True) and call_rows:
        path = out_dir / f"{base}_calls.csv"
        fieldnames: List[str] = []
        for row in call_rows:
            for key in row:
                if key not in fieldnames:
                    fieldnames.append(key)
        with open(path, 'w', newline='') as handle:
            writer = csv.DictWriter(handle, fieldnames=fieldnames)
            writer.writeheader()
            writer.writerows(call_rows)
        written.append(path)

    return written


def _sanitize(value: Any) -> Any:
    """Recursively replace NaN/inf with ``None`` so the JSON stays valid."""
    if isinstance(value, dict):
        return {str(k): _sanitize(v) for k, v in value.items()}
    if isinstance(value, (list, tuple)):
        return [_sanitize(v) for v in value]
    if isinstance(value, float) and not math.isfinite(value):
        return None
    return value


def _json_default(value: Any) -> Any:
    if isinstance(value, Path):
        return str(value)
    return str(value)
