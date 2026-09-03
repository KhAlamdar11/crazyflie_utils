"""Tests for report rendering and artefact writing (no ROS required)."""

import csv
import json
import time

from crazyflie_utils.metrics import CallRecord, MetricsCollector, TelemetryWindow
from crazyflie_utils.report import (
    format_console_report, render_table, write_reports,
)


def _collector():
    collector = MetricsCollector('takeoff_land_cycle', {
        'fleet': {'num_uavs': 2, 'broadcast_namespace': 'all'},
        'safety': {'dry_run': False},
        '_meta': {'sources': ['config/common.yaml']},
    })
    for i, target in enumerate(('cf1', 'cf2')):
        collector.add_call(CallRecord(
            action='takeoff', target=target, phase='cycle_1', t_start=float(i),
            latency=0.01 * (i + 1), success=True))
    collector.add_call(CallRecord(
        action='land', target='cf2', phase='cycle_1', t_start=2.0,
        latency=3.0, success=False, error='timeout'))
    collector.add_telemetry(TelemetryWindow(
        namespace='cf1', label='cycle_1', duration=10.0, pose_count=98,
        pose_rate=9.8, expected_rate=10.0, rate_ratio=0.98, max_gap=0.2,
        stalls=0, stalled_time=0.0, status_count=10, rssi_mean=-55.0,
        battery_start=4.1, battery_end=3.9))
    collector.add_finding('2 cycles completed, 1 failed service call')
    collector.set_metric('cycles', [{
        'cycle': 1, 'height': 0.5,
        'takeoff': {'median_ms': 12.0, 'max_ms': 20.0, 'count': 2},
        'takeoff_failures': 0, 'land': {'median_ms': 11.0, 'count': 1},
        'land_failures': 1, 'airborne_count': 2, 'airborne_spread_s': 0.12,
        'airborne_missing': [],
    }])
    collector.finished_wall = time.time()
    return collector


def test_render_table_pads_ragged_rows():
    lines = render_table(['a', 'b', 'c'], [['1', '2'], ['3', '4', '5', '6']])
    assert len(lines) == 4  # header, rule, two rows
    assert lines[0].strip().startswith('a')


def test_render_table_handles_no_rows():
    assert 'no data' in render_table(['a'], [])[0]


def test_console_report_contains_the_key_numbers():
    text = format_console_report(_collector().summary())
    assert 'CRAZYFLIE STRESS TEST' in text
    assert 'UPLINK (service calls)' in text
    assert 'DOWNLINK (telemetry per drone)' in text
    assert 'timeout=1' in text
    assert 'FINDINGS' in text
    assert 'cf1' in text and 'cf2' in text


def test_dry_run_is_announced_in_the_report():
    collector = _collector()
    collector.config['safety']['dry_run'] = True
    assert 'DRY RUN' in format_console_report(collector.summary())


def test_write_reports_emits_valid_json_and_csv(tmp_path):
    collector = _collector()
    summary = collector.summary()
    written = write_reports(summary, collector.call_rows(), {
        'formats': ['json', 'csv'],
        'output_dir': str(tmp_path),
        'tag': 'unit',
        'per_call_rows': True,
    })
    assert len(written) == 2

    json_path = next(p for p in written if p.suffix == '.json')
    payload = json.loads(json_path.read_text())  # NaN would break this
    assert payload['scenario'] == 'takeoff_land_cycle'
    assert payload['calls']['overall']['failed'] == 1
    assert 'unit' in json_path.name

    csv_path = next(p for p in written if p.suffix == '.csv')
    rows = list(csv.DictReader(csv_path.read_text().splitlines()))
    assert len(rows) == 3
    assert rows[-1]['error'] == 'timeout'


def test_nan_statistics_serialise_as_null(tmp_path):
    """Empty latency buckets are NaN internally; JSON must stay valid."""
    collector = MetricsCollector('service_ping', {})
    collector.add_call(CallRecord(
        action='takeoff', target='cf1', phase='ping', t_start=0.0,
        latency=0.0, success=False, error='timeout'))
    written = write_reports(collector.summary(), collector.call_rows(),
                            {'formats': ['json'], 'output_dir': str(tmp_path)})
    payload = json.loads(written[0].read_text())
    assert payload['calls']['overall']['latency']['median_ms'] is None


def test_no_formats_writes_nothing(tmp_path):
    collector = _collector()
    assert write_reports(collector.summary(), collector.call_rows(),
                         {'formats': [], 'output_dir': str(tmp_path)}) == []
