"""Tests for the metric aggregation (no ROS required)."""

import math

import pytest

from crazyflie_utils.metrics import (
    CallRecord, MetricsCollector, TelemetryWindow, latency_stats, percentile,
)


def _record(action='takeoff', target='cf1', phase='cycle_1', latency=0.01,
            success=True, error=None):
    return CallRecord(action=action, target=target, phase=phase, t_start=0.0,
                      latency=latency, success=success, error=error)


def test_percentile_uses_nearest_rank():
    values = [1.0, 2.0, 3.0, 4.0, 5.0]
    assert percentile(values, 0) == 1.0
    assert percentile(values, 50) == 3.0
    assert percentile(values, 100) == 5.0
    assert math.isnan(percentile([], 95))


def test_latency_stats_are_reported_in_milliseconds():
    stats = latency_stats([0.001, 0.002, 0.003])
    assert stats['count'] == 3
    assert stats['min_ms'] == pytest.approx(1.0)
    assert stats['max_ms'] == pytest.approx(3.0)
    assert stats['mean_ms'] == pytest.approx(2.0)
    assert stats['median_ms'] == pytest.approx(2.0)


def test_latency_stats_of_nothing_is_nan_not_a_crash():
    stats = latency_stats([])
    assert stats['count'] == 0
    assert math.isnan(stats['median_ms'])


def test_summary_groups_by_action_target_and_phase():
    collector = MetricsCollector('demo', {'fleet': {'num_uavs': 2}})
    collector.add_call(_record(action='takeoff', target='cf1', latency=0.010))
    collector.add_call(_record(action='takeoff', target='cf2', latency=0.020))
    collector.add_call(_record(action='land', target='cf1', latency=0.030,
                               success=False, error='timeout'))

    summary = collector.summary()
    overall = summary['calls']['overall']
    assert overall['total'] == 3
    assert overall['failed'] == 1
    assert overall['failure_rate'] == pytest.approx(1 / 3)
    assert overall['errors'] == {'timeout': 1}
    # Failed calls must not pollute the latency distribution.
    assert overall['latency']['count'] == 2

    assert summary['calls']['by_action']['takeoff']['total'] == 2
    assert summary['calls']['by_target']['cf1']['total'] == 2
    assert summary['calls']['by_phase']['cycle_1']['total'] == 3


def test_telemetry_and_events_land_in_the_summary():
    collector = MetricsCollector('demo', {})
    collector.add_telemetry(TelemetryWindow(
        namespace='cf1', label='cycle_1', duration=10.0, pose_count=95,
        pose_rate=9.5, expected_rate=10.0, rate_ratio=0.95, max_gap=0.3,
        stalls=0, stalled_time=0.0))
    collector.add_event('window_start', label='cycle_1')
    collector.add_finding('all good')
    collector.set_metric('cycles', [{'cycle': 1}])

    summary = collector.summary()
    assert summary['telemetry']['cf1'][0]['pose_count'] == 95
    assert summary['events'][0]['name'] == 'window_start'
    assert summary['findings'] == ['all good']
    assert summary['scenario_metrics']['cycles'] == [{'cycle': 1}]


def test_call_rows_flatten_extras_for_csv():
    collector = MetricsCollector('demo', {})
    record = _record()
    record.extra = {'height': 0.5}
    collector.add_call(record)
    rows = collector.call_rows()
    assert rows[0]['extra_height'] == 0.5
    assert 'extra' not in rows[0]
