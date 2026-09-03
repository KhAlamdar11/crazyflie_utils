"""Tests for the layered configuration (no ROS required)."""

import pytest

from crazyflie_utils.config import (
    DEFAULTS, cfg_get, clamp_height, deep_merge, find_config, load_config,
    parse_override, resolve_namespaces, set_dotted, validate_config,
)


def test_deep_merge_is_recursive_and_non_destructive():
    base = {'a': {'b': 1, 'c': 2}, 'list': [1, 2]}
    merged = deep_merge(base, {'a': {'c': 3}, 'list': [9]})
    assert merged == {'a': {'b': 1, 'c': 3}, 'list': [9]}
    assert base == {'a': {'b': 1, 'c': 2}, 'list': [1, 2]}, 'base was mutated'


def test_parse_override_uses_yaml_types():
    assert parse_override('fleet.num_uavs=4') == ('fleet.num_uavs', 4)
    assert parse_override('safety.dry_run=true') == ('safety.dry_run', True)
    assert parse_override('test.params.rate_hz=2.5')[1] == 2.5
    assert parse_override('fleet.namespaces=[cf1, cf2]')[1] == ['cf1', 'cf2']
    with pytest.raises(ValueError):
        parse_override('no_equals_sign')


def test_set_dotted_creates_intermediate_levels():
    cfg = {}
    set_dotted(cfg, 'test.params.cycles', 7)
    assert cfg == {'test': {'params': {'cycles': 7}}}
    assert cfg_get(cfg, 'test.params.cycles') == 7
    assert cfg_get(cfg, 'test.params.missing', 'fallback') == 'fallback'


def test_namespaces_generated_from_count():
    cfg = deep_merge(DEFAULTS, {'fleet': {'num_uavs': 3, 'prefix': 'cf',
                                          'index_start': 1}})
    assert resolve_namespaces(cfg) == ['cf1', 'cf2', 'cf3']


def test_explicit_namespaces_win_and_are_normalised():
    cfg = deep_merge(DEFAULTS, {'fleet': {'namespaces': ['/cf7', 'cf9'],
                                          'num_uavs': 3}})
    assert resolve_namespaces(cfg) == ['cf7', 'cf9']


def test_duplicate_and_empty_fleets_are_rejected():
    with pytest.raises(ValueError):
        resolve_namespaces({'fleet': {'namespaces': ['cf1', 'cf1']}})
    with pytest.raises(ValueError):
        resolve_namespaces({'fleet': {'num_uavs': 0, 'namespaces': []}})


def test_height_is_clamped_to_safety_limit():
    cfg = deep_merge(DEFAULTS, {'safety': {'max_height': 1.0}})
    assert clamp_height(cfg, 2.5) == 1.0
    assert clamp_height(cfg, -1.0) == 0.0
    assert clamp_height(cfg, 0.6) == pytest.approx(0.6)


def test_validate_rejects_bad_values():
    cfg = deep_merge(DEFAULTS, {'timing': {'service_timeout': 0.0}})
    with pytest.raises(ValueError):
        validate_config(cfg)
    cfg = deep_merge(DEFAULTS, {'report': {'formats': ['pdf']}})
    with pytest.raises(ValueError):
        validate_config(cfg)


def test_shipped_scenario_configs_load_and_layer():
    """Every config/*.yaml must merge cleanly on top of common.yaml."""
    for name in ('service_ping', 'takeoff_land_cycle', 'sync_vs_async',
                 'goto_flood', 'setpoint_stream'):
        cfg = load_config(name, 'common.yaml', [])
        assert cfg_get(cfg, 'test.type') == name
        assert resolve_namespaces(cfg)
        # The scenario file must not have dropped the common defaults.
        assert cfg_get(cfg, 'safety.max_height') is not None
        assert cfg_get(cfg, 'timing.service_timeout') is not None


def test_cli_overrides_are_applied_last():
    cfg = load_config('takeoff_land_cycle', 'common.yaml',
                      ['fleet.num_uavs=5', 'test.params.cycles=3'])
    assert resolve_namespaces(cfg) == ['cf1', 'cf2', 'cf3', 'cf4', 'cf5']
    assert cfg_get(cfg, 'test.params.cycles') == 3
    assert any('--set' in source for source in cfg['_meta']['sources'])


def test_missing_config_lists_search_paths():
    with pytest.raises(FileNotFoundError) as excinfo:
        find_config('definitely_not_a_config')
    assert 'Searched' in str(excinfo.value)
