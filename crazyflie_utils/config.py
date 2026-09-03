"""Layered YAML configuration for the crazyflie_utils stress tests.

Every knob a stress test exposes -- fleet size, namespaces, takeoff heights,
durations, rates, iteration counts, safety limits -- lives in a YAML file under
``config/``. Scenario code never hardcodes a number: it reads ``params`` that
come from the merged config, whose defaults live in ``DEFAULTS`` below and in
each scenario's ``DEFAULT_PARAMS``.

Layering (later wins, deep-merged key by key):

  1. :data:`DEFAULTS` in this module
  2. ``config/common.yaml``  -- fleet / safety / telemetry / report defaults
  3. the scenario file, e.g. ``config/takeoff_land_cycle.yaml``
  4. ``--set fleet.num_uavs=5`` style command-line overrides

Config files are located by :func:`find_config`, which searches the CWD, a
``config/`` folder next to the CWD, the installed package share directory and
the source checkout, so the same command works from a colcon workspace, from
the source tree, or with an absolute path.
"""

from __future__ import annotations

import copy
import os
from pathlib import Path
from typing import Any, Dict, List, Optional, Sequence

import yaml

try:  # ament is only present inside a sourced ROS 2 environment
    from ament_index_python.packages import get_package_share_directory
    _AMENT_AVAILABLE = True
except ImportError:  # pragma: no cover - exercised only outside ROS
    _AMENT_AVAILABLE = False
    get_package_share_directory = None  # type: ignore[assignment]

PACKAGE_NAME = 'crazyflie_utils'

#: Baseline configuration. Anything not set in YAML falls back to these values,
#: so a scenario file can be as short as a ``test:`` block.
DEFAULTS: Dict[str, Any] = {
    'fleet': {
        # Explicit namespace list. When non-empty this wins over num_uavs and
        # is the only way to talk to a fleet with non-sequential names.
        'namespaces': [],
        # Otherwise namespaces are generated as f"{prefix}{i}" for
        # i in [index_start, index_start + num_uavs).
        'num_uavs': 2,
        'prefix': 'cf',
        'index_start': 1,
        # Namespace the crazyflie_server exposes its broadcast services under.
        'broadcast_namespace': 'all',
    },
    'timing': {
        # Per-call deadline: a service call that has not returned within this
        # many seconds counts as a timeout (the headline failure metric).
        'service_timeout': 3.0,
        # How long to wait at startup for the crazyflie_server services.
        'service_ready_timeout': 4.0,
        # How long to wait for the first pose message from every drone.
        'telemetry_ready_timeout': 4.0,
        # Settling pause after the whole fleet reaches a state.
        'settle_time': 2.0,
        # Pause between arming a drone and commanding its takeoff. The
        # supervisor needs a moment to latch the armed state, and an
        # arm+takeoff sent back to back can race it.
        'arm_delay': 0.5,
    },
    'safety': {
        # Log every command instead of sending it. Exercises config, fleet
        # discovery and reporting without touching a motor.
        'dry_run': False,
        # Hard clamp applied to every commanded height in this package.
        'max_height': 1.5,
        # Abort before flying if any expected service is missing.
        'require_all_services': True,
        # Services pre-flight insists on. Keep this to what the scenarios
        # actually command: plenty of servers (simulators, dummy servers) never
        # advertise 'emergency', and that does not make them untestable.
        # 'emergency' is added automatically when emergency_on_abort is set.
        'required_services': ['takeoff', 'land'],
        # Arm each drone immediately before its own takeoff. Firmware with the
        # supervisor enabled refuses to fly unarmed, and an unarmed takeoff
        # looks exactly like a lost command, so this defaults to 'auto':
        #   'auto'   arm every drone that advertises <ns>/arm, skip the rest
        #   True     arming is mandatory; missing service or failure aborts
        #   False    never arm
        'arm_before_flight': 'auto',
        # Disarm during the shutdown/abort path, after the landing has had
        # time to finish. Off by default: disarming a drone that is still in
        # the air drops it.
        'disarm_after_flight': False,
        # On Ctrl-C / unhandled error: land the fleet before exiting.
        'land_on_abort': True,
        'abort_land_height': 0.0,
        'abort_land_duration': 2.0,
        # Cut motors instead of landing. Drones fall out of the sky - only for
        # a caged/tethered setup.
        'emergency_on_abort': False,
        # Abort the run if any drone's telemetry goes silent for this long
        # (seconds). 0 disables the watchdog.
        'telemetry_stall_abort': 0.0,
    },
    'telemetry': {
        # The link monitor is what turns a flight test into a comms test: it
        # measures the downlink (pose/status rates, stalls, RSSI, battery).
        'enabled': True,
        'pose_topic': 'pose',
        'status_topic': 'status',
        # Nominal downlink rate, used to compute "% of expected messages seen".
        'expected_pose_rate': 10.0,
        # A gap larger than this between consecutive pose messages is a stall
        # (i.e. the radio link dropped out for that long).
        'stall_threshold': 0.5,
        # Ring-buffer length per drone for the z-history used by the
        # motion-start detection in the sync/async comparison.
        'history_length': 4000,
        'qos_depth': 10,
    },
    'report': {
        'console': True,
        # Written as <output_dir>/<timestamp>_<scenario><_tag>.{json,csv}
        'output_dir': 'stress_results',
        'formats': ['json', 'csv'],
        'tag': '',
        # Per-call rows in the CSV (can get large for flood tests).
        'per_call_rows': True,
    },
    'test': {
        'type': 'service_ping',
        'params': {},
    },
}


# --------------------------------------------------------------------------
# merging / lookup helpers
# --------------------------------------------------------------------------

def deep_merge(base: Dict[str, Any], override: Dict[str, Any]) -> Dict[str, Any]:
    """Recursively merge ``override`` into ``base``, returning a new dict.

    Dicts merge key by key; every other type (including lists) is replaced
    wholesale so a scenario file can shorten ``fleet.namespaces``.
    """
    result = copy.deepcopy(base)
    for key, value in (override or {}).items():
        if (
            key in result
            and isinstance(result[key], dict)
            and isinstance(value, dict)
        ):
            result[key] = deep_merge(result[key], value)
        else:
            result[key] = copy.deepcopy(value)
    return result


def cfg_get(cfg: Dict[str, Any], dotted: str, default: Any = None) -> Any:
    """Look up ``a.b.c`` in a nested dict, returning ``default`` if absent."""
    node: Any = cfg
    for part in dotted.split('.'):
        if not isinstance(node, dict) or part not in node:
            return default
        node = node[part]
    return node


def set_dotted(cfg: Dict[str, Any], dotted: str, value: Any) -> None:
    """Set ``a.b.c`` in a nested dict, creating intermediate dicts."""
    parts = dotted.split('.')
    node = cfg
    for part in parts[:-1]:
        if not isinstance(node.get(part), dict):
            node[part] = {}
        node = node[part]
    node[parts[-1]] = value


def parse_override(text: str) -> tuple:
    """Parse a ``key.path=value`` override.

    The value is parsed as YAML, so ``3``, ``0.5``, ``true``, ``[cf1, cf2]``
    and ``hover`` all end up with the type you would get from the YAML file.
    """
    if '=' not in text:
        raise ValueError(
            f"Malformed override '{text}', expected key.path=value"
        )
    key, _, raw = text.partition('=')
    return key.strip(), yaml.safe_load(raw)


# --------------------------------------------------------------------------
# file discovery / loading
# --------------------------------------------------------------------------

def config_search_dirs() -> List[Path]:
    """Directories searched for a config file, in priority order."""
    dirs: List[Path] = [
        Path.cwd(),
        Path.cwd() / 'config',
    ]

    env_dir = os.environ.get('CRAZYFLIE_UTILS_CONFIG_DIR')
    if env_dir:
        dirs.append(Path(env_dir))

    # Installed share directory (colcon install space).
    if _AMENT_AVAILABLE:
        try:
            dirs.append(Path(get_package_share_directory(PACKAGE_NAME)) / 'config')
        except Exception:  # package not installed / not sourced
            pass

    # Source checkout: <pkg>/crazyflie_utils/config.py -> <pkg>/config
    dirs.append(Path(__file__).resolve().parents[1] / 'config')
    return dirs


def find_config(name: str) -> Path:
    """Resolve a config file name to a path.

    ``name`` may be an absolute path, a path relative to the CWD, or a bare
    file name such as ``sync_vs_async.yaml`` (``.yaml`` is appended when the
    name has no suffix).
    """
    candidates: List[Path] = []
    raw = Path(name).expanduser()

    if raw.is_absolute():
        candidates.append(raw)
    else:
        names = [raw]
        if raw.suffix == '':
            names = [raw.with_suffix('.yaml'), raw.with_suffix('.yml'), raw]
        for search_dir in config_search_dirs():
            candidates.extend(search_dir / n for n in names)

    for candidate in candidates:
        if candidate.is_file():
            return candidate

    searched = '\n  '.join(str(c) for c in candidates)
    raise FileNotFoundError(
        f"Config '{name}' not found. Searched:\n  {searched}"
    )


def load_yaml_file(path: Path) -> Dict[str, Any]:
    """Load a YAML file, returning ``{}`` for an empty document."""
    with open(path, 'r') as handle:
        data = yaml.safe_load(handle)
    if data is None:
        return {}
    if not isinstance(data, dict):
        raise ValueError(f"{path}: expected a YAML mapping at the top level")
    return data


def load_config(
    scenario: Optional[str] = None,
    common: Optional[str] = 'common.yaml',
    overrides: Optional[Sequence[str]] = None,
) -> Dict[str, Any]:
    """Build the merged configuration.

    Args:
        scenario: Scenario config file (name, relative or absolute path).
            ``None`` runs whatever ``test.type`` the common file specifies.
        common: Shared defaults file. Missing is not an error -- a scenario
            file may carry the whole configuration on its own.
        overrides: ``key.path=value`` strings applied last.

    Returns:
        The merged config dict, with ``_meta.sources`` listing the files used.
    """
    cfg = copy.deepcopy(DEFAULTS)
    sources: List[str] = []

    if common:
        try:
            common_path = find_config(common)
        except FileNotFoundError:
            common_path = None
        if common_path is not None:
            cfg = deep_merge(cfg, load_yaml_file(common_path))
            sources.append(str(common_path))

    if scenario:
        scenario_path = find_config(scenario)
        cfg = deep_merge(cfg, load_yaml_file(scenario_path))
        sources.append(str(scenario_path))

    for override in overrides or []:
        key, value = parse_override(override)
        set_dotted(cfg, key, value)
        sources.append(f"--set {override}")

    cfg['_meta'] = {'sources': sources}
    validate_config(cfg)
    return cfg


# --------------------------------------------------------------------------
# derived values / validation
# --------------------------------------------------------------------------

def resolve_namespaces(cfg: Dict[str, Any]) -> List[str]:
    """Return the drone namespaces this run targets.

    An explicit ``fleet.namespaces`` list wins; otherwise namespaces are
    generated from ``prefix``/``index_start``/``num_uavs`` (``cf1``, ``cf2``
    ... by default). Leading slashes are stripped so callers can build both
    ``/cf1/takeoff`` and relative names without double slashes.
    """
    fleet = cfg.get('fleet', {})
    explicit = fleet.get('namespaces') or []
    if explicit:
        names = [str(n).strip().strip('/') for n in explicit]
    else:
        count = int(fleet.get('num_uavs', 0))
        start = int(fleet.get('index_start', 1))
        prefix = str(fleet.get('prefix', 'cf'))
        names = [f"{prefix}{i}" for i in range(start, start + count)]

    if not names:
        raise ValueError(
            'No drones configured: set fleet.namespaces or fleet.num_uavs'
        )
    duplicates = {n for n in names if names.count(n) > 1}
    if duplicates:
        raise ValueError(f"Duplicate namespaces in fleet config: {sorted(duplicates)}")
    return names


#: Resolved values of ``safety.arm_before_flight``.
ARM_MODES = ('auto', 'always', 'never')


def arming_mode(cfg: Dict[str, Any]) -> str:
    """Resolve ``safety.arm_before_flight`` to one of :data:`ARM_MODES`.

    Accepts the YAML spellings people actually write: ``auto``, ``true``,
    ``false``, ``always``, ``never``, ``yes``, ``no``.
    """
    setting = cfg_get(cfg, 'safety.arm_before_flight', 'auto')
    if isinstance(setting, bool):
        return 'always' if setting else 'never'
    text = str(setting).strip().lower()
    if text in ('true', 'yes', 'always'):
        return 'always'
    if text in ('false', 'no', 'never', 'off'):
        return 'never'
    if text == 'auto':
        return 'auto'
    raise ValueError(
        f"safety.arm_before_flight must be one of {ARM_MODES} (or true/false), "
        f"got {setting!r}")


def clamp_height(cfg: Dict[str, Any], height: float) -> float:
    """Clamp a commanded height to ``safety.max_height`` (and >= 0)."""
    max_height = float(cfg_get(cfg, 'safety.max_height', 1.5))
    return max(0.0, min(float(height), max_height))


def validate_config(cfg: Dict[str, Any]) -> None:
    """Fail fast on configurations that cannot produce a meaningful run."""
    resolve_namespaces(cfg)
    arming_mode(cfg)

    if float(cfg_get(cfg, 'timing.service_timeout', 0.0)) <= 0.0:
        raise ValueError('timing.service_timeout must be > 0')
    if float(cfg_get(cfg, 'safety.max_height', 0.0)) <= 0.0:
        raise ValueError('safety.max_height must be > 0')

    formats = cfg_get(cfg, 'report.formats', []) or []
    unknown = set(formats) - {'json', 'csv'}
    if unknown:
        raise ValueError(f"Unknown report.formats entries: {sorted(unknown)}")

    test_type = cfg_get(cfg, 'test.type')
    if not test_type:
        raise ValueError('test.type is not set')
