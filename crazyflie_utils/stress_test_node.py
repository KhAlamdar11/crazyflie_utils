#!/usr/bin/env python3
"""Entry point for the Crazyflie communication stress tests.

Examples::

    # ground-safe uplink benchmark, no flight
    ros2 run crazyflie_utils stress_test service_ping

    # 20 takeoff/land cycles over 3 drones at 0.6 m
    ros2 run crazyflie_utils stress_test takeoff_land_cycle \\
        --set fleet.num_uavs=3 --set test.params.cycles=20 \\
        --set test.params.takeoff_height=0.6

    # rehearse any config without sending a single command
    ros2 run crazyflie_utils stress_test sync_vs_async --dry-run

The positional argument is either a config file in ``config/`` (with or
without the ``.yaml``) or a bare scenario name; ``--set`` overrides any key in
the merged config using dotted paths.
"""

from __future__ import annotations

import argparse
import signal
import sys
from typing import List, Optional

import rclpy

from crazyflie_utils import colors
from crazyflie_utils.colors import bold, red, yellow
from crazyflie_utils.config import (
    find_config, load_config, resolve_namespaces, set_dotted,
)
from crazyflie_utils.runner import StressTestRunner
from crazyflie_utils.scenarios import scenario_listing


def build_parser() -> argparse.ArgumentParser:
    parser = argparse.ArgumentParser(
        prog='stress_test',
        description='Crazyflie communication stress tests',
        formatter_class=argparse.RawDescriptionHelpFormatter,
        epilog='Every knob lives in the YAML files under config/; '
               'use --set key.path=value for one-off changes.')
    parser.add_argument(
        'scenario', nargs='?', default=None,
        help='config file in config/ (e.g. sync_vs_async[.yaml]) or a bare '
             'scenario name')
    parser.add_argument(
        '-c', '--config', default=None,
        help='explicit scenario config path (overrides the positional form)')
    parser.add_argument(
        '--common', default='common.yaml',
        help='shared defaults file merged first (default: common.yaml)')
    parser.add_argument(
        '-s', '--set', dest='overrides', action='append', default=[],
        metavar='KEY=VALUE',
        help='override any config key, e.g. -s fleet.num_uavs=4 '
             '-s test.params.cycles=25 (repeatable)')
    parser.add_argument('--num-uavs', type=int, default=None,
                        help='shorthand for -s fleet.num_uavs=N')
    parser.add_argument('--namespaces', default=None,
                        help='comma-separated drone namespaces, e.g. cf1,cf3,cf7')
    parser.add_argument('--dry-run', action='store_true',
                        help='log commands instead of sending them')
    parser.add_argument('--tag', default=None,
                        help='suffix added to the report file names')
    parser.add_argument('--output-dir', default=None,
                        help='directory for the JSON/CSV artefacts')
    parser.add_argument('--no-report-files', action='store_true',
                        help='console report only, write nothing to disk')
    parser.add_argument('--no-color', action='store_true',
                        help='disable ANSI colours')
    parser.add_argument('--list', action='store_true',
                        help='list the available scenarios and exit')
    parser.add_argument('--print-config', action='store_true',
                        help='print the merged config and exit without flying')
    return parser


def apply_cli_overrides(cfg, args: argparse.Namespace) -> None:
    """Fold the convenience flags into the merged config."""
    if args.num_uavs is not None:
        set_dotted(cfg, 'fleet.num_uavs', int(args.num_uavs))
        set_dotted(cfg, 'fleet.namespaces', [])
    if args.namespaces:
        names = [n.strip() for n in args.namespaces.split(',') if n.strip()]
        set_dotted(cfg, 'fleet.namespaces', names)
    if args.dry_run:
        set_dotted(cfg, 'safety.dry_run', True)
    if args.tag is not None:
        set_dotted(cfg, 'report.tag', args.tag)
    if args.output_dir is not None:
        set_dotted(cfg, 'report.output_dir', args.output_dir)
    if args.no_report_files:
        set_dotted(cfg, 'report.formats', [])


def resolve_scenario_argument(scenario: Optional[str],
                              config_path: Optional[str]) -> Optional[str]:
    """Work out which file to load from the positional/`--config` arguments.

    Returns the config file to load, or ``None`` when the positional argument
    was a bare scenario name (in which case ``test.type`` is set directly).
    """
    if config_path:
        return config_path
    if not scenario:
        return None
    try:
        find_config(scenario)
        return scenario
    except FileNotFoundError:
        return None


def main(argv: Optional[List[str]] = None) -> int:
    raw = list(sys.argv[1:] if argv is None else argv)
    # Drop --ros-args ... so argparse only sees our own flags.
    try:
        raw = rclpy.utilities.remove_ros_args(args=['stress_test'] + raw)[1:]
    except Exception:
        pass

    parser = build_parser()
    args = parser.parse_args(raw)

    if args.no_color:
        colors.set_enabled(False)

    if args.list:
        print(bold('Available scenarios:'))
        for line in scenario_listing():
            print('  ' + line)
        return 0

    # Two-step so a bare scenario name still gets the common defaults.
    scenario_file = resolve_scenario_argument(args.scenario, args.config)
    try:
        cfg = load_config(scenario_file, args.common, args.overrides)
        if scenario_file is None and args.scenario:
            set_dotted(cfg, 'test.type', args.scenario)
        apply_cli_overrides(cfg, args)
        # Re-validate after the CLI layer.
        resolve_namespaces(cfg)
    except (FileNotFoundError, ValueError, KeyError) as exc:
        print(red(f"Configuration error: {exc}"))
        return 2

    if args.print_config:
        import yaml
        print(yaml.safe_dump(cfg, sort_keys=False, default_flow_style=False))
        return 0

    # Own the SIGINT handler so Ctrl-C can land the fleet instead of tearing
    # the context down underneath it.
    try:
        from rclpy.signals import SignalHandlerOptions
        rclpy.init(signal_handler_options=SignalHandlerOptions.NO)
    except (ImportError, TypeError):  # older rclpy
        rclpy.init()

    try:
        runner = StressTestRunner(cfg)
    except (KeyError, ValueError) as exc:
        print(red(f"Configuration error: {exc}"))
        rclpy.shutdown()
        return 2

    interrupts = {'count': 0}

    def _on_sigint(signum, frame):  # noqa: ARG001 - signal signature
        interrupts['count'] += 1
        if interrupts['count'] == 1:
            print(yellow('\nCtrl-C: aborting scenario and landing the fleet '
                         '(press again to exit immediately)'))
            runner.request_abort('sigint')
        else:
            print(red('\nSecond Ctrl-C: exiting without landing'))
            import os
            os._exit(130)

    previous = signal.signal(signal.SIGINT, _on_sigint)
    try:
        return runner.run()
    finally:
        signal.signal(signal.SIGINT, previous)
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == '__main__':
    sys.exit(main())
