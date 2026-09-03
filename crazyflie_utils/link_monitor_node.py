#!/usr/bin/env python3
"""Passive link monitor -- watch fleet telemetry without commanding anything.

Prints a per-drone table of downlink health (pose rate, longest dropout,
stalls, RSSI, battery) every ``--interval`` seconds and a whole-run summary on
Ctrl-C. Safe to run alongside a flight: it only subscribes.

Useful on its own to answer "is the radio healthy right now?", and as a second
terminal while another stack (or another stress test) drives the drones::

    ros2 run crazyflie_utils link_monitor --num-uavs 4
    ros2 run crazyflie_utils link_monitor --namespaces cf1,cf5 --interval 2
"""

from __future__ import annotations

import argparse
import signal
import sys
import threading
import time
from typing import List, Optional

import rclpy
from rclpy.executors import SingleThreadedExecutor
from rclpy.node import Node

from crazyflie_utils import colors
from crazyflie_utils.colors import bold, cyan, green, red, yellow
from crazyflie_utils.config import cfg_get, load_config, resolve_namespaces, set_dotted
from crazyflie_utils.link_monitor import LinkMonitor
from crazyflie_utils.report import fmt, render_table


def build_parser() -> argparse.ArgumentParser:
    parser = argparse.ArgumentParser(
        prog='link_monitor',
        description='Passive Crazyflie downlink monitor (never commands a drone)')
    parser.add_argument('--config', default='common.yaml',
                        help='config file supplying fleet/telemetry settings')
    parser.add_argument('--num-uavs', type=int, default=None,
                        help='shorthand for fleet.num_uavs')
    parser.add_argument('--namespaces', default=None,
                        help='comma-separated drone namespaces')
    parser.add_argument('--interval', type=float, default=5.0,
                        help='seconds between table updates (default: 5)')
    parser.add_argument('--duration', type=float, default=0.0,
                        help='stop after this many seconds (0 = until Ctrl-C)')
    parser.add_argument('--no-color', action='store_true')
    return parser


def _table(monitor: LinkMonitor, namespaces: List[str], label: str,
           t_start: float, expected: float) -> str:
    windows = {w.namespace: w for w in monitor.close_window(label, t_start)}
    rows = []
    for ns in namespaces:
        window = windows.get(ns)
        if window is None:
            rows.append([ns] + ['-'] * 9)
            continue
        ratio = window.rate_ratio
        health = green('ok') if ratio >= 0.9 else (
            yellow('degraded') if ratio >= 0.5 else red('bad'))
        position = monitor.latest_position(ns)
        rows.append([
            ns,
            str(window.pose_count),
            fmt(window.pose_rate, '.1f'),
            fmt(ratio * 100, '.0f') + '%',
            fmt(window.max_gap * 1000, '.0f'),
            str(window.stalls),
            fmt(window.rssi_mean, '.0f'),
            fmt(window.battery_end, '.2f'),
            f"z={position[2]:.2f}" if position else '-',
            health,
        ])
    header = ['drone', 'poses', 'Hz', f"%of {expected:g}Hz", 'max gap ms',
              'stalls', 'rssi', 'batt V', 'pose', 'state']
    return '\n'.join(render_table(header, rows))


def main(argv: Optional[List[str]] = None) -> int:
    raw = list(sys.argv[1:] if argv is None else argv)
    try:
        raw = rclpy.utilities.remove_ros_args(args=['link_monitor'] + raw)[1:]
    except Exception:
        pass
    args = build_parser().parse_args(raw)

    if args.no_color:
        colors.set_enabled(False)

    try:
        cfg = load_config(None, args.config, [])
        if args.num_uavs is not None:
            set_dotted(cfg, 'fleet.num_uavs', int(args.num_uavs))
            set_dotted(cfg, 'fleet.namespaces', [])
        if args.namespaces:
            set_dotted(cfg, 'fleet.namespaces',
                       [n.strip() for n in args.namespaces.split(',') if n.strip()])
        namespaces = resolve_namespaces(cfg)
    except (FileNotFoundError, ValueError) as exc:
        print(red(f"Configuration error: {exc}"))
        return 2

    expected = float(cfg_get(cfg, 'telemetry.expected_pose_rate', 10.0))

    try:
        from rclpy.signals import SignalHandlerOptions
        rclpy.init(signal_handler_options=SignalHandlerOptions.NO)
    except (ImportError, TypeError):
        rclpy.init()

    node = Node('crazyflie_link_monitor')
    monitor = LinkMonitor(node, namespaces, cfg.get('telemetry', {}))
    executor = SingleThreadedExecutor()
    executor.add_node(node)
    spin_thread = threading.Thread(target=executor.spin, daemon=True)
    spin_thread.start()

    stop = threading.Event()
    signal.signal(signal.SIGINT, lambda *_: stop.set())

    print(bold(f"Monitoring {len(namespaces)} drone(s): {', '.join(namespaces)}"))
    print(cyan(f"Expecting {expected:g} Hz pose telemetry; Ctrl-C to stop"))

    run_start = monitor.begin_window()
    try:
        while not stop.is_set():
            window_start = monitor.begin_window()
            if stop.wait(max(0.5, float(args.interval))):
                break
            print('')
            print(bold(time.strftime('%H:%M:%S')))
            print(_table(monitor, namespaces, 'interval', window_start, expected))
            if args.duration and (time.perf_counter() - run_start) >= args.duration:
                break
    finally:
        print('')
        print(bold('SUMMARY (whole run)'))
        print(_table(monitor, namespaces, 'run', run_start, expected))
        monitor.destroy()
        executor.shutdown()
        spin_thread.join(timeout=2.0)
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()
    return 0


if __name__ == '__main__':
    sys.exit(main())
