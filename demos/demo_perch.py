#!/usr/bin/env python3
"""Perch demo: latch a top-mounted gripper onto a perch, hang, twist free.

Standalone rclpy node for testing the perch mechanism against a running
crazyswarm2 ``crazyflie_server``. It needs nothing from the rest of this
package -- only rclpy, crazyflie_interfaces and ``demo_perch.yaml``::

    python3 demos/demo_perch.py [-c demos/demo_perch.yaml]

Sequence (one drone):

1. arm + high-level takeoff
2. **approach**  cmd_hover to ``approach.below`` under the perch
3. **engage**    climb slowly past the perch height so the gripper snaps shut,
   press for ``engage.hold`` s, then disarm
4. **perched**   hang disarmed for ``perched.duration``
5. **release**   re-arm, spin up holding the hanging height, yaw clockwise
   until the mechanism unlocks, descend ``release.drop``, check that the
   drone really came off, turn back to the starting heading
6. **return**    cmd_hover back over the start position, high-level land

``mode`` runs the halves in isolation: ``perch_only`` is steps 1-3 and leaves
the drone hanging disarmed; ``unperch_only`` starts from a drone hung on the
perch by hand and runs steps 4-6, flying to (0, 0) since it never saw a
takeoff spot.

Everything between takeoff and land is ``cmd_hover``, a velocity setpoint with
height hold, so each phase closes the loop on ``/<ns>/pose``: XY is
P-controlled (vx/vy are body frame and rotated as such), the height setpoint
is ramped. The yaw rate is zero throughout except for the release twist and
the turn back after it, both P-controlled on the measured heading.

Ctrl-C lands in place -- which is also safe while the gripper still holds the
drone, it just ends up hanging. While it hangs disarmed, Ctrl-C leaves it
there. A second Ctrl-C exits immediately.
"""

from __future__ import annotations

import argparse
import copy
import math
import os
import signal
import sys
import threading
import time
import traceback
from dataclasses import dataclass
from pathlib import Path
from typing import Any, Dict, Optional, Sequence, Tuple

import rclpy
import yaml
from crazyflie_interfaces.msg import Hover
from crazyflie_interfaces.srv import Land, NotifySetpointsStop, Takeoff
from geometry_msgs.msg import PoseStamped
from rclpy.duration import Duration
from rclpy.executors import SingleThreadedExecutor
from rclpy.node import Node
from rclpy.qos import QoSProfile, QoSReliabilityPolicy

try:  # Arm only exists on newer crazyflie_interfaces
    from crazyflie_interfaces.srv import Arm
except ImportError:  # pragma: no cover
    Arm = None  # type: ignore[assignment,misc]

DEFAULT_CONFIG = Path(__file__).resolve().with_name('demo_perch.yaml')

# A closed-loop turn whose heading error grows by this much is turning the
# wrong way -- stop before it winds the gripper up in the wrong direction.
WRONG_WAY_DEG = 20.0

# The approach height under the perch has to leave room to fly.
MIN_APPROACH_Z = 0.2

MODES = ('full', 'perch_only', 'unperch_only')

#: Schema and fallback values; demo_perch.yaml documents every key.
DEFAULTS: Dict[str, Any] = {
    'mode': 'full',
    'namespace': 'cf1',
    'perch': {'position': None},
    'takeoff': {'height': 0.5, 'duration': 2.5, 'settle': 1.0},
    'approach': {'below': 0.25, 'settle': 2.0, 'timeout': 20.0},
    'engage': {'overshoot': 0.05, 'speed': 0.10, 'hold': 1.5},
    'perched': {'duration': 5.0},
    'release': {'spinup': 1.0, 'z_offset': 0.0, 'angle_deg': 90.0,
                'yaw_rate': 30.0, 'timeout': 8.0, 'drop': 0.30,
                'descent_speed': 0.15, 'settle': 1.0, 'min_drop_fraction': 0.5},
    'return': {'height': 0.5, 'settle': 1.0, 'timeout': 20.0},
    'land': {'duration': 3.0},
    'control': {'rate_hz': 20.0, 'kp_xy': 1.0, 'max_vxy': 0.3, 'max_vz': 0.3,
                'kp_yaw': 1.5, 'xy_tolerance': 0.03, 'z_tolerance': 0.05,
                'yaw_tolerance_deg': 3.0, 'yaw_rate_sign': 1},
    'safety': {'max_height': 1.8, 'pose_timeout': 0.5, 'arm_delay': 0.5,
               'notify_stop_ms': 300, 'service_timeout': 3.0, 'startup_timeout': 5.0},
}


# --------------------------------------------------------------------------
# config
# --------------------------------------------------------------------------

def merge_checked(base: Dict[str, Any], override: Dict[str, Any],
                  path: str = '') -> Dict[str, Any]:
    """Merge ``override`` onto ``base``, rejecting keys ``base`` does not have.

    A typo such as ``perch.positon`` must not silently fall back to a default
    in a script that flies at a physical structure.
    """
    merged = copy.deepcopy(base)
    for key, value in (override or {}).items():
        where = f"{path}{key}"
        if key not in base:
            raise ValueError(f"unknown config key '{where}'")
        if isinstance(base[key], dict):
            if not isinstance(value, dict):
                raise ValueError(f"'{where}' must be a mapping")
            merged[key] = merge_checked(base[key], value, where + '.')
        else:
            merged[key] = value
    return merged


def load_config(path: str) -> Dict[str, Any]:
    with open(path, 'r') as handle:
        data = yaml.safe_load(handle) or {}
    if not isinstance(data, dict):
        raise ValueError(f"{path}: expected a mapping at the top level")
    cfg = merge_checked(DEFAULTS, data)
    validate(cfg)
    return cfg


def validate(cfg: Dict[str, Any]) -> None:
    """Catch before takeoff what would otherwise only fail mid-flight."""
    if cfg['mode'] not in MODES:
        raise ValueError(f"mode must be one of {', '.join(MODES)}")
    max_height = float(cfg['safety']['max_height'])

    # unperch_only starts from wherever the drone hangs; it never uses this.
    if cfg['mode'] != 'unperch_only':
        position = cfg['perch']['position']
        if not (isinstance(position, (list, tuple)) and len(position) == 3):
            raise ValueError('perch.position must be set to [x, y, z]')
        perch_z = float(position[2])
        top = perch_z + float(cfg['engage']['overshoot'])
        if top > max_height:
            raise ValueError(
                f"perch z + engage.overshoot = {top:.2f} m is above "
                f"safety.max_height ({max_height:.2f} m)")
        for name, z in (('perch z - approach.below',
                         perch_z - float(cfg['approach']['below'])),
                        ('perch z - release.drop',
                         perch_z - float(cfg['release']['drop']))):
            if z < MIN_APPROACH_Z:
                raise ValueError(f"{name} = {z:.2f} m, must be >= {MIN_APPROACH_Z} m")

    for key in ('takeoff', 'return'):
        height = float(cfg[key]['height'])
        if not 0.0 < height <= max_height:
            raise ValueError(f"{key}.height must be in (0, {max_height}]")

    for dotted in ('engage.speed', 'release.descent_speed', 'control.rate_hz',
                   'control.max_vz', 'control.max_vxy', 'release.yaw_rate'):
        section, key = dotted.split('.')
        if float(cfg[section][key]) <= 0.0:
            raise ValueError(f"{dotted} must be > 0")
    if cfg['control']['yaw_rate_sign'] not in (1, -1):
        raise ValueError('control.yaw_rate_sign must be 1 or -1')


# --------------------------------------------------------------------------
# helpers
# --------------------------------------------------------------------------

class Abort(Exception):
    """Stop the sequence and bring the drone to a safe state."""


@dataclass
class Pose:
    stamp: float  # time.monotonic() at reception
    x: float
    y: float
    z: float
    yaw: float    # rad, CCW from world +x


def clamp(value: float, low: float, high: float) -> float:
    return max(low, min(high, value))


def wrap_deg(angle: float) -> float:
    """Wrap to [-180, 180)."""
    return (angle + 180.0) % 360.0 - 180.0


def yaw_of(q) -> float:
    return math.atan2(2.0 * (q.w * q.z + q.x * q.y),
                      1.0 - 2.0 * (q.y * q.y + q.z * q.z))


def world_to_body(vx: float, vy: float, yaw: float) -> Tuple[float, float]:
    c, s = math.cos(yaw), math.sin(yaw)
    return c * vx + s * vy, -s * vx + c * vy


# --------------------------------------------------------------------------
# the demo
# --------------------------------------------------------------------------

class PerchDemo(Node):

    def __init__(self, cfg: Dict[str, Any]):
        super().__init__('demo_perch')
        self.cfg = cfg
        self.ns = str(cfg['namespace']).strip('/')
        self.log = self.get_logger()
        self.abort_event = threading.Event()

        # ground | flying | hanging (disarmed on the perch) | landed.
        # Decides what the abort path does.
        self.state = 'ground'
        self.yaw_sign = int(cfg['control']['yaw_rate_sign'])
        self.z_cmd = 0.0
        self.start: Optional[Pose] = None

        self._pose: Optional[Pose] = None
        self._pose_lock = threading.Lock()
        # BEST_EFFORT subscribes to both reliable and best-effort publishers.
        qos = QoSProfile(depth=10, reliability=QoSReliabilityPolicy.BEST_EFFORT)
        self.create_subscription(PoseStamped, f'/{self.ns}/pose', self._on_pose, qos)

        self.hover_pub = self.create_publisher(Hover, f'/{self.ns}/cmd_hover', 10)
        self.clients_by_name = {
            'takeoff': self.create_client(Takeoff, f'/{self.ns}/takeoff'),
            'land': self.create_client(Land, f'/{self.ns}/land'),
            'notify_setpoints_stop': self.create_client(
                NotifySetpointsStop, f'/{self.ns}/notify_setpoints_stop'),
        }
        if Arm is not None:
            self.clients_by_name['arm'] = self.create_client(Arm, f'/{self.ns}/arm')

    # -- sequence ----------------------------------------------------------

    def run(self) -> bool:
        mode = self.cfg['mode']
        try:
            self._startup()
            if mode in ('full', 'perch_only'):
                self._takeoff()
                self._approach()
                self._engage()
            if mode in ('full', 'unperch_only'):
                self._perch()
                self._release()
                self._return_and_land()
        except Abort as exc:
            self._abort(str(exc) or 'aborted')
            return False
        except Exception:  # noqa: BLE001 - whatever broke, get the drone down
            self.log.error(traceback.format_exc())
            self._abort('unexpected error')
            return False
        if mode == 'perch_only':
            self.log.info('Perch demo complete (perch_only): left hanging, disarmed')
        else:
            self.log.info(f"Perch demo complete ({mode})")
        return True

    def _startup(self) -> None:
        timeout = float(self.cfg['safety']['startup_timeout'])
        deadline = time.monotonic() + timeout
        if Arm is None:
            raise Abort('this crazyflie_interfaces has no Arm service, and the '
                        'demo disarms on the perch')
        required = ('takeoff', 'land', 'notify_setpoints_stop', 'arm')
        while time.monotonic() < deadline:
            missing = [n for n in required
                       if not self.clients_by_name[n].service_is_ready()]
            if not missing and self._latest_pose() is not None:
                break
            self._sleep(0.1)
        else:
            missing = [n for n in required
                       if not self.clients_by_name[n].service_is_ready()]
            if missing:
                raise Abort(f"services not available after {timeout:.0f} s: "
                            f"{', '.join(f'/{self.ns}/{n}' for n in missing)}")
            raise Abort(f"no pose on /{self.ns}/pose after {timeout:.0f} s")

        self.start = self._fresh_pose()
        where = (f"{self.ns} at ({self.start.x:.2f}, {self.start.y:.2f}, "
                 f"{self.start.z:.2f})")
        if self.cfg['mode'] == 'unperch_only':
            # Hung on the perch by hand: disarmed, and an abort before the
            # re-arm must leave it there.
            self.state = 'hanging'
            self.log.info(f"{where}, hanging on the perch (unperch_only)")
        else:
            px, py, pz = self._perch_position()
            self.log.info(f"{where}, perch at ({px:.2f}, {py:.2f}, {pz:.2f})")

    def _takeoff(self) -> None:
        height = float(self.cfg['takeoff']['height'])
        duration = float(self.cfg['takeoff']['duration'])
        self.log.info(f"[takeoff] to {height:.2f} m")
        self._set_armed(True)
        request = Takeoff.Request()
        request.group_mask = 0
        request.height = height
        request.duration = Duration(seconds=duration).to_msg()
        # Flying from here on as far as the abort path is concerned: a takeoff
        # whose response got lost may still be climbing.
        self.state = 'flying'
        self._call('takeoff', request)
        self._sleep(duration + float(self.cfg['takeoff']['settle']))
        pose = self._fresh_pose()
        if pose.z < 0.5 * height:
            raise Abort(f"takeoff did not happen (z = {pose.z:.2f} m)")
        self.z_cmd = pose.z

    def _approach(self) -> None:
        px, py, pz = self._perch_position()
        z = pz - float(self.cfg['approach']['below'])
        self.log.info(f"[approach] to ({px:.2f}, {py:.2f}, {z:.2f})")
        if not self._fly('approach', (px, py), z,
                         settle=float(self.cfg['approach']['settle']),
                         timeout=float(self.cfg['approach']['timeout'])):
            raise Abort('could not settle under the perch')

    def _engage(self) -> None:
        cfg = self.cfg['engage']
        px, py, pz = self._perch_position()
        speed = float(cfg['speed'])
        top = pz + float(cfg['overshoot'])

        self.log.info(f"[engage] climbing to {top:.2f} m at {speed:.2f} m/s")
        self._fly('engage', (px, py), top, max_vz=speed,
                  duration=(top - self.z_cmd) / speed)
        # Zero XY velocity while pressed into the gripper: position control
        # would only wind up against the perch.
        self._fly('engage', None, top, duration=float(cfg['hold']))

        self.log.info(f"[engage] disarming at z = {self._fresh_pose().z:.2f} m")
        self._set_armed(False)
        self.state = 'hanging'

    def _perch(self) -> None:
        duration = float(self.cfg['perched']['duration'])
        self.log.info(f"[perched] hanging for {duration:.0f} s")
        self._sleep(duration)

    def _release(self) -> None:
        cfg = self.cfg['release']
        # No pose -> Abort while still 'hanging': it stays on the perch.
        self._fresh_pose()
        self._set_armed(True)
        hold = self._fresh_pose().z + float(cfg['z_offset'])
        self.z_cmd = hold

        self.log.info(f"[release] spinning up, holding z = {hold:.2f} m")
        self.state = 'flying'
        self._fly('spin-up', None, hold, duration=float(cfg['spinup']))

        angle = float(cfg['angle_deg'])
        yaw0 = self._fresh_pose().yaw
        self.log.info(f"[release] turning {angle:.0f} deg clockwise")
        if not self._fly('twist', None, hold, yaw=yaw0 - math.radians(angle), settle=0.3,
                         timeout=float(cfg['timeout'])):
            self.log.warning('[release] twist did not finish, trying to drop anyway')
        pose = self._fresh_pose()
        turned = -wrap_deg(math.degrees(pose.yaw - yaw0))
        self.log.info(f"[release] turned {turned:.1f} deg clockwise")

        drop = float(cfg['drop'])
        speed = float(cfg['descent_speed'])
        self.log.info(f"[release] dropping {drop:.2f} m")
        self._fly('drop', (pose.x, pose.y), hold - drop, max_vz=speed,
                  duration=drop / speed + float(cfg['settle']))
        dropped = hold - self._fresh_pose().z
        if dropped < float(cfg['min_drop_fraction']) * drop:
            raise Abort(f"still attached: only dropped {dropped * 100:.0f} cm of "
                        f"{drop * 100:.0f} cm")
        self.log.info(f"[release] free, dropped {dropped * 100:.0f} cm")

        assert self.start is not None
        pose = self._fresh_pose()
        self.log.info(f"[release] turning back to the starting heading "
                      f"({math.degrees(self.start.yaw):.0f} deg)")
        if not self._fly('turn back', (pose.x, pose.y), self.z_cmd,
                         yaw=self.start.yaw, settle=0.3,
                         timeout=float(cfg['timeout'])):
            self.log.warning('[release] did not finish turning back, returning anyway')

    def _return_and_land(self) -> None:
        assert self.start is not None
        cfg = self.cfg['return']
        # unperch_only never saw a takeoff spot, so it goes to the origin.
        if self.cfg['mode'] == 'unperch_only':
            home = (0.0, 0.0)
        else:
            home = (self.start.x, self.start.y)
        self.log.info(f"[return] to ({home[0]:.2f}, {home[1]:.2f}) "
                      f"at {float(cfg['height']):.2f} m")
        if not self._fly('return', home, float(cfg['height']),
                         settle=float(cfg['settle']),
                         timeout=float(cfg['timeout'])):
            self.log.warning('[return] did not settle over the start, landing here')
        self.log.info('[land]')
        self._land()

    # -- abort -------------------------------------------------------------

    def _abort(self, reason: str) -> None:
        self.log.error(f"ABORT: {reason}")
        if self.state == 'hanging':
            self.log.warning('Disarmed and hanging on the perch -- leaving it there')
        elif self.state == 'flying':
            self.log.warning('Landing in place (if the gripper still holds it, '
                             'this just leaves it hanging)')
            try:
                self._land()
            except Abort as exc:
                self.log.error(f"Land failed: {exc}")

    # -- flight primitives -------------------------------------------------

    def _fly(self, what: str, xy: Optional[Tuple[float, float]], z: float, *,
             yaw: Optional[float] = None, max_vz: Optional[float] = None,
             duration: Optional[float] = None, settle: float = 0.0,
             timeout: Optional[float] = None) -> bool:
        """Stream cmd_hover towards a target.

        ``xy=None`` streams zero horizontal velocity instead of position
        control. ``yaw=None`` streams zero yaw rate, i.e. keeps the current
        heading; a ``yaw`` target (rad) turns at up to ``release.yaw_rate``.
        The height setpoint ramps from ``self.z_cmd`` at ``max_vz``.

        With ``duration``: stream for exactly that long and return True.
        Otherwise: True once inside tolerance for ``settle`` s, False on
        ``timeout``.
        """
        ctl = self.cfg['control']
        dt = 1.0 / float(ctl['rate_hz'])
        z = clamp(z, 0.0, float(self.cfg['safety']['max_height']))
        z_step = float(ctl['max_vz'] if max_vz is None else max_vz) * dt
        yaw_cap = float(self.cfg['release']['yaw_rate'])
        kp_xy, max_vxy = float(ctl['kp_xy']), float(ctl['max_vxy'])

        start = time.monotonic()
        next_tick = start
        inside_since: Optional[float] = None
        initial_yaw_error: Optional[float] = None

        while True:
            now = time.monotonic()
            if duration is not None and now - start >= duration:
                return True
            if timeout is not None and now - start >= timeout:
                self.log.warning(f"[{what}] not settled after {timeout:.0f} s")
                return False
            pose = self._fresh_pose()

            if abs(z - self.z_cmd) <= z_step:
                self.z_cmd = z
            else:
                self.z_cmd += math.copysign(z_step, z - self.z_cmd)

            vx = vy = xy_error = 0.0
            if xy is not None:
                ex, ey = xy[0] - pose.x, xy[1] - pose.y
                xy_error = math.hypot(ex, ey)
                wx, wy = kp_xy * ex, kp_xy * ey
                speed = math.hypot(wx, wy)
                if speed > max_vxy:
                    wx, wy = wx * max_vxy / speed, wy * max_vxy / speed
                vx, vy = world_to_body(wx, wy, pose.yaw)

            yaw_rate = yaw_error = 0.0
            if yaw is not None:
                yaw_error = wrap_deg(math.degrees(yaw - pose.yaw))
                if initial_yaw_error is None:
                    initial_yaw_error = abs(yaw_error)
                elif abs(yaw_error) > initial_yaw_error + WRONG_WAY_DEG:
                    raise Abort(f"[{what}] heading is moving away from the target "
                                f"({yaw_error:+.0f} deg off) -- "
                                f"control.yaw_rate_sign is wrong for this firmware")
                yaw_rate = clamp(float(ctl['kp_yaw']) * yaw_error, -yaw_cap, yaw_cap)

            self._publish_hover(vx, vy, yaw_rate, self.z_cmd)

            if duration is None:
                inside = (xy_error <= float(ctl['xy_tolerance'])
                          and self.z_cmd == z
                          and abs(z - pose.z) <= float(ctl['z_tolerance'])
                          and abs(yaw_error) <= float(ctl['yaw_tolerance_deg']))
                if not inside:
                    inside_since = None
                elif inside_since is None:
                    inside_since = now
                elif now - inside_since >= settle:
                    return True

            next_tick = max(next_tick + dt, time.monotonic())
            self._sleep(next_tick - time.monotonic())

    def _publish_hover(self, vx: float, vy: float, yaw_rate_deg: float,
                       z: float) -> None:
        msg = Hover()
        msg.header.stamp = self.get_clock().now().to_msg()
        msg.vx = float(vx)
        msg.vy = float(vy)
        # Hover.yaw_rate is rad/s: crazyflie_server converts it to deg/s for
        # the firmware.
        msg.yaw_rate = math.radians(self.yaw_sign * yaw_rate_deg)
        msg.z_distance = float(clamp(z, 0.0, float(self.cfg['safety']['max_height'])))
        self.hover_pub.publish(msg)

    def _land(self) -> None:
        """Hand back to the high-level commander and land where it is."""
        safety = self.cfg['safety']
        duration = float(self.cfg['land']['duration'])

        request = NotifySetpointsStop.Request()
        request.group_mask = 0
        request.remain_valid_millisecs = int(safety['notify_stop_ms'])
        try:
            self._call('notify_setpoints_stop', request)
        except Abort as exc:
            # Still land: the high-level commander takes over on its own once
            # the last cmd_hover setpoint times out, just later.
            self.log.warning(f"{exc}, landing anyway")

        request = Land.Request()
        request.group_mask = 0
        request.height = 0.0
        request.duration = Duration(seconds=duration).to_msg()
        self._call('land', request)
        # Deliberately not abort-aware: a Ctrl-C now should not land twice.
        time.sleep(duration + int(safety['notify_stop_ms']) / 1000.0 + 0.5)
        self.state = 'landed'

    def _set_armed(self, armed: bool) -> None:
        request = Arm.Request()
        request.arm = armed
        self._call('arm', request)
        if armed:
            # Let the supervisor latch the armed state before setpoints arrive.
            self._sleep(float(self.cfg['safety']['arm_delay']))

    # -- ROS plumbing ------------------------------------------------------

    def _call(self, name: str, request) -> Any:
        client = self.clients_by_name[name]
        if not client.service_is_ready():
            raise Abort(f"/{self.ns}/{name} is not available")
        future = client.call_async(request)
        deadline = time.monotonic() + float(self.cfg['safety']['service_timeout'])
        while not future.done():
            if time.monotonic() > deadline:
                future.cancel()
                raise Abort(f"/{self.ns}/{name} did not answer")
            time.sleep(0.01)
        return future.result()

    def _sleep(self, seconds: float) -> None:
        """Sleep, raising Abort as soon as Ctrl-C is pressed."""
        if self.abort_event.wait(max(0.0, seconds)):
            raise Abort('Ctrl-C')

    def _on_pose(self, msg: PoseStamped) -> None:
        p = msg.pose.position
        pose = Pose(time.monotonic(), p.x, p.y, p.z, yaw_of(msg.pose.orientation))
        with self._pose_lock:
            self._pose = pose

    def _latest_pose(self) -> Optional[Pose]:
        with self._pose_lock:
            return self._pose

    def _fresh_pose(self) -> Pose:
        pose = self._latest_pose()
        if pose is None:
            raise Abort(f"no pose from /{self.ns}/pose")
        age = time.monotonic() - pose.stamp
        if age > float(self.cfg['safety']['pose_timeout']):
            raise Abort(f"pose is {age:.1f} s old")
        return pose

    def _perch_position(self) -> Tuple[float, float, float]:
        x, y, z = (float(v) for v in self.cfg['perch']['position'])
        return x, y, z


def main(argv: Optional[Sequence[str]] = None) -> int:
    argv = list(sys.argv if argv is None else argv)
    parser = argparse.ArgumentParser(
        prog='demo_perch', description='Perch-mechanism demo for one Crazyflie')
    parser.add_argument('-c', '--config', default=str(DEFAULT_CONFIG),
                        help=f'YAML config (default: {DEFAULT_CONFIG})')
    args = parser.parse_args(rclpy.utilities.remove_ros_args(args=argv)[1:])

    try:
        cfg = load_config(args.config)
    except (OSError, ValueError, yaml.YAMLError) as exc:
        print(f"Config error: {exc}", file=sys.stderr)
        return 2

    # Own SIGINT so Ctrl-C can land the drone instead of tearing ROS down.
    try:
        from rclpy.signals import SignalHandlerOptions
        rclpy.init(args=argv, signal_handler_options=SignalHandlerOptions.NO)
    except (ImportError, TypeError):  # older rclpy
        rclpy.init(args=argv)

    node = PerchDemo(cfg)
    executor = SingleThreadedExecutor()
    executor.add_node(node)
    spin_thread = threading.Thread(target=executor.spin, daemon=True)
    spin_thread.start()

    presses = [0]

    def on_sigint(signum, frame):  # noqa: ARG001 - signal signature
        presses[0] += 1
        if presses[0] == 1:
            node.log.warning('Ctrl-C: aborting (press again to exit immediately)')
            node.abort_event.set()
        else:
            os._exit(130)

    signal.signal(signal.SIGINT, on_sigint)
    try:
        return 0 if node.run() else 1
    finally:
        executor.shutdown()
        # Destroying the node while spin() is still inside rcl aborts the
        # process ("terminate called without an active exception").
        spin_thread.join(timeout=2.0)
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == '__main__':
    sys.exit(main())
