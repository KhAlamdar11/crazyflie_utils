"""Thin, instrumented client for a crazyswarm2 ``crazyflie_server``.

:class:`CrazyflieFleet` is the only place in this package that talks to the
server. It owns one set of service clients per drone plus the broadcast
(``/all/...``) clients, and every call it makes is timed and written into a
:class:`~crazyflie_utils.metrics.MetricsCollector`.

Two dispatch styles are offered, and the difference between them is exactly
what the sync/async scenario measures:

``takeoff(ns, ...)``
    Fire one call and block until the response arrives -- sequential, one
    radio transaction at a time.
``takeoff_async(ns, ...)`` + :meth:`gather`
    Fire calls at every drone back to back without waiting, then collect the
    responses -- the uplink sees N transactions in flight at once.

Broadcast (``broadcast_takeoff`` and friends) is a third style: a single
service call that the server turns into one broadcast radio packet for the
whole fleet.

Every command respects ``safety.max_height`` and ``safety.dry_run``.
"""

from __future__ import annotations

import math
import time
from dataclasses import dataclass
from typing import Any, Dict, Iterable, List, Optional, Sequence, Tuple

import rclpy
from rclpy.callback_groups import ReentrantCallbackGroup
from rclpy.node import Node
from std_srvs.srv import Empty

from crazyflie_interfaces.srv import GoTo, Land, NotifySetpointsStop, Takeoff
from crazyflie_interfaces.msg import FullState, Hover

from crazyflie_utils.config import cfg_get, clamp_height
from crazyflie_utils.metrics import CallRecord, MetricsCollector

try:  # Arm exists only on newer crazyflie_interfaces
    from crazyflie_interfaces.srv import Arm
    _ARM_AVAILABLE = True
except ImportError:  # pragma: no cover
    Arm = None  # type: ignore[assignment]
    _ARM_AVAILABLE = False

try:  # Position (cmd_position) is not offered by every server build
    from crazyflie_interfaces.msg import Position
    _POSITION_AVAILABLE = True
except ImportError:  # pragma: no cover
    Position = None  # type: ignore[assignment]
    _POSITION_AVAILABLE = False


@dataclass
class PendingCall:
    """An in-flight service call awaiting :meth:`CrazyflieFleet.gather`."""

    action: str
    target: str
    phase: str
    t_start: float
    wall_time: float
    future: Any
    client: Any
    extra: Dict[str, Any]
    # Pre-resolved records (dry-run, or a client that was not ready) skip the
    # wait entirely.
    resolved: Optional[CallRecord] = None
    # One-element list that the future's done-callback stamps with the
    # completion time. Without it, latency would be measured when the caller
    # got around to looking at the future, which for parallel dispatch means
    # every call inherits the wait of the ones ahead of it.
    done_stamp: Optional[List[float]] = None


def wait_for_future(future, timeout: float, poll: float = 0.001) -> bool:
    """Block until ``future`` completes or ``timeout`` elapses.

    The node is spun by the runner's executor thread, so this only has to
    poll. Returns ``True`` when the future completed in time.
    """
    deadline = time.perf_counter() + timeout
    while time.perf_counter() < deadline:
        if future.done():
            return True
        time.sleep(poll)
    return future.done()


class CrazyflieFleet(Node):
    """Instrumented service/topic client for a fleet of Crazyflies."""

    def __init__(self, cfg: Dict[str, Any], namespaces: Sequence[str],
                 metrics: MetricsCollector, node_name: str = 'crazyflie_stress_test'):
        super().__init__(node_name)
        self.cfg = cfg
        self.namespaces: List[str] = list(namespaces)
        self.metrics = metrics
        self.broadcast_ns = str(cfg_get(cfg, 'fleet.broadcast_namespace', 'all'))
        self.dry_run = bool(cfg_get(cfg, 'safety.dry_run', False))
        self.service_timeout = float(cfg_get(cfg, 'timing.service_timeout', 3.0))

        # Reentrant so the executor can service several response callbacks
        # while the scenario thread is blocked waiting on one of them.
        self._cb_group = ReentrantCallbackGroup()

        self._srv_clients: Dict[Tuple[str, str], Any] = {}
        self._topic_pubs: Dict[Tuple[str, str], Any] = {}

        targets = self.namespaces + [self.broadcast_ns]
        for ns in targets:
            self._srv_clients[(ns, 'takeoff')] = self._client(Takeoff, ns, 'takeoff')
            self._srv_clients[(ns, 'land')] = self._client(Land, ns, 'land')
            self._srv_clients[(ns, 'go_to')] = self._client(GoTo, ns, 'go_to')
            self._srv_clients[(ns, 'emergency')] = self._client(Empty, ns, 'emergency')
            if _ARM_AVAILABLE:
                self._srv_clients[(ns, 'arm')] = self._client(Arm, ns, 'arm')

        # notify_setpoints_stop is per-drone only on a stock server.
        for ns in self.namespaces:
            self._srv_clients[(ns, 'notify_setpoints_stop')] = self._client(
                NotifySetpointsStop, ns, 'notify_setpoints_stop')

        self.get_logger().info(
            f"Fleet client ready for {len(self.namespaces)} drone(s): "
            f"{', '.join(self.namespaces)}"
            + ('  [DRY RUN]' if self.dry_run else '')
        )

    # -- construction helpers ---------------------------------------------

    def _client(self, srv_type, ns: str, name: str):
        return self.create_client(
            srv_type, f"/{ns}/{name}", callback_group=self._cb_group)

    def client_for(self, ns: str, action: str):
        return self._srv_clients.get((ns, action))

    def _publisher(self, ns: str, topic: str, msg_type):
        key = (ns, topic)
        if key not in self._topic_pubs:
            self._topic_pubs[key] = self.create_publisher(
                msg_type, f"/{ns}/{topic}", 10)
        return self._topic_pubs[key]

    # -- readiness ---------------------------------------------------------

    def wait_for_services(self, actions: Sequence[str],
                          timeout: float) -> Dict[str, List[str]]:
        """Wait for the given actions on every drone.

        Returns ``{namespace: [missing actions]}`` -- empty when everything
        showed up. ``<ns>/emergency`` is the last service a crazyflie_server
        advertises, so waiting on it is a good proxy for "the server finished
        connecting to this drone".
        """
        deadline = time.perf_counter() + timeout
        missing: Dict[str, List[str]] = {}

        while True:
            missing = {}
            for ns in self.namespaces + [self.broadcast_ns]:
                absent = [
                    action for action in actions
                    if (ns, action) in self._srv_clients
                    and not self._srv_clients[(ns, action)].service_is_ready()
                ]
                # An action we have no client for at all (e.g. arm on old
                # interfaces, or notify_setpoints_stop on /all) is reported so
                # the caller can decide whether it matters.
                absent += [action for action in actions
                           if (ns, action) not in self._srv_clients
                           and ns != self.broadcast_ns]
                if absent:
                    missing[ns] = absent
            if not missing or time.perf_counter() >= deadline:
                return missing
            time.sleep(0.1)

    # -- generic call plumbing --------------------------------------------

    def _dispatch(self, ns: str, action: str, request, phase: str,
                  extra: Optional[Dict[str, Any]] = None) -> PendingCall:
        """Send one service call without waiting for the response."""
        extra = dict(extra or {})
        t_start = time.perf_counter()
        wall = time.time()
        client = self._srv_clients.get((ns, action))

        if self.dry_run:
            extra['dry_run'] = True
            record = CallRecord(action=action, target=ns, phase=phase,
                                t_start=t_start, latency=0.0, success=True,
                                wall_time=wall, extra=extra)
            return PendingCall(action, ns, phase, t_start, wall, None, None,
                               extra, resolved=record)

        if client is None:
            record = CallRecord(action=action, target=ns, phase=phase,
                                t_start=t_start, latency=0.0, success=False,
                                error='no_client', wall_time=wall, extra=extra)
            return PendingCall(action, ns, phase, t_start, wall, None, None,
                               extra, resolved=record)

        if not client.service_is_ready():
            record = CallRecord(action=action, target=ns, phase=phase,
                                t_start=t_start, latency=0.0, success=False,
                                error='unavailable', wall_time=wall, extra=extra)
            return PendingCall(action, ns, phase, t_start, wall, None, None,
                               extra, resolved=record)

        future = client.call_async(request)
        done_stamp: Optional[List[float]] = []
        try:
            future.add_done_callback(
                lambda _future, _stamp=done_stamp: _stamp.append(time.perf_counter()))
        except (AttributeError, TypeError):  # future without callbacks
            done_stamp = None
        return PendingCall(action, ns, phase, t_start, wall, future, client,
                           extra, done_stamp=done_stamp)

    def _resolve(self, pending: PendingCall, timeout: Optional[float] = None,
                 deadline: Optional[float] = None) -> CallRecord:
        """Wait for one dispatched call and turn it into a :class:`CallRecord`."""
        if pending.resolved is not None:
            self.metrics.add_call(pending.resolved)
            return pending.resolved

        timeout = self.service_timeout if timeout is None else timeout
        if deadline is None:
            deadline = pending.t_start + timeout
        remaining = max(0.0, deadline - time.perf_counter())

        done = wait_for_future(pending.future, remaining)
        # Prefer the done-callback's stamp: it is when the response actually
        # arrived, not when this thread noticed.
        if done and pending.done_stamp:
            latency = pending.done_stamp[0] - pending.t_start
        else:
            latency = time.perf_counter() - pending.t_start

        if not done:
            # Drop the request so a late response cannot pile up in the client.
            try:
                pending.future.cancel()
                remove = getattr(pending.client, 'remove_pending_request', None)
                if remove is not None:
                    remove(pending.future)
            except Exception:
                pass
            record = CallRecord(
                action=pending.action, target=pending.target, phase=pending.phase,
                t_start=pending.t_start, latency=latency, success=False,
                error='timeout', wall_time=pending.wall_time, extra=pending.extra)
        else:
            error = None
            success = True
            try:
                result = pending.future.result()
            except Exception as exc:  # service call raised on the client side
                result, success, error = None, False, f"exception:{type(exc).__name__}"
            if success and result is not None and hasattr(result, 'success'):
                # Most crazyflie_interfaces responses are empty; honour a
                # success flag when the server build provides one.
                success = bool(result.success)
                if not success:
                    error = 'rejected'
            record = CallRecord(
                action=pending.action, target=pending.target, phase=pending.phase,
                t_start=pending.t_start, latency=latency, success=success,
                error=error, wall_time=pending.wall_time, extra=pending.extra)

        self.metrics.add_call(record)
        return record

    def wait(self, pending: PendingCall,
             timeout: Optional[float] = None) -> CallRecord:
        """Wait for a single dispatched call and record the result."""
        return self._resolve(pending, timeout=timeout)

    def gather(self, pending: Iterable[PendingCall],
               timeout: Optional[float] = None) -> List[CallRecord]:
        """Wait for a batch of dispatched calls.

        The deadline is common to the batch (measured from each call's own
        dispatch time), so a slow first response does not inflate the apparent
        latency of the ones behind it.
        """
        timeout = self.service_timeout if timeout is None else timeout
        pending = list(pending)
        records: List[CallRecord] = []
        for call in pending:
            records.append(self._resolve(call, deadline=call.t_start + timeout))
        return records

    def _call(self, ns: str, action: str, request, phase: str,
              extra: Optional[Dict[str, Any]] = None,
              timeout: Optional[float] = None) -> CallRecord:
        """Dispatch one call and block for its response."""
        return self._resolve(self._dispatch(ns, action, request, phase, extra),
                             timeout=timeout)

    # -- request builders --------------------------------------------------

    @staticmethod
    def _duration(seconds: float):
        return rclpy.duration.Duration(seconds=max(0.0, float(seconds))).to_msg()

    def _takeoff_request(self, height: float, duration: float, group_mask: int):
        request = Takeoff.Request()
        request.group_mask = int(group_mask)
        request.height = float(clamp_height(self.cfg, height))
        request.duration = self._duration(duration)
        return request

    def _land_request(self, height: float, duration: float, group_mask: int):
        request = Land.Request()
        request.group_mask = int(group_mask)
        request.height = float(height)
        request.duration = self._duration(duration)
        return request

    def _go_to_request(self, x: float, y: float, z: float, yaw: float,
                       duration: float, relative: bool, group_mask: int):
        request = GoTo.Request()
        request.group_mask = int(group_mask)
        request.relative = bool(relative)
        request.goal.x = float(x)
        request.goal.y = float(y)
        request.goal.z = float(z if relative else clamp_height(self.cfg, z))
        request.yaw = float(yaw)
        request.duration = self._duration(duration)
        return request

    # -- per-drone commands ------------------------------------------------

    def takeoff_async(self, ns: str, height: float, duration: float,
                      phase: str = '', group_mask: int = 0) -> PendingCall:
        return self._dispatch(ns, 'takeoff',
                              self._takeoff_request(height, duration, group_mask),
                              phase, {'height': clamp_height(self.cfg, height),
                                      'duration': duration})

    def takeoff(self, ns: str, height: float, duration: float,
                phase: str = '', group_mask: int = 0) -> CallRecord:
        return self._resolve(self.takeoff_async(ns, height, duration, phase, group_mask))

    def land_async(self, ns: str, height: float = 0.0, duration: float = 2.0,
                   phase: str = '', group_mask: int = 0) -> PendingCall:
        return self._dispatch(ns, 'land',
                              self._land_request(height, duration, group_mask),
                              phase, {'height': height, 'duration': duration})

    def land(self, ns: str, height: float = 0.0, duration: float = 2.0,
             phase: str = '', group_mask: int = 0) -> CallRecord:
        return self._resolve(self.land_async(ns, height, duration, phase, group_mask))

    def go_to_async(self, ns: str, x: float, y: float, z: float, yaw: float = 0.0,
                    duration: float = 2.0, relative: bool = False,
                    phase: str = '', group_mask: int = 0) -> PendingCall:
        return self._dispatch(
            ns, 'go_to',
            self._go_to_request(x, y, z, yaw, duration, relative, group_mask),
            phase, {'x': x, 'y': y, 'z': z, 'duration': duration,
                    'relative': relative})

    def go_to(self, ns: str, x: float, y: float, z: float, yaw: float = 0.0,
              duration: float = 2.0, relative: bool = False,
              phase: str = '', group_mask: int = 0) -> CallRecord:
        return self._resolve(self.go_to_async(
            ns, x, y, z, yaw, duration, relative, phase, group_mask))

    def notify_setpoints_stop_async(self, ns: str, remain_valid_millisecs: int = 100,
                                    phase: str = '', group_mask: int = 0) -> PendingCall:
        """Hand control back to the high-level commander.

        Required after streaming setpoints (``cmd_hover`` and friends),
        otherwise the streaming-priority lock makes the next Takeoff/Land a
        no-op -- a failure mode that looks exactly like a lost packet, so the
        setpoint scenarios always call this before switching back.

        It is also the cheapest command that still makes a full radio round
        trip while the drone sits on the ground, which is why the ground-safe
        ping scenario uses it.
        """
        request = NotifySetpointsStop.Request()
        request.group_mask = int(group_mask)
        request.remain_valid_millisecs = int(remain_valid_millisecs)
        return self._dispatch(ns, 'notify_setpoints_stop', request, phase,
                              {'remain_valid_millisecs': remain_valid_millisecs})

    def notify_setpoints_stop(self, ns: str, remain_valid_millisecs: int = 100,
                              phase: str = '', group_mask: int = 0) -> CallRecord:
        return self._resolve(self.notify_setpoints_stop_async(
            ns, remain_valid_millisecs, phase, group_mask))

    def arm_async(self, ns: str, armed: bool = True, phase: str = '') -> PendingCall:
        if not _ARM_AVAILABLE:
            now = time.perf_counter()
            record = CallRecord(action='arm', target=ns, phase=phase,
                                t_start=now, latency=0.0, success=False,
                                error='unsupported', wall_time=time.time(),
                                extra={'armed': armed})
            return PendingCall('arm', ns, phase, now, time.time(), None, None,
                               {'armed': armed}, resolved=record)
        request = Arm.Request()
        request.arm = bool(armed)
        return self._dispatch(ns, 'arm', request, phase, {'armed': armed})

    def arm(self, ns: str, armed: bool = True, phase: str = '') -> CallRecord:
        return self._resolve(self.arm_async(ns, armed, phase))

    def armable(self, namespaces: Optional[Sequence[str]] = None) -> List[str]:
        """Namespaces that actually advertise an ``<ns>/arm`` service."""
        if not _ARM_AVAILABLE:
            return []
        candidates = self.namespaces if namespaces is None else list(namespaces)
        ready = []
        for ns in candidates:
            client = self._srv_clients.get((ns, 'arm'))
            if client is not None and (self.dry_run or client.service_is_ready()):
                ready.append(ns)
        return ready

    def arm_all(self, namespaces: Sequence[str], armed: bool = True,
                phase: str = '') -> List[CallRecord]:
        """Arm/disarm several drones with the calls in flight together.

        Dispatched in parallel deliberately: arming is a precondition, not
        part of the manoeuvre being measured, so it should add as little skew
        as possible before the takeoff that follows.
        """
        pending = [self.arm_async(ns, armed, phase=phase) for ns in namespaces]
        return self.gather(pending, timeout=self.service_timeout)

    @staticmethod
    def arm_supported() -> bool:
        return _ARM_AVAILABLE

    def emergency(self, ns: str, phase: str = 'abort') -> CallRecord:
        """Cut the motors. The drone falls -- abort path only."""
        return self._call(ns, 'emergency', Empty.Request(), phase)

    # -- broadcast commands ------------------------------------------------

    def broadcast_takeoff_async(self, height: float, duration: float,
                                phase: str = '', group_mask: int = 0) -> PendingCall:
        return self._dispatch(self.broadcast_ns, 'takeoff',
                              self._takeoff_request(height, duration, group_mask),
                              phase, {'height': clamp_height(self.cfg, height),
                                      'duration': duration})

    def broadcast_takeoff(self, height: float, duration: float,
                          phase: str = '', group_mask: int = 0) -> CallRecord:
        return self._resolve(
            self.broadcast_takeoff_async(height, duration, phase, group_mask))

    def broadcast_land_async(self, height: float = 0.0, duration: float = 2.0,
                             phase: str = '', group_mask: int = 0) -> PendingCall:
        return self._dispatch(self.broadcast_ns, 'land',
                              self._land_request(height, duration, group_mask),
                              phase, {'height': height, 'duration': duration})

    def broadcast_land(self, height: float = 0.0, duration: float = 2.0,
                       phase: str = '', group_mask: int = 0) -> CallRecord:
        return self._resolve(
            self.broadcast_land_async(height, duration, phase, group_mask))

    def broadcast_go_to_async(self, x: float, y: float, z: float, yaw: float = 0.0,
                              duration: float = 2.0, relative: bool = True,
                              phase: str = '', group_mask: int = 0) -> PendingCall:
        return self._dispatch(
            self.broadcast_ns, 'go_to',
            self._go_to_request(x, y, z, yaw, duration, relative, group_mask),
            phase, {'x': x, 'y': y, 'z': z, 'duration': duration,
                    'relative': relative})

    def broadcast_go_to(self, x: float, y: float, z: float, yaw: float = 0.0,
                        duration: float = 2.0, relative: bool = True,
                        phase: str = '', group_mask: int = 0) -> CallRecord:
        return self._resolve(self.broadcast_go_to_async(
            x, y, z, yaw, duration, relative, phase, group_mask))

    def broadcast_emergency(self, phase: str = 'abort') -> CallRecord:
        return self._call(self.broadcast_ns, 'emergency', Empty.Request(), phase)

    # -- streaming setpoints ----------------------------------------------

    def publish_hover(self, ns: str, vx: float = 0.0, vy: float = 0.0,
                      yaw_rate: float = 0.0, z_distance: float = 0.5) -> None:
        """Publish one ``cmd_hover`` (velocity + height hold) setpoint."""
        if self.dry_run:
            return
        msg = Hover()
        msg.vx = float(vx)
        msg.vy = float(vy)
        msg.yaw_rate = float(yaw_rate)
        msg.z_distance = float(clamp_height(self.cfg, z_distance))
        self._publisher(ns, 'cmd_hover', Hover).publish(msg)

    def publish_full_state(self, ns: str, x: float, y: float, z: float,
                           yaw: float = 0.0) -> None:
        """Publish one ``cmd_full_state`` setpoint (position only, zero vel)."""
        if self.dry_run:
            return
        msg = FullState()
        msg.header.stamp = self.get_clock().now().to_msg()
        msg.header.frame_id = 'world'
        msg.pose.position.x = float(x)
        msg.pose.position.y = float(y)
        msg.pose.position.z = float(clamp_height(self.cfg, z))
        half = float(yaw) / 2.0
        msg.pose.orientation.z = math.sin(half)
        msg.pose.orientation.w = math.cos(half)
        self._publisher(ns, 'cmd_full_state', FullState).publish(msg)

    def publish_position(self, ns: str, x: float, y: float, z: float,
                         yaw: float = 0.0) -> bool:
        """Publish one ``cmd_position`` setpoint. False if unsupported."""
        if not _POSITION_AVAILABLE:
            return False
        if self.dry_run:
            return True
        msg = Position()
        msg.header.stamp = self.get_clock().now().to_msg()
        msg.header.frame_id = 'world'
        msg.x = float(x)
        msg.y = float(y)
        msg.z = float(clamp_height(self.cfg, z))
        msg.yaw = float(yaw)
        self._publisher(ns, 'cmd_position', Position).publish(msg)
        return True

    @staticmethod
    def setpoint_types() -> List[str]:
        types = ['hover', 'full_state']
        if _POSITION_AVAILABLE:
            types.append('position')
        return types
