"""Passive downlink monitor for a Crazyflie fleet.

Subscribes to every drone's ``<ns>/pose`` and (when the message type is
available) ``<ns>/status`` topic and keeps enough history to answer the
questions a communication stress test asks:

* Is telemetry still flowing at the expected rate while we hammer the uplink?
* How long was the longest silence (radio dropout) on each drone?
* What did RSSI and battery voltage do during the run?
* When did drone *n* actually start moving? (used to measure how far apart a
  "synchronised" takeoff really lands on the fleet)

Nothing here commands a drone -- it is safe to run on its own via the
``link_monitor`` entry point while some other stack is flying.
"""

from __future__ import annotations

import threading
import time
from collections import deque
from typing import Any, Deque, Dict, List, Optional, Tuple

from geometry_msgs.msg import PoseStamped
from rclpy.node import Node
from rclpy.qos import QoSHistoryPolicy, QoSProfile, QoSReliabilityPolicy

from crazyflie_utils.metrics import TelemetryWindow

try:  # crazyflie_interfaces gained Status later than the rest of the API
    from crazyflie_interfaces.msg import Status as CFStatus
    _STATUS_AVAILABLE = True
except ImportError:  # pragma: no cover - depends on the installed interfaces
    CFStatus = None  # type: ignore[assignment]
    _STATUS_AVAILABLE = False


class _DroneLink:
    """Rolling history of one drone's downlink."""

    def __init__(self, namespace: str, history_length: int):
        self.namespace = namespace
        # (perf_counter receive time, z) -- receive time, not the header stamp,
        # because we are measuring when data reached us, not when the drone
        # thought it was there.
        self.pose_history: Deque[Tuple[float, float]] = deque(maxlen=history_length)
        # (perf_counter, rssi, battery_voltage)
        self.status_history: Deque[Tuple[float, Optional[float], Optional[float]]] = \
            deque(maxlen=history_length)
        self.pose_total = 0
        self.status_total = 0
        self.last_pose_time: Optional[float] = None
        self.last_pose: Optional[Tuple[float, float, float]] = None


class LinkMonitor:
    """Fleet-wide downlink bookkeeping, attached to an existing node."""

    def __init__(self, node: Node, namespaces: List[str], cfg: Dict[str, Any]):
        self._node = node
        self._cfg = cfg
        self._lock = threading.Lock()
        self._enabled = bool(cfg.get('enabled', True))
        self._expected_rate = float(cfg.get('expected_pose_rate', 10.0))
        self._stall_threshold = float(cfg.get('stall_threshold', 0.5))
        history_length = int(cfg.get('history_length', 4000))

        self.links: Dict[str, _DroneLink] = {
            ns: _DroneLink(ns, history_length) for ns in namespaces
        }
        self._subs: List[Any] = []
        self.status_available = _STATUS_AVAILABLE

        if not self._enabled:
            node.get_logger().info('Link monitor disabled by config')
            return

        # BEST_EFFORT on the subscriber side is compatible with both a
        # best-effort and a reliable publisher, so this works against a stock
        # crazyflie_server as well as customised ones.
        qos = QoSProfile(
            reliability=QoSReliabilityPolicy.BEST_EFFORT,
            history=QoSHistoryPolicy.KEEP_LAST,
            depth=int(cfg.get('qos_depth', 10)),
        )
        pose_topic = str(cfg.get('pose_topic', 'pose'))
        status_topic = str(cfg.get('status_topic', 'status'))

        for ns in namespaces:
            self._subs.append(node.create_subscription(
                PoseStamped, f"/{ns}/{pose_topic}",
                self._make_pose_cb(ns), qos))
            if _STATUS_AVAILABLE:
                self._subs.append(node.create_subscription(
                    CFStatus, f"/{ns}/{status_topic}",
                    self._make_status_cb(ns), qos))

        if not _STATUS_AVAILABLE:
            node.get_logger().warn(
                'crazyflie_interfaces/msg/Status not available -- RSSI and '
                'battery will be omitted from the report'
            )

    # -- subscriptions -----------------------------------------------------

    def _make_pose_cb(self, ns: str):
        link = self.links[ns]

        def _cb(msg: PoseStamped) -> None:
            now = time.perf_counter()
            position = msg.pose.position
            with self._lock:
                link.pose_history.append((now, float(position.z)))
                link.pose_total += 1
                link.last_pose_time = now
                link.last_pose = (float(position.x), float(position.y), float(position.z))

        return _cb

    def _make_status_cb(self, ns: str):
        link = self.links[ns]

        def _cb(msg) -> None:
            now = time.perf_counter()
            # Field names have shifted between crazyflie_interfaces releases,
            # so read defensively rather than hard-failing mid-flight.
            rssi = getattr(msg, 'rssi', None)
            battery = getattr(msg, 'battery_voltage', None)
            with self._lock:
                link.status_history.append((
                    now,
                    float(rssi) if rssi is not None else None,
                    float(battery) if battery is not None else None,
                ))
                link.status_total += 1

        return _cb

    # -- queries -----------------------------------------------------------

    @property
    def enabled(self) -> bool:
        return self._enabled

    def wait_for_telemetry(self, timeout: float) -> List[str]:
        """Block until every drone has published at least one pose.

        Returns the namespaces that stayed silent (empty list = all present),
        which the runner treats as "these drones are not connected".
        """
        if not self._enabled:
            return []
        deadline = time.perf_counter() + timeout
        while time.perf_counter() < deadline:
            silent = [ns for ns, link in self.links.items() if link.pose_total == 0]
            if not silent:
                return []
            time.sleep(0.05)
        with self._lock:
            return [ns for ns, link in self.links.items() if link.pose_total == 0]

    def latest_z(self, ns: str) -> Optional[float]:
        with self._lock:
            link = self.links.get(ns)
            if link is None or link.last_pose is None:
                return None
            return link.last_pose[2]

    def latest_position(self, ns: str) -> Optional[Tuple[float, float, float]]:
        with self._lock:
            link = self.links.get(ns)
            return None if link is None or link.last_pose is None else link.last_pose

    def seconds_since_pose(self, ns: str) -> Optional[float]:
        """Age of the newest pose message, or ``None`` if there never was one."""
        with self._lock:
            link = self.links.get(ns)
            if link is None or link.last_pose_time is None:
                return None
            return time.perf_counter() - link.last_pose_time

    def first_time_above(self, ns: str, z_threshold: float,
                         since: float) -> Optional[float]:
        """Receive time of the first pose after ``since`` with ``z`` above the
        threshold, or ``None`` if the drone never got there.

        Resolution is bounded by the pose rate (100 ms at the default 10 Hz),
        so treat the derived motion-start spread as an estimate, not a
        microsecond-accurate measurement.
        """
        with self._lock:
            link = self.links.get(ns)
            if link is None:
                return None
            for stamp, z in link.pose_history:
                if stamp >= since and z >= z_threshold:
                    return stamp
        return None

    # -- measurement windows ----------------------------------------------

    @staticmethod
    def begin_window() -> float:
        """Timestamp to hand back to :meth:`close_window` later."""
        return time.perf_counter()

    def close_window(self, label: str, t_start: float,
                     t_end: Optional[float] = None) -> List[TelemetryWindow]:
        """Summarise every drone's downlink over ``[t_start, t_end]``."""
        if not self._enabled:
            return []
        t_end = time.perf_counter() if t_end is None else t_end
        duration = max(1e-9, t_end - t_start)
        windows: List[TelemetryWindow] = []

        with self._lock:
            for ns, link in self.links.items():
                stamps = [t for t, _ in link.pose_history if t_start <= t <= t_end]
                # Include the last sample before the window so a dropout that
                # straddles the window start is still counted.
                prior = [t for t, _ in link.pose_history if t < t_start]
                edges = ([prior[-1]] if prior else [t_start]) + stamps + [t_end]

                gaps = [b - a for a, b in zip(edges, edges[1:])]
                max_gap = max(gaps) if gaps else duration
                stall_gaps = [g for g in gaps if g > self._stall_threshold]

                rate = len(stamps) / duration
                expected = self._expected_rate if self._expected_rate > 0 else float('nan')

                status_rows = [row for row in link.status_history
                               if t_start <= row[0] <= t_end]
                rssis = [row[1] for row in status_rows if row[1] is not None]
                batteries = [row[2] for row in status_rows if row[2] is not None]

                windows.append(TelemetryWindow(
                    namespace=ns,
                    label=label,
                    duration=duration,
                    pose_count=len(stamps),
                    pose_rate=rate,
                    expected_rate=expected,
                    rate_ratio=(rate / expected) if expected and expected == expected else float('nan'),
                    max_gap=max_gap,
                    stalls=len(stall_gaps),
                    stalled_time=sum(stall_gaps),
                    status_count=len(status_rows),
                    rssi_mean=(sum(rssis) / len(rssis)) if rssis else None,
                    rssi_min=min(rssis) if rssis else None,
                    battery_start=batteries[0] if batteries else None,
                    battery_end=batteries[-1] if batteries else None,
                ))

        return windows

    def destroy(self) -> None:
        for sub in self._subs:
            try:
                self._node.destroy_subscription(sub)
            except Exception:
                pass
        self._subs.clear()
