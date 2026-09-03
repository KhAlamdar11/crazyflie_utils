"""Ground-safe uplink benchmark: hammer a cheap service and time every call.

This is the scenario to run first. It never takes off, so it isolates the
communication path -- ROS 2 service hop, crazyflie_server, radio round trip --
from anything the flight controller does. Whatever latency and loss you see
here is the floor for every other test.

Default service is ``notify_setpoints_stop``, which the firmware answers
without touching the motors. ``arm`` (arm/disarm alternating) and ``land`` are
also available; ``land`` is the only one that can be broadcast.
"""

from __future__ import annotations

import time
from typing import Any, Dict, List

from crazyflie_utils.metrics import CallRecord, latency_stats
from crazyflie_utils.scenarios.base import StressScenario

#: Which services this scenario knows how to ping, and whether /all works.
PINGABLE = {
    'notify_setpoints_stop': {'broadcast': False},
    'arm': {'broadcast': False},
    'land': {'broadcast': True},
}


class ServicePingScenario(StressScenario):
    """Repeatedly call one service on every drone and measure the round trip."""

    name = 'service_ping'
    description = ('Ground-safe: flood one cheap service and measure round-trip '
                   'latency, timeouts and per-batch spread')
    requires_flight = False

    DEFAULT_PARAMS: Dict[str, Any] = {
        # Service to ping: notify_setpoints_stop | arm | land
        'service': 'notify_setpoints_stop',
        # parallel = all drones at once, sequential = one at a time,
        # broadcast = single /all call (land only)
        'mode': 'parallel',
        'iterations': 200,
        # Dispatch rate for the whole batch. 0 means "as fast as responses
        # come back", which is the harshest setting.
        'rate_hz': 10.0,
        'warmup_iterations': 5,
        # notify_setpoints_stop payload; 0 = release the lock immediately.
        'remain_valid_millisecs': 0,
        # Only used when service == 'land'.
        'land_height': 0.0,
        'land_duration': 1.0,
        'log_every': 25,
    }

    def run(self) -> None:
        service = str(self.params['service'])
        mode = str(self.params['mode'])
        iterations = int(self.params['iterations'])
        rate_hz = float(self.params['rate_hz'])
        warmup = int(self.params['warmup_iterations'])
        log_every = max(1, int(self.params['log_every']))

        if service not in PINGABLE:
            raise ValueError(
                f"Unknown ping service '{service}', expected one of "
                f"{sorted(PINGABLE)}")
        if mode == 'broadcast' and not PINGABLE[service]['broadcast']:
            raise ValueError(
                f"service_ping: '{service}' has no /{self.fleet.broadcast_ns} "
                f"equivalent; use mode: parallel or sequential")
        if service == 'arm' and not self.fleet.arm_supported():
            raise ValueError(
                'service_ping: crazyflie_interfaces/srv/Arm is not available in '
                'this workspace; pick another service')

        period = (1.0 / rate_hz) if rate_hz > 0 else 0.0
        self.info(
            f"service_ping: {iterations} x '{service}' to {len(self.namespaces)} "
            f"drone(s), mode={mode}, "
            f"rate={'max' if period == 0 else f'{rate_hz:g} Hz'}")

        for i in range(warmup):
            self._ping_once(service, mode, phase='warmup')
            self.sleep(period)

        batch_spreads: List[float] = []
        iteration_latency: List[float] = []
        start = time.perf_counter()

        with self.measure('ping'):
            for i in range(iterations):
                self.check_abort()
                target_time = start + (i * period) if period else 0.0
                records = self._ping_once(service, mode, phase='ping')

                if records:
                    first_send = min(r.t_start for r in records)
                    last_done = max(r.t_start + r.latency for r in records)
                    batch_spreads.append(last_done - first_send)
                    iteration_latency.extend(r.latency for r in records if r.success)

                if (i + 1) % log_every == 0:
                    ok = sum(1 for r in records if r.success)
                    self.info(
                        f"  [{i + 1}/{iterations}] batch {ok}/{len(records)} ok, "
                        f"span {batch_spreads[-1] * 1000:.1f} ms")

                if period:
                    remaining = (start + (i + 1) * period) - time.perf_counter()
                    self.sleep(max(0.0, remaining))

        if service == 'arm' and self.fleet.arm_supported():
            # Never leave the fleet armed because of a test.
            for ns in self.namespaces:
                self.fleet.arm(ns, False, phase='cleanup')

        self.metrics.set_metric('batch_span_ms', latency_stats(batch_spreads))
        self.metrics.set_metric('ping_latency_ms', latency_stats(iteration_latency))
        self.metrics.set_metric('service', service)
        self.metrics.set_metric('mode', mode)
        self._add_findings(iterations)

    # -- internals ---------------------------------------------------------

    def _ping_once(self, service: str, mode: str, phase: str) -> List[CallRecord]:
        if service == 'land':
            return self.land_fleet(
                mode,
                height=float(self.params['land_height']),
                duration=float(self.params['land_duration']),
                phase=phase)

        if mode == 'sequential':
            return [self._ping_one(service, ns, phase) for ns in self.namespaces]

        pending = [self._ping_async(service, ns, phase) for ns in self.namespaces]
        return self.fleet.gather(pending, timeout=self.service_timeout)

    def _ping_async(self, service: str, ns: str, phase: str):
        if service == 'arm':
            # Alternate so the request payload changes; the firmware still
            # answers every one.
            armed = (len(self.metrics.calls) % 2) == 0
            return self.fleet.arm_async(ns, armed, phase=phase)
        return self.fleet.notify_setpoints_stop_async(
            ns, int(self.params['remain_valid_millisecs']), phase=phase)

    def _ping_one(self, service: str, ns: str, phase: str) -> CallRecord:
        return self.fleet.wait(self._ping_async(service, ns, phase))

    def _add_findings(self, iterations: int) -> None:
        calls = [c for c in self.metrics.calls if c.phase == 'ping']
        if not calls:
            self.metrics.add_finding('No ping calls were recorded')
            return
        failed = [c for c in calls if not c.success]
        stats = latency_stats([c.latency for c in calls if c.success])
        rate = len(failed) / len(calls)

        self.metrics.add_finding(
            f"{len(calls)} calls, {len(failed)} failed ({rate * 100:.2f}%), "
            f"median {stats['median_ms']:.1f} ms, p95 {stats['p95_ms']:.1f} ms, "
            f"max {stats['max_ms']:.1f} ms")
        if rate > 0.01:
            self.metrics.add_finding(
                f"Uplink failure rate above 1% on a ground-safe ping -- the link "
                f"itself is unhealthy before any flight load is added")
        if stats['count'] and stats['max_ms'] > 10 * max(stats['median_ms'], 1.0):
            self.metrics.add_finding(
                f"Worst-case latency is {stats['max_ms'] / max(stats['median_ms'], 1.0):.0f}x "
                f"the median -- the radio is retrying or the server is blocking")
