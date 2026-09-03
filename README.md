# crazyflie_utils

Helper and test tooling for Crazyflie fleets on ROS 2. Self-contained: it talks
to a running crazyswarm2 `crazyflie_server` through `crazyflie_interfaces`
services/topics and nothing else.

First deliverable: **communication stress tests** — how well does the radio /
server / ROS path hold up under consecutive takeoffs and landings, synchronous
vs unsynchronised fleet commands, and command floods.

## Install

```bash
cd ~/ros2_ws/src && ln -s /path/to/crazyflie_utils .
cd ~/ros2_ws && colcon build --packages-select crazyflie_utils
source install/setup.bash
```

## Run

```bash
ros2 run crazyflie_utils stress_test --list          # available scenarios
ros2 run crazyflie_utils stress_test service_ping    # ground-safe, run first
ros2 run crazyflie_utils stress_test takeoff_land_cycle --num-uavs 3
ros2 run crazyflie_utils stress_test sync_vs_async --dry-run   # rehearse
ros2 launch crazyflie_utils stress_test.launch.py scenario:=goto_flood num_uavs:=4
ros2 run crazyflie_utils link_monitor --num-uavs 4   # passive, never commands
```

Anything in the config can be overridden inline:

```bash
ros2 run crazyflie_utils stress_test takeoff_land_cycle \
  -s test.params.cycles=25 -s test.params.takeoff_height=0.8 \
  -s test.params.mode=parallel --namespaces cf1,cf3,cf7
```

`--print-config` shows the fully merged config without flying.

## Scenarios

| scenario | flies? | what it stresses |
|---|---|---|
| `service_ping` | no | Baseline uplink: floods one cheap service (`notify_setpoints_stop`), times every round trip. Run this first — it is the floor for everything else. |
| `takeoff_land_cycle` | yes | Consecutive takeoff/hover/land cycles. Per-cycle latency, pose-verified command loss, fleet skew, telemetry health, latency drift across the run. |
| `sync_vs_async` | yes | Broadcast vs parallel-unicast vs sequential-unicast dispatch, compared on uplink latency **and** real fleet skew measured from `<ns>/pose`. |
| `goto_flood` | yes | Sustained `go_to` flood at increasing rates until calls start timing out. `await_responses: false` gives a true fire-and-forget flood. |
| `setpoint_stream` | yes | Streams `cmd_hover`/`cmd_full_state` at increasing rates and measures how far the downlink collapses under uplink load. |

## Configuration

Everything lives in `config/`, layered: built-in defaults → `common.yaml`
(fleet, safety, telemetry, reporting) → the scenario file → `--set` overrides.

- `fleet`: `num_uavs` + `prefix`/`index_start`, or an explicit `namespaces` list
- `test.params`: heights, durations, cycles, rates, dispatch modes — per scenario
- `safety`: `max_height` clamp, `dry_run`, `telemetry_stall_abort` watchdog,
  arming, and what happens on abort (land by default, emergency-stop optional)
- `telemetry`: pose/status topics, `expected_pose_rate`, stall threshold
- `report`: console on/off, `output_dir`, `json`/`csv`

## Output

Console report (uplink table by action/target/phase, downlink table per drone,
scenario metrics, findings) plus `<output_dir>/<timestamp>_<scenario>.json` and
`..._calls.csv` with one row per service call.

## Safety

- Every commanded height is clamped to `safety.max_height`.
- Ctrl-C, a scenario error, and the telemetry watchdog all go through the same
  path: `notify_setpoints_stop` → broadcast land (per-drone land as fallback).
  A second Ctrl-C exits immediately without landing.
- Pre-flight refuses to fly if a service or a drone's telemetry is missing
  (`safety.require_all_services`).
- `--dry-run` executes the whole scenario without sending a command.

## Notes / limitations

- Fleet-skew resolution is bounded by the pose rate (100 ms at 10 Hz). Compare
  dispatch modes against each other, not against an absolute number.
- `cmd_hover` is a *velocity* setpoint with height hold, so long streaming
  stages drift in XY.
- Latency is measured from `call_async` to the response future's done-callback,
  so it covers the ROS hop plus the radio round trip — not firmware execution.

## Tests

`test/` covers config layering, metrics aggregation and report rendering; they
need no ROS: `pytest test/` (or `colcon test`).
