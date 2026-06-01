# rebotarm_monitor

`ament_python` package that subscribes to the reBot Arm B601-DM driver topics,
polls host-side link checks (serial TTY, optional SocketCAN counters, driver
process health) and publishes diagnostics in the standard `diagnostic_msgs`
format.

## Node

| Field | Value |
|-------|-------|
| Package | `rebotarm_monitor` |
| Executable | `monitor` (alias: `joint_state_monitor`) |
| Default node name | `rebotarm_monitor` |
| Launch file | `launch/monitor.launch.py` |
| YAML defaults | `config/monitor.yaml` |
| Aggregator analyzers (serial) | `config/diagnostic_aggregator.yaml` |
| Aggregator analyzers (CAN) | `config/diagnostic_aggregator_can.yaml` |

## Subscribed topics

| Topic | Type |
|-------|------|
| `/rebotarm/joint_states` | `sensor_msgs/JointState` |
| `/rebotarm/joints/jointN/state` (N=1..6) | `rebotarm_msgs/JointMotorState` |
| `/rebotarm/arm_status` (latched) | `rebotarm_msgs/ArmStatus` |
| `/rebotarm/gripper/state` | `rebotarm_msgs/JointMotorState` |

## Published topics

| Topic | Type |
|-------|------|
| `/diagnostics` | `diagnostic_msgs/DiagnosticArray` |
| `/diagnostics_agg` | `diagnostic_msgs/DiagnosticArray` |
| `/diagnostics_toplevel_state` | `diagnostic_msgs/DiagnosticStatus` |

`/diagnostics_agg` and `/diagnostics_toplevel_state` are produced by
`diagnostic_aggregator`, launched together with the monitor when
`use_diagnostic_aggregator:=true` (default).

The launch picks one of two aggregator configs based on `enable_can_monitor`:

| `enable_can_monitor` | Config file | rqt groups |
|----------------------|-------------|------------|
| `false` (default) | `diagnostic_aggregator.yaml` | Control, Gripper, Hardware, Joints, Link, System |
| `true` | `diagnostic_aggregator_can.yaml` | the above + Bus |

In `rqt_robot_monitor`, expand **RebotArm → System** for
`rebotarm/system/monitor_config` (active `payload_profile` in the Message
column) and `rebotarm/system/driver`.

## Diagnostic names

| Name | Source | Checks |
|------|--------|--------|
| `rebotarm/hardware/joint_states` | `/rebotarm/joint_states` | rate, stale timeout, finite values, value jumps, high velocity, high effort |
| `rebotarm/control/arm_status` | `/rebotarm/arm_status` | `enabled`, `mode`, `control_loop_active`, `state_machine`, `error_codes` |
| `rebotarm/control/gravity_compensation` | `/rebotarm/arm_status` | `gravity_compensation_active` when `state_machine == GRAVITY_COMP` (not inferred from `mode`); `state_machine`, `mode`, `enabled`, `control_loop_active` |
| `rebotarm/joints/jointN` | `/rebotarm/joints/jointN/state` | stale timeout, finite values, value jumps, high velocity, high torque, stationary idle torque (suppressed during gravity compensation or position hold), `status_code` |
| `rebotarm/gripper/state` | `/rebotarm/gripper/state` | stale timeout, finite values, high velocity, high torque, `status_code` |
| `rebotarm/link/serial` | host device node | path exists, character device, read/write permissions |
| `rebotarm/bus/<iface>` | `/sys/class/net/<iface>` | `operstate`, `rx_errors`, `tx_errors`, `rx_dropped`, `tx_dropped` (only when `enable_can_monitor:=true`) |
| `rebotarm/system/monitor_config` | node parameters | active `payload_profile` and assumed payload mass |
| `rebotarm/system/driver` | `psutil` | resolution by name/PID, CPU%, RSS, threads, FDs, zombie/stopped state |

## Link layer (serial vs CAN)

The driver `channel` argument selects the transport. The monitor mirrors that
choice with separate toggles; it does not parse the driver's parameters.

| Transport | Driver | Monitor defaults |
|-----------|--------|------------------|
| USB serial | `channel:=/dev/ttyACM0` | `enable_serial_monitor:=true`, `serial_device:=/dev/ttyACM0` |
| USB + udev symlink | `channel:=/dev/ttyRebotB601` | `serial_device:=/dev/ttyRebotB601` |
| SocketCAN | `channel:=can0` | `enable_serial_monitor:=false`, `enable_can_monitor:=true` |

Set `serial_device` to the same path used for the driver's `channel:=`
argument. The default `/dev/ttyACM0` matches Seeed's `arm.yaml`.

## Parameters

ROS 2 scalar parameters resolve in this order (lowest → highest precedence):

1. Defaults in `rebotarm_monitor/parameters.py` (`_PARAM_SPECS`).
2. `config/monitor.yaml` (loaded by the launch file).
3. The inline parameter dict in `monitor.launch.py`.
4. CLI overrides on `ros2 launch ... key:=value`.

After scalars are resolved, `load_params()` injects the B601 per-joint
threshold maps from the resolved `payload_profile`. Those maps are Python
constants, not ROS parameters (`rclpy` rejects `dict` types), so they
cannot be overridden from YAML or the launch file.

`config/monitor.yaml` uses the wildcard `/**:` block so it applies regardless
of the node name you launch with.

Most-used parameters:

| Parameter | Default |
|-----------|---------|
| `expected_rate_hz` | `100.0` |
| `stale_timeout_s` | `0.5` |
| `min_rate_ratio` | `0.5` |
| `max_position_jump_rad` | `0.5` |
| `max_abs_velocity_rad_s` | `10.0` |
| `max_abs_effort_nm` | `8.0` |
| `idle_velocity_threshold_rad_s` | `0.05` |
| `idle_torque_warn_nm` | `3.0` (global fallback for joints not in the B601 per-joint map) |
| `max_abs_joint_torque_nm` | `8.0` (global fallback for joints not in the B601 per-joint map) |
| `payload_profile` | `"light"` (selects per-joint torque maps; see "Payload profiles") |
| `per_joint_idle_torque_warn_nm` | B601 map via `load_params()`, varies by profile |
| `per_joint_max_abs_torque_nm` | B601 map via `load_params()`, varies by profile |
| `per_joint_max_abs_velocity_rad_s` | B601 map via `load_params()` (profile-independent, joint1–3: 6.0, joint4–6: 20.0 rad/s) |
| `arm_status_stale_timeout_s` | `1.0` |
| `arm_status_warn_on_snapshot_age` | `false` |
| `gripper_stale_timeout_s` | `1.0` |
| `enable_serial_monitor` | `true` |
| `serial_device` | `"/dev/ttyACM0"` |
| `enable_can_monitor` | `false` |
| `can_interfaces` | `"can0"` (comma-separated) |
| `can_error_warn_per_period` | `1` |
| `can_dropped_warn_per_period` | `10` |
| `enable_process_monitor` | `true` |
| `driver_process_pattern` | `"reBotArmController"` |
| `driver_cpu_warn_percent` | `90.0` |
| `driver_rss_warn_mb` | `1024.0` |
| `driver_threads_warn` | `64` |
| `status_log_period_s` | `1.0` |
| `diagnostics_period_s` | `1.0` via `monitor.yaml`; launch default `0.0` (= same as `status_log_period_s`) |

### Per-joint torque behavior

`MonitorOrchestrator` builds a `TrackerContext` each cycle from
`ArmStatusTracker.last_msg`:

- `arm_enabled` — forwarded to per-joint `status_code` checks.
- `gravity_compensation_active` — `True` when `state_machine == GRAVITY_COMP`.
- `position_hold_active` — `True` when `mode == pos_vel`, `enabled`, and `control_loop_active`.
- `control_context` — `gravity_compensation`, `position_hold`, or `normal_or_unknown`.

`PerJointTracker` uses the context as follows. Torque and velocity
levels are derived from the latest sample (`last_msg`), not latched
across the diagnostic period, so the message always matches what the joint
is doing now. Position/torque jumps still latch within the period. Peak
|T| and |v| seen during the period are exposed as KeyValues.

| Check | `normal_or_unknown` | `gravity_compensation` or `position_hold` |
|-------|---------------------|-------------------------------------------|
| Stationary effort (\|vel\| below `idle_velocity_threshold_rad_s`, \|torque\| above `idle_torque_warn_nm`, at or below max torque) | WARN (`high stationary effort`, includes \|T\| and \|v\|) | OK with KeyValues (`load_state=elevated`, `stationary_effort_check_suppressed=true`) |
| Absolute high torque (\|torque\| above `max_abs_joint_torque_nm`) | WARN (includes \|T\| and \|v\|) | WARN (includes \|T\|, \|v\|, and holding context) |
| Absolute high velocity (\|velocity\| above `max_abs_joint_velocity_rad_s`) | WARN (includes \|T\| and \|v\|) | WARN (includes \|T\|, \|v\|, and holding context) |

Do not infer gravity compensation from `mode == "mit"`.

### Payload profiles

Per-joint torque thresholds depend on the expected gripper payload. The
`payload_profile` parameter selects one of three profiles at node startup
(default `light`):

| Profile | Assumed payload | Idle j1/j2/j3/j4/j5/j6 (Nm) | Max j1–3 / j4–6 (Nm) |
|---------|-----------------|------------------------------|----------------------|
| `light` (default) | 0.5 kg | 1 / 8 / 8 / 2.5 / 1.5 / 1 | 9 / 3 |
| `medium` | 1.0 kg | 1 / 10 / 10 / 3 / 2 / 1 | 12 / 4 |
| `rated` | 1.5 kg | 1 / 14 / 12 / 4 / 2.5 / 1 | 18 / 5.5 |

Per-joint velocity limits are profile-independent (joint1–3: 6.0 rad/s,
joint4–6: 20.0 rad/s). The active profile appears in **System →
monitor_config** and as a `payload_profile` KeyValue on each per-joint
diagnostic. An unknown profile name prevents the node from starting.
Restart the monitor to change profile.

Derivation notes and operational edge cases:
[`docs/per-joint-thresholds.md`](../../docs/per-joint-thresholds.md).

Per-joint `message` lines always show live |T| and |v| against the resolved
limits. WARN lines keep both measurements and name the failing check (for
example, `joint3 high torque |T|=9.5/9.0 Nm |v|=0.0/6.0 rad/s`).

## Example overrides

```bash
ros2 launch rebotarm_monitor monitor.launch.py \
  expected_rate_hz:=50.0 \
  max_abs_effort_nm:=12.0

# Switch payload profile (light / medium / rated)
ros2 launch rebotarm_monitor monitor.launch.py \
  payload_profile:=medium

# USB serial with udev symlink (match driver channel:=)
ros2 launch rebotarm_monitor monitor.launch.py \
  serial_device:=/dev/ttyRebotB601

# SocketCAN
ros2 launch rebotarm_monitor monitor.launch.py \
  enable_serial_monitor:=false \
  enable_can_monitor:=true \
  can_interfaces:=can0

ros2 launch rebotarm_monitor monitor.launch.py \
  use_diagnostic_aggregator:=false
```

## Internal layout

```
rebotarm_monitor/
├── node.py             # ROS 2 adapter (rclpy + timers + publisher)
├── orchestrator.py     # owns trackers, builds DiagnosticArray
├── factories.py        # composition root: params dict → trackers
├── parameters.py       # declare + load ROS parameters
├── domain/             # HealthTracker ABC, TrackerContext
├── trackers/           # one strategy per concern
├── adapters/           # SysFsReader, ProcessInspector, DevicePathInspector
└── support/            # diagnostics helpers, rate window
```

Trackers depend only on `domain/` and `adapters/`; the orchestrator reads
`ArmStatusTracker` once per cycle to populate `TrackerContext` for sibling
trackers (including per-joint stationary-effort suppression). The node only
touches `parameters.py`, `factories.py`, and the orchestrator. Core logic is
testable without rclpy.

## Tests

```bash
colcon test --packages-select rebotarm_monitor
colcon test-result --verbose
```

Tests use `FakeSysFsReader`, `FakeProcessInspector`, and
`FakeDevicePathInspector` so they run without a real CAN device, TTY, or
driver process.
