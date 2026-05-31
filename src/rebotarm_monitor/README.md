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
| Aggregator analyzers | `config/diagnostic_aggregator.yaml` |

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

## Diagnostic names

| Name | Source | Checks |
|------|--------|--------|
| `rebotarm/hardware/joint_states` | `/rebotarm/joint_states` | rate, stale timeout, finite values, value jumps, high velocity, high effort |
| `rebotarm/joints/jointN` | `/rebotarm/joints/jointN/state` | stale timeout, finite values, value jumps, high velocity, high torque, idle torque, `status_code` |
| `rebotarm/hardware/arm_status` | `/rebotarm/arm_status` | `enabled`, `mode`, `control_loop_active`, `state_machine`, `error_codes` |
| `rebotarm/gripper/state` | `/rebotarm/gripper/state` | stale timeout, finite values, high velocity, high torque, `status_code` |
| `rebotarm/link/serial` | host device node | path exists, character device, read/write permissions |
| `rebotarm/bus/<iface>` | `/sys/class/net/<iface>` | `operstate`, `rx_errors`, `tx_errors`, `rx_dropped`, `tx_dropped` (delta per cycle) |
| `rebotarm/system/driver` | `psutil` | resolution by name/PID, CPU%, RSS, threads, FDs, zombie/stopped state |

## Link layer (serial vs CAN)

The driver `channel` argument selects the transport. The monitor mirrors that
choice with separate toggles — it does **not** parse the driver's parameters.

| Transport | Driver | Monitor defaults |
|-----------|--------|------------------|
| USB serial | `channel:=/dev/ttyACM0` | `enable_serial_monitor:=true`, `serial_device:=/dev/ttyACM0` |
| USB + udev symlink | `channel:=/dev/ttyRebotB601` | `serial_device:=/dev/ttyRebotB601` |
| SocketCAN | `channel:=can0` | `enable_serial_monitor:=false`, `enable_can_monitor:=true` |

Always pass the **same device path** to `serial_device` that you use for the
driver's `channel:=` argument. The default `/dev/ttyACM0` matches Seeed's
`arm.yaml`; custom udev symlinks are supported but never hardcoded.

## Parameters

ROS 2 parameter resolution order (lowest → highest):

1. Defaults declared in `rebotarm_monitor/parameters.py`.
2. `config/monitor.yaml` loaded by the launch file.
3. The `LaunchConfiguration` dict in `monitor.launch.py`.
4. CLI overrides on `ros2 launch ... key:=value`.

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
| `idle_torque_warn_nm` | `3.0` |
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
| `diagnostics_period_s` | `1.0` |

## Example overrides

```bash
ros2 launch rebotarm_monitor monitor.launch.py \
  expected_rate_hz:=50.0 \
  max_abs_effort_nm:=12.0

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

Trackers depend only on `domain/` and `adapters/`; the orchestrator only on
`domain/`; the node only on `parameters.py`, `factories.py`, and the
orchestrator. This keeps logic testable without rclpy.

## Tests

```bash
colcon test --packages-select rebotarm_monitor
colcon test-result --verbose
```

Tests use `FakeSysFsReader`, `FakeProcessInspector`, and
`FakeDevicePathInspector` so they run without a real CAN device, TTY, or
driver process.
