# rebotarm_monitor

`ament_python` package that subscribes to the reBot Arm B601-DM driver topics
and publishes diagnostics in the standard `diagnostic_msgs` format.

## Node

| Field | Value |
|-------|-------|
| Package | `rebotarm_monitor` |
| Executable | `joint_state_monitor` |
| Default node name | `rebotarm_joint_state_monitor` |
| Launch file | `launch/monitor.launch.py` |

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

| Name | Source topic | Checks |
|------|--------------|--------|
| `rebotarm/hardware/joint_states` | `/rebotarm/joint_states` | rate, stale timeout, finite values, value jumps, high velocity, high effort |
| `rebotarm/joints/jointN` | `/rebotarm/joints/jointN/state` | stale timeout, finite values, value jumps, high velocity, high torque, idle torque, `status_code` |
| `rebotarm/hardware/arm_status` | `/rebotarm/arm_status` | enabled, mode, control loop active, state machine, `error_codes` |
| `rebotarm/gripper/state` | `/rebotarm/gripper/state` | stale timeout, finite values, `status_code`, high velocity, high torque |

## Parameters

Full list with defaults: `config/joint_state_monitor.yaml`.
Aggregator analyzers: `config/diagnostic_aggregator.yaml`.

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
| `status_log_period_s` | `1.0` |
| `diagnostics_period_s` | `1.0` |

## Example overrides

```bash
ros2 launch rebotarm_monitor monitor.launch.py \
  expected_rate_hz:=50.0 \
  max_abs_effort_nm:=12.0

ros2 launch rebotarm_monitor monitor.launch.py \
  use_diagnostic_aggregator:=false
```
