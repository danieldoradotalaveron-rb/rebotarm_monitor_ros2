# rebotarm_monitor

Passive external health monitor for reBot Arm ROS 2 drivers. Subscribes only to
existing driver topics — **does not control the robot**, does not open
MotorBridge/CAN, and does not modify the driver or SDK.

Compatible with:

- [rebotarm_ros2](https://github.com/EclipseaHime017/reBotArmController_ROS2) (PROD-style workspace)
- [reBotArmController_ROS2_dev](https://github.com/EclipseaHime017/reBotArmController_ROS2) (DEV + MoveIt)

Requires a running driver that publishes the standard `/rebotarm/*` topics and
`rebotarm_msgs` installed from the driver workspace.

## Quick start

Terminal 1 — driver (from **their** workspace, not this repo):

```bash
source /opt/ros/jazzy/setup.zsh
source ~/path/to/driver-workspace/install/setup.zsh
ros2 launch rebotarm_bringup driver_only.launch.py channel:=/dev/ttyRebotB601
# DEV workspace uses: driver.launch.py instead of driver_only.launch.py
```

Terminal 2 — monitor (this repo):

```bash
source /opt/ros/jazzy/setup.zsh
source ~/path/to/driver-workspace/install/setup.zsh   # rebotarm_msgs
cd ~/projects/rebot/rebotarm_monitor_ros2
colcon build --packages-select rebotarm_monitor
source install/setup.zsh
ros2 launch rebotarm_monitor monitor.launch.py
```

Optional GUI:

```bash
ros2 run rqt_robot_monitor rqt_robot_monitor
```

## What it checks

| Diagnostic name | Source topic | Checks |
|-----------------|--------------|--------|
| `rebotarm/hardware/joint_states` | `/rebotarm/joint_states` | rate, stale, arrays, finite values, jumps, high vel/effort |
| `rebotarm/joints/jointN` | `/rebotarm/joints/jointN/state` | stale, finite values, status_code, high vel/torque, idle torque, jumps |
| `rebotarm/hardware/arm_status` | `/rebotarm/arm_status` | latched snapshot age, enabled, mode, control loop, state machine, error_codes |
| `rebotarm/gripper/state` | `/rebotarm/gripper/state` | stale, finite values, status_code, high vel/torque |

See package config `config/joint_state_monitor.yaml` for thresholds.

## Topics and parameters

Default topics assume namespace `rebotarm`. Override via launch args or YAML if
your driver uses a different `arm_namespace`.

## Intentionally not covered

- Motor temperature, phase current, bus voltage, CAN error counters
- Certified safety / E-stop integration
- Motor control or command paths
- Gravity compensation debug telemetry
