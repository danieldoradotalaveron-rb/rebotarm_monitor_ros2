# rebotarm_monitor_ros2

Standalone ROS 2 workspace for **passive hardware monitoring** of the reBot Arm
B601-DM driver.

This repo is intentionally **separate** from the official driver workspaces
(PROD / DEV). It only subscribes to topics the driver already publishes; it does
not ship or launch the driver.

## Requirements

- ROS 2 Jazzy (or Humble)
- A built driver workspace with `rebotarm_msgs` (from upstream PROD or DEV)
- A running `reBotArmController` node

## Build

```bash
source /opt/ros/jazzy/setup.zsh
source ~/path/to/rebotarm_ros2/install/setup.zsh    # or DEV workspace
cd ~/projects/rebot/rebotarm_monitor_ros2
colcon build --packages-select rebotarm_monitor
source install/setup.zsh
```

## Run (two terminals)

**1. Driver** (official workspace — example PROD):

```bash
ros2 launch rebotarm_bringup driver_only.launch.py channel:=/dev/ttyRebotB601
```

DEV workspace: `driver.launch.py` instead of `driver_only.launch.py`.

**2. Monitor** (this workspace):

```bash
ros2 launch rebotarm_monitor monitor.launch.py
ros2 run rqt_robot_monitor rqt_robot_monitor   # optional
```

## Package layout

```
src/rebotarm_monitor/   # ament_python package
  launch/monitor.launch.py
  config/joint_state_monitor.yaml
  config/diagnostic_aggregator.yaml
```

Details: [src/rebotarm_monitor/README.md](src/rebotarm_monitor/README.md)

## License

Apache-2.0
