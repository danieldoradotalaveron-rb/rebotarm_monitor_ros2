# rebotarm_monitor_ros2

[![ROS 2](https://img.shields.io/badge/ROS%202-Jazzy%20%7C%20Humble-22314E?logo=ros&logoColor=white)](https://docs.ros.org/)
[![Python](https://img.shields.io/badge/Python-3.10%2B-3776AB?logo=python&logoColor=white)](https://www.python.org/)
[![License: Apache 2.0](https://img.shields.io/badge/License-Apache_2.0-blue.svg)](https://opensource.org/licenses/Apache-2.0)
[![Build system: colcon](https://img.shields.io/badge/build-colcon-9cf)](https://colcon.readthedocs.io/)

ROS 2 workspace with a single package, `rebotarm_monitor`, that subscribes to
the topics published by a running reBot Arm B601-DM driver, polls a few host
metrics (SocketCAN counters, driver process health), and publishes
`diagnostic_msgs/DiagnosticArray` on `/diagnostics`.

<p align="center">
  <img src="docs/hero.png" alt="rebotarm_monitor_ros2 — ROS 2 diagnostics monitor for reBot Arm B601-DM" width="92%"/>
</p>

## Contents

- [Requirements](#requirements)
- [Build](#build)
- [Run](#run)
- [What it reports](#what-it-reports)
- [Configuration](#configuration)
- [Architecture](#architecture)
- [Testing](#testing)
- [Repository layout](#repository-layout)
- [License](#license)

## Requirements

- ROS 2 Jazzy (also tested on Humble).
- A sourced ROS 2 workspace that provides `rebotarm_msgs` (built from the
  Seeed reBot Arm driver workspace with `colcon`).
- A running `reBotArmController` node.
- `python3-psutil` (declared as an `exec_depend`; install with `rosdep` or
  `apt install python3-psutil`).

## Build

```bash
source /opt/ros/jazzy/setup.bash
source /path/to/driver-workspace/install/setup.bash   # provides rebotarm_msgs
cd /path/to/rebotarm_monitor_ros2
colcon build --packages-select rebotarm_monitor
source install/setup.bash
```

## Run

```bash
ros2 launch rebotarm_monitor monitor.launch.py
ros2 run rqt_robot_monitor rqt_robot_monitor   # optional GUI
```

The launch file starts the monitor node plus a `diagnostic_aggregator` so the
output is ready to be visualised in `rqt_robot_monitor`.

### Aggregator groups in rqt

The launch loads one of two aggregator configs based on `enable_can_monitor`:

| `enable_can_monitor` | rqt tree (`RebotArm/…`) |
|----------------------|------------------------|
| `false` (default — USB/serial) | Control, Gripper, Hardware, Joints, Link, System |
| `true` (SocketCAN) | Control, Gripper, Hardware, Joints, Link, System, **Bus** |

### Driver channel and link monitoring

The Seeed driver connects over a **character device** (`channel` launch
argument). The default in their `arm.yaml` is `/dev/ttyACM0` (USB serial).
Some setups use a **udev symlink** (e.g. `/dev/ttyRebotB601`) or **SocketCAN**
(`channel:=can0`) instead.

The monitor does not read the driver's `channel` parameter. Configure the link
layer explicitly so it matches how you launch the driver:

| Driver transport | Driver launch | Monitor launch |
|------------------|---------------|----------------|
| USB serial (default) | `channel:=/dev/ttyACM0` | default (no extra args) |
| USB serial + udev symlink | `channel:=/dev/ttyRebotB601` | `serial_device:=/dev/ttyRebotB601` |
| SocketCAN | `channel:=can0` | `enable_serial_monitor:=false enable_can_monitor:=true` |

Example with a custom symlink:

```bash
# Terminal 1 — driver
ros2 launch rebotarm_bringup driver_only.launch.py channel:=/dev/ttyRebotB601

# Terminal 2 — monitor (same device path)
ros2 launch rebotarm_monitor monitor.launch.py serial_device:=/dev/ttyRebotB601
```

## What it reports

Each `DiagnosticStatus` in `/diagnostics` corresponds to one tracker. By
default every tracker is enabled; turn off the ones you don't need via the
launch arguments listed below.

| Diagnostic name | Source | Default |
|-----------------|--------|---------|
| `rebotarm/hardware/joint_states` | `/rebotarm/joint_states` | on |
| `rebotarm/control/arm_status` | `/rebotarm/arm_status` (latched) | on |
| `rebotarm/control/gravity_compensation` | `/rebotarm/arm_status` (latched) | on (with arm status monitor) |
| `rebotarm/joints/jointN` (N=1..6) | `/rebotarm/joints/jointN/state` | on |
| `rebotarm/gripper/state` | `/rebotarm/gripper/state` | on |
| `rebotarm/link/serial` | host device node | on (default `/dev/ttyACM0`, Seeed standard) |
| `rebotarm/bus/<iface>` | `/sys/class/net/<iface>` counters | off (only with `enable_can_monitor:=true`) |
| `rebotarm/system/driver` | `psutil` lookup of the driver process | on |

Topics published:

| Topic | Type |
|-------|------|
| `/diagnostics` | `diagnostic_msgs/DiagnosticArray` |
| `/diagnostics_agg` | `diagnostic_msgs/DiagnosticArray` (from aggregator) |
| `/diagnostics_toplevel_state` | `diagnostic_msgs/DiagnosticStatus` (from aggregator) |

## Configuration

ROS 2 parameters resolve in this order (lowest → highest precedence):

1. Defaults declared in `rebotarm_monitor/parameters.py`.
2. YAML loaded by the launch file (`config/monitor.yaml`).
3. The `LaunchConfiguration` dict passed in the launch (overrides #2 for the
   subset of parameters the launch exposes as `DeclareLaunchArgument`).
4. CLI overrides: `ros2 launch ... key:=value`.

The most-used parameters are exposed as launch arguments:

| Argument | Default |
|----------|---------|
| `joint_states_topic` | `/rebotarm/joint_states` |
| `expected_rate_hz` | `100.0` |
| `stale_timeout_s` | `0.5` |
| `min_rate_ratio` | `0.5` |
| `max_position_jump_rad` | `0.5` |
| `max_abs_velocity_rad_s` | `10.0` |
| `max_abs_effort_nm` | `8.0` |
| `status_log_period_s` | `1.0` |
| `diagnostics_period_s` | `0.0` (means same as `status_log_period_s`) |
| `enable_serial_monitor` | `true` |
| `serial_device` | `/dev/ttyACM0` (same path as driver `channel:=`) |
| `enable_can_monitor` | `false` (set `true` for SocketCAN) |
| `can_interfaces` | `can0` (comma-separated list, e.g. `can0,can1`) |
| `enable_process_monitor` | `true` |
| `driver_process_pattern` | `reBotArmController` |
| `driver_process_pid` | `0` (auto-discover by pattern) |
| `use_diagnostic_aggregator` | `true` |

Example overrides:

```bash
ros2 launch rebotarm_monitor monitor.launch.py \
  expected_rate_hz:=50.0 \
  max_abs_effort_nm:=12.0

# SocketCAN setup
ros2 launch rebotarm_monitor monitor.launch.py \
  enable_serial_monitor:=false \
  enable_can_monitor:=true \
  can_interfaces:=can0

# USB serial with udev symlink (must match driver channel:=)
ros2 launch rebotarm_monitor monitor.launch.py \
  serial_device:=/dev/ttyRebotB601
```

For the full list (per-joint thresholds, status-code mapping, etc.) edit
`config/monitor.yaml`.

## Architecture

The package follows a small hexagonal layout. Trackers are the strategies,
the orchestrator is the application service, and adapters isolate the only
parts that touch the outside world (filesystem, `psutil`):

```
rebotarm_monitor/
├── node.py             # ROS 2 adapter: params, publisher, timers
├── orchestrator.py     # registers trackers + builds DiagnosticArray
├── factories.py        # composition root: params dict → trackers
├── parameters.py       # declare + load ROS parameters
├── domain/             # HealthTracker contract + TrackerContext
├── trackers/           # one file per concern (joint_states, per_joint,
│                       # arm_status, gravity_compensation, gripper,
│                       # serial_link, can_bus, process)
├── adapters/           # SysFsReader, ProcessInspector, DevicePathInspector
└── support/            # diagnostics helpers, rate window
```

Adding a new tracker = drop a file in `trackers/`, implement the
`HealthTracker` ABC, and register it in `factories.build_trackers`.

## Testing

Unit tests live next to the package and use the in-memory adapter fakes
(`FakeSysFsReader`, `FakeProcessInspector`, `FakeDevicePathInspector`) so no
real CAN interface, TTY device, or driver process is needed.

```bash
colcon test --packages-select rebotarm_monitor
colcon test-result --verbose
```

Or run pytest directly inside the package once the workspace is sourced:

```bash
cd src/rebotarm_monitor
pytest
```

## Repository layout

```
rebotarm_monitor_ros2/
├── docs/
│   └── hero.png
└── src/
    └── rebotarm_monitor/        # ament_python package
        ├── config/
        ├── launch/
        ├── rebotarm_monitor/    # source
        └── test/                # unit tests
```

See [`src/rebotarm_monitor/README.md`](src/rebotarm_monitor/README.md) for the
package-level reference (diagnostic names, full parameter list, examples).

## License

Released under the [Apache License 2.0](https://www.apache.org/licenses/LICENSE-2.0).
