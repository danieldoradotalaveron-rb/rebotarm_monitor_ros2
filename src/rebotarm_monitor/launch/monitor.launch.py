from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.conditions import IfCondition
from launch.substitutions import (
    LaunchConfiguration,
    PathJoinSubstitution,
    PythonExpression,
)
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare


def generate_launch_description():
    pkg_share = FindPackageShare("rebotarm_monitor")
    config_file = PathJoinSubstitution([pkg_share, "config", "monitor.yaml"])
    aggregator_serial = PathJoinSubstitution(
        [pkg_share, "config", "diagnostic_aggregator.yaml"]
    )
    aggregator_can = PathJoinSubstitution(
        [pkg_share, "config", "diagnostic_aggregator_can.yaml"]
    )

    joint_states_topic = LaunchConfiguration("joint_states_topic")
    expected_rate_hz = LaunchConfiguration("expected_rate_hz")
    stale_timeout_s = LaunchConfiguration("stale_timeout_s")
    min_rate_ratio = LaunchConfiguration("min_rate_ratio")
    max_position_jump_rad = LaunchConfiguration("max_position_jump_rad")
    max_abs_velocity_rad_s = LaunchConfiguration("max_abs_velocity_rad_s")
    max_abs_effort_nm = LaunchConfiguration("max_abs_effort_nm")
    status_log_period_s = LaunchConfiguration("status_log_period_s")
    diagnostics_period_s = LaunchConfiguration("diagnostics_period_s")
    use_diagnostic_aggregator = LaunchConfiguration("use_diagnostic_aggregator")
    enable_can_monitor = LaunchConfiguration("enable_can_monitor")
    can_interfaces = LaunchConfiguration("can_interfaces")
    enable_serial_monitor = LaunchConfiguration("enable_serial_monitor")
    serial_device = LaunchConfiguration("serial_device")
    enable_process_monitor = LaunchConfiguration("enable_process_monitor")
    driver_process_pattern = LaunchConfiguration("driver_process_pattern")
    driver_process_pid = LaunchConfiguration("driver_process_pid")

    return LaunchDescription(
        [
            DeclareLaunchArgument(
                "joint_states_topic",
                default_value="/rebotarm/joint_states",
            ),
            DeclareLaunchArgument("expected_rate_hz", default_value="100.0"),
            DeclareLaunchArgument("stale_timeout_s", default_value="0.5"),
            DeclareLaunchArgument("min_rate_ratio", default_value="0.5"),
            DeclareLaunchArgument("max_position_jump_rad", default_value="0.5"),
            DeclareLaunchArgument("max_abs_velocity_rad_s", default_value="10.0"),
            DeclareLaunchArgument("max_abs_effort_nm", default_value="8.0"),
            DeclareLaunchArgument("status_log_period_s", default_value="1.0"),
            DeclareLaunchArgument(
                "diagnostics_period_s",
                default_value="0.0",
                description="0 = same as status_log_period_s",
            ),
            DeclareLaunchArgument(
                "use_diagnostic_aggregator",
                default_value="true",
                description="Start diagnostic_aggregator for rqt_robot_monitor",
            ),
            DeclareLaunchArgument(
                "enable_can_monitor",
                default_value="false",
                description=(
                    "Enable SocketCAN interface health checks. Only useful when "
                    "the driver uses channel:=can0 (not serial/USB)."
                ),
            ),
            DeclareLaunchArgument(
                "can_interfaces",
                default_value="can0",
                description="Comma-separated SocketCAN interfaces (e.g. can0,can1)",
            ),
            DeclareLaunchArgument(
                "enable_serial_monitor",
                default_value="true",
                description=(
                    "Check serial/USB device node presence. Default device "
                    "matches Seeed arm.yaml (/dev/ttyACM0)."
                ),
            ),
            DeclareLaunchArgument(
                "serial_device",
                default_value="/dev/ttyACM0",
                description=(
                    "Character device path; use the same value as the driver "
                    "channel:= launch argument."
                ),
            ),
            DeclareLaunchArgument(
                "enable_process_monitor",
                default_value="true",
                description="Enable driver process health checks (psutil)",
            ),
            DeclareLaunchArgument(
                "driver_process_pattern",
                default_value="reBotArmController",
                description="Substring match against process name or cmdline",
            ),
            DeclareLaunchArgument(
                "driver_process_pid",
                default_value="0",
                description="Force PID; 0 means auto-discover by pattern",
            ),
            Node(
                package="rebotarm_monitor",
                executable="monitor",
                name="rebotarm_monitor",
                output="screen",
                parameters=[
                    config_file,
                    {
                        "joint_states_topic": joint_states_topic,
                        "expected_rate_hz": expected_rate_hz,
                        "stale_timeout_s": stale_timeout_s,
                        "min_rate_ratio": min_rate_ratio,
                        "max_position_jump_rad": max_position_jump_rad,
                        "max_abs_velocity_rad_s": max_abs_velocity_rad_s,
                        "max_abs_effort_nm": max_abs_effort_nm,
                        "status_log_period_s": status_log_period_s,
                        "diagnostics_period_s": diagnostics_period_s,
                        "enable_can_monitor": enable_can_monitor,
                        "can_interfaces": can_interfaces,
                        "enable_serial_monitor": enable_serial_monitor,
                        "serial_device": serial_device,
                        "enable_process_monitor": enable_process_monitor,
                        "driver_process_pattern": driver_process_pattern,
                        "driver_process_pid": driver_process_pid,
                    },
                ],
            ),
            # Two aggregator instances, mutually exclusive: with vs without CAN
            # bus group. Avoids the Bus group reporting STALE on serial-only setups.
            Node(
                package="diagnostic_aggregator",
                executable="aggregator_node",
                name="diagnostic_aggregator",
                output="screen",
                parameters=[aggregator_serial],
                condition=IfCondition(
                    PythonExpression([
                        "'", use_diagnostic_aggregator, "' == 'true' and '",
                        enable_can_monitor, "' != 'true'",
                    ])
                ),
            ),
            Node(
                package="diagnostic_aggregator",
                executable="aggregator_node",
                name="diagnostic_aggregator",
                output="screen",
                parameters=[aggregator_can],
                condition=IfCondition(
                    PythonExpression([
                        "'", use_diagnostic_aggregator, "' == 'true' and '",
                        enable_can_monitor, "' == 'true'",
                    ])
                ),
            ),
        ]
    )
