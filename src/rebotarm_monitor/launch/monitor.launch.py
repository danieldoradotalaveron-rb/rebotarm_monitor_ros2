from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.conditions import IfCondition
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare


def generate_launch_description():
    pkg_share = FindPackageShare("rebotarm_monitor")
    config_file = PathJoinSubstitution(
        [pkg_share, "config", "joint_state_monitor.yaml"]
    )
    aggregator_config = PathJoinSubstitution(
        [pkg_share, "config", "diagnostic_aggregator.yaml"]
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
            Node(
                package="rebotarm_monitor",
                executable="joint_state_monitor",
                name="rebotarm_joint_state_monitor",
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
                    },
                ],
            ),
            Node(
                package="diagnostic_aggregator",
                executable="aggregator_node",
                name="diagnostic_aggregator",
                output="screen",
                parameters=[aggregator_config],
                condition=IfCondition(use_diagnostic_aggregator),
            ),
        ]
    )
