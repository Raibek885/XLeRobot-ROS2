"""Start the XLeRobot driver and name-based leader/follower teleop."""

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue
from launch_ros.substitutions import FindPackageShare


def generate_launch_description() -> LaunchDescription:
    driver_config = PathJoinSubstitution(
        [FindPackageShare("xlerobot_bringup"), "config", "driver.yaml"]
    )
    teleop_config = PathJoinSubstitution(
        [FindPackageShare("xlerobot_bringup"), "config", "leader_follower.yaml"]
    )
    mock_hardware = LaunchConfiguration("mock_hardware")
    robot_variant = LaunchConfiguration("robot_variant")
    robot_id = LaunchConfiguration("robot_id")
    calibrate_on_connect = LaunchConfiguration("calibrate_on_connect")
    port1 = LaunchConfiguration("port1")
    port2 = LaunchConfiguration("port2")
    arm_p_coefficient = LaunchConfiguration("arm_p_coefficient")

    return LaunchDescription(
        [
            DeclareLaunchArgument("mock_hardware", default_value="false"),
            DeclareLaunchArgument("robot_variant", default_value="xlerobot"),
            DeclareLaunchArgument("robot_id", default_value="xlerobot_ros2"),
            DeclareLaunchArgument("calibrate_on_connect", default_value="false"),
            DeclareLaunchArgument("port1", default_value="/dev/ttyACM0"),
            DeclareLaunchArgument("port2", default_value="/dev/ttyACM1"),
            DeclareLaunchArgument("arm_p_coefficient", default_value="24"),
            Node(
                package="xlerobot_driver",
                executable="xlerobot_driver",
                parameters=[
                    driver_config,
                    {
                        "mock_hardware": ParameterValue(mock_hardware, value_type=bool),
                        "robot_variant": robot_variant,
                        "robot_id": robot_id,
                        "calibrate_on_connect": ParameterValue(
                            calibrate_on_connect, value_type=bool
                        ),
                        "port1": port1,
                        "port2": port2,
                        "arm_p_coefficient": ParameterValue(
                            arm_p_coefficient, value_type=int
                        ),
                    },
                ],
                output="screen",
                emulate_tty=True,
            ),
            Node(
                package="xlerobot_teleop",
                executable="leader_follower",
                parameters=[teleop_config],
                output="screen",
                emulate_tty=True,
            ),
        ]
    )
