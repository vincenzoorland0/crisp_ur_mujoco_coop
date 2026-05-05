import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.conditions import IfCondition, UnlessCondition
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():
    package_share = get_package_share_directory("vaacum_gripper_description")
    urdf_path = os.path.join(package_share, "vacuum_gripper", "vacuum_gripper.urdf")
    rviz_config_path = os.path.join(package_share, "rviz", "vacuum_gripper.rviz")
    use_gui = LaunchConfiguration("use_gui")

    with open(urdf_path, "r", encoding="utf-8") as urdf_file:
        robot_description = urdf_file.read()

    return LaunchDescription(
        [
            DeclareLaunchArgument(
                "use_gui",
                default_value="true",
                description="Start joint_state_publisher_gui instead of joint_state_publisher.",
            ),
            Node(
                package="robot_state_publisher",
                executable="robot_state_publisher",
                parameters=[{"robot_description": robot_description}],
                output="screen",
            ),
            Node(
                package="joint_state_publisher",
                executable="joint_state_publisher",
                parameters=[{"robot_description": robot_description}],
                condition=UnlessCondition(use_gui),
                output="screen",
            ),
            Node(
                package="joint_state_publisher_gui",
                executable="joint_state_publisher_gui",
                parameters=[{"robot_description": robot_description}],
                condition=IfCondition(use_gui),
                output="screen",
            ),
            Node(
                package="rviz2",
                executable="rviz2",
                arguments=["-d", rviz_config_path],
                output="screen",
            ),
        ]
    )
