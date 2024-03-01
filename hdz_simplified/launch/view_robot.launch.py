from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch.substitutions import Command, FindExecutable, LaunchConfiguration, PathJoinSubstitution
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare
from launch.launch_description_sources import PythonLaunchDescriptionSource
from ament_index_python.packages import get_package_share_directory
import os
from typing import Tuple, Dict, List

kThisPackageName = "hdz_simplified"


def generate_launch_description():
    launch_entities = []

    # robot_description_content = Command(
    #     [
    #         PathJoinSubstitution([FindExecutable(name="xacro")]),
    #         " ",
    #         PathJoinSubstitution([FindPackageShare(kThisPackageName), "urdf", "hdz_simplified.urdf"]),
    #     ]
    # )

    urdf_file = os.path.join(get_package_share_directory(kThisPackageName), "urdf", "hdz_simplified.urdf")

    with open(urdf_file, "r") as f:
        robot_description_content = f.read()

    launch_entities.append(
        Node(
            package="robot_state_publisher",
            executable="robot_state_publisher",
            output="both",
            parameters=[{"robot_description": robot_description_content}],
        )
    )

    launch_entities.append(
        Node(
            package="joint_state_publisher_gui",
            executable="joint_state_publisher_gui",
        )
    )

    rviz_config_file = PathJoinSubstitution([FindPackageShare(kThisPackageName), "config", "view_robot.rviz"])
    launch_entities.append(
        Node(
            package="rviz2",
            executable="rviz2",
            name="rviz2",
            output="log",
            arguments=["-d", rviz_config_file],
        )
    )

    return LaunchDescription(launch_entities)
