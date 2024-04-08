from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription, ExecuteProcess
from launch.substitutions import Command, FindExecutable, LaunchConfiguration, PathJoinSubstitution
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare
from launch.launch_description_sources import PythonLaunchDescriptionSource
from ament_index_python.packages import get_package_share_directory
import os
from typing import Tuple, Dict, List

kThisPackageName = "ur_perception_description"


def generate_launch_description():
    launch_entities = []

    launch_entities.append(
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(
                [os.path.join(get_package_share_directory(kThisPackageName), "launch/robot_state_publisher.launch.py")]
            )
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
