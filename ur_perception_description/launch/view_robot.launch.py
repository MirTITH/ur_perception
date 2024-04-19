from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription, ExecuteProcess
from launch.substitutions import Command, FindExecutable, LaunchConfiguration, PathJoinSubstitution
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare
from launch.launch_description_sources import PythonLaunchDescriptionSource
from ament_index_python.packages import get_package_share_directory
import os
from typing import Tuple, Dict, List
from ur_perception_scripts import get_robot_description_content

kThisPackageName = "ur_perception_description"


def generate_launch_description():
    launch_entities = []

    launch_entities.append(
        DeclareLaunchArgument(
            "use_sim_time",
            description="Use simulation (Gazebo) clock if true",
            choices=["true", "false"],
            default_value="false",
        )
    )
    use_sim_time = LaunchConfiguration("use_sim_time")

    launch_entities.append(
        ExecuteProcess(
            cmd=["echo", " use_sim_time: ", use_sim_time],
            output="both",
            name=__file__,
        )
    )

    launch_entities.append(
        Node(
            package="robot_state_publisher",
            executable="robot_state_publisher",
            output="both",
            parameters=[
                {
                    "robot_description": get_robot_description_content(),
                    "use_sim_time": use_sim_time,
                }
            ],
        )
    )

    launch_entities.append(
        Node(
            package="joint_state_publisher_gui",
            executable="joint_state_publisher_gui",
            parameters=[{"use_sim_time": use_sim_time}],
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
            parameters=[{"use_sim_time": use_sim_time}],
        )
    )

    return LaunchDescription(launch_entities)
