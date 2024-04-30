import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import (
    IncludeLaunchDescription,
    TimerAction,
    DeclareLaunchArgument,
    ExecuteProcess,
    RegisterEventHandler,
    LogInfo,
)
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.conditions import IfCondition, UnlessCondition
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch.event_handlers import OnExecutionComplete, OnProcessExit, OnProcessIO, OnProcessStart, OnShutdown
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare

kThisPackageName = "move_grasp"


def generate_launch_description():
    launch_entities = []

    launch_entities.append(
        DeclareLaunchArgument(
            "use_sim_time",
            description="Use simulation (Gazebo) clock if true",
            choices=["true", "false"],
            default_value="true",
        )
    )
    use_sim_time = LaunchConfiguration("use_sim_time")

    robot_description_kinematics = PathJoinSubstitution(
        [FindPackageShare("ur_perception_description"), "config", "kinematics.yaml"]
    )

    launch_entities.append(
        Node(
            package=kThisPackageName,
            executable="move_grasp",
            output="both",
            parameters=[
                {
                    "use_sim_time": use_sim_time,
                },
                robot_description_kinematics,
            ],
        )
    )

    return LaunchDescription(launch_entities)
