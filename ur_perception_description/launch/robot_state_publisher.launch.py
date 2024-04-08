from launch import LaunchDescription
from launch.substitutions import Command, PathJoinSubstitution, FindExecutable, LaunchConfiguration
from launch_ros.actions import Node
from launch.actions import ExecuteProcess, DeclareLaunchArgument
from ament_index_python.packages import get_package_share_directory
import os

kThisPackageName = "ur_perception_description"


def generate_launch_description():
    launch_entities = []

    launch_entities.append(
        DeclareLaunchArgument(
            "sim_gazebo",
            description="Set argument 'sim_gazebo' in xacro file",
            choices=["true", "false"],
            default_value="false",
        )
    )

    launch_entities.append(
        DeclareLaunchArgument(
            "sim_ignition",
            description="Set argument 'sim_ignition' in xacro file",
            choices=["true", "false"],
            default_value="false",
        )
    )

    robot_description_content = Command(
        [
            PathJoinSubstitution([FindExecutable(name="xacro")]),
            " ",
            os.path.join(get_package_share_directory(kThisPackageName), "urdf/ue5e_perception.urdf.xacro"),
            " ",
            "sim_gazebo:=",
            LaunchConfiguration("sim_gazebo"),
            " ",
            "sim_ignition:=",
            LaunchConfiguration("sim_ignition"),
        ]
    )

    launch_entities.append(
        Node(
            package="robot_state_publisher",
            executable="robot_state_publisher",
            output="both",
            parameters=[{"robot_description": robot_description_content}],
        )
    )

    return LaunchDescription(launch_entities)
