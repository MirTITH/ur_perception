from launch import LaunchDescription
from launch.substitutions import Command, PathJoinSubstitution, FindExecutable
from launch_ros.actions import Node
from launch.actions import ExecuteProcess
from ament_index_python.packages import get_package_share_directory
import os

kThisPackageName = "ur_perception_description"


def generate_launch_description():
    launch_entities = []

    robot_description_content = Command(
        [
            PathJoinSubstitution([FindExecutable(name="xacro")]),
            " ",
            os.path.join(get_package_share_directory(kThisPackageName), "urdf/ue5e_perception.urdf.xacro"),
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
