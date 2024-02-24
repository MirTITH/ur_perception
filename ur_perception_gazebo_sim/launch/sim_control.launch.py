import os

from ament_index_python.packages import get_package_share_directory

from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription, DeclareLaunchArgument
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch_ros.actions import Node

kThisPackageName = "ur_perception_gazebo_sim"


def generate_launch_description():
    launch_entities = []
    launch_entities.append(
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(
                [os.path.join(get_package_share_directory("ur_simulation_gazebo"), "launch/ur_sim_control.launch.py")]
            ),
            launch_arguments={
                "description_package": "ur_perception_description",
                "description_file": "ue5e_perception.urdf.xacro",
                "launch_rviz": "false",
            }.items(),
        )
    )

    # rviz
    rviz_config_file = os.path.join(get_package_share_directory(kThisPackageName), "rviz", "view_robot_rgbd.rviz")
    launch_entities.append(
        Node(
            package="rviz2",
            executable="rviz2",
            name="rviz2",
            output="log",
            arguments=["-d", rviz_config_file],
            parameters=[{"use_sim_time": True}],
        )
    )

    return LaunchDescription(launch_entities)
