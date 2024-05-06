import os
import yaml
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare


kThisPackageName = "move_grasp"


def generate_launch_description():
    def load_yaml(file_path):
        with open(file_path, "r") as file:
            return yaml.load(file, Loader=yaml.FullLoader)

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

    kinematics_file = os.path.join(
        get_package_share_directory("ur_perception_description"), "config", "kinematics.yaml"
    )
    robot_description_kinematics = load_yaml(kinematics_file)

    launch_entities.append(
        Node(
            package=kThisPackageName,
            executable="move_grasp",
            # output="screen",
            parameters=[
                {"use_sim_time": use_sim_time},
                {"robot_description_kinematics": robot_description_kinematics},
                {"max_velocity_scaling_factor": 1.0},
                {"max_acceleration_scaling_factor": 1.0},
                {"planning_group": "ur_manipulator"},
            ],
        )
    )

    return LaunchDescription(launch_entities)
