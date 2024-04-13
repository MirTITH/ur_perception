import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch_ros.actions import Node
from launch import LaunchDescription
from launch.actions import (
    DeclareLaunchArgument,
    IncludeLaunchDescription,
    OpaqueFunction,
    RegisterEventHandler,
)
from launch.conditions import IfCondition, UnlessCondition
from launch.event_handlers import OnProcessExit
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import Command, FindExecutable, LaunchConfiguration, PathJoinSubstitution
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare
from launch.actions import ExecuteProcess

kThisPackageName = "ur_perception_gazebo_sim"


def generate_launch_description():
    launch_entities = []

    # Use simulation (Gazebo) clock
    launch_entities.append(
        DeclareLaunchArgument(
            "use_sim_time",
            description="Use simulation (Gazebo) clock if true",
            choices=["true", "false"],
            default_value="true",
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
        DeclareLaunchArgument(
            "initial_joint_controller",
            default_value="joint_trajectory_controller",
            description="Robot controller to start.",
        )
    )
    launch_entities.append(
        DeclareLaunchArgument(
            "start_joint_controller",
            default_value="true",
            description="Enable headless mode for robot control",
        )
    )

    # Robot State Publisher
    # This launch file will set argument 'robot_description_content'
    launch_entities.append(
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(
                [
                    os.path.join(
                        get_package_share_directory("ur_perception_description"),
                        "launch/robot_state_publisher.launch.py",
                    )
                ]
            ),
            launch_arguments={
                "use_sim_time": use_sim_time,
                "sim_gazebo": "true",
                "simulation_controllers": PathJoinSubstitution(
                    [FindPackageShare(kThisPackageName), "config", "ur_controllers.yaml"]
                ),
            }.items(),
        )
    )
    # robot_description_content = LaunchConfiguration("robot_description_content")

    joint_state_broadcaster_spawner = Node(
        package="controller_manager",
        executable="spawner",
        arguments=["joint_state_broadcaster", "--controller-manager", "/controller_manager"],
    )
    launch_entities.append(joint_state_broadcaster_spawner)

    # There may be other controllers of the joints, but this is the initially-started one
    initial_joint_controller = LaunchConfiguration("initial_joint_controller")
    launch_entities.append(
        Node(
            package="controller_manager",
            executable="spawner",
            arguments=[initial_joint_controller, "-c", "/controller_manager"],
            condition=IfCondition(LaunchConfiguration("start_joint_controller")),
        )
    )
    launch_entities.append(
        Node(
            package="controller_manager",
            executable="spawner",
            arguments=[initial_joint_controller, "-c", "/controller_manager", "--stopped"],
            condition=UnlessCondition(LaunchConfiguration("start_joint_controller")),
        )
    )

    # Gazebo
    # gazebo_world = ""
    gazebo_world = os.path.join(get_package_share_directory(kThisPackageName), "gazebo_world/cafe_earthquake.xml")
    launch_entities.append(
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource([FindPackageShare("gazebo_ros"), "/launch", "/gazebo.launch.py"]),
            launch_arguments={"world": gazebo_world}.items(),
        )
    )
    # Spawn robot
    launch_entities.append(
        Node(
            package="gazebo_ros",
            executable="spawn_entity.py",
            arguments=["-entity", "hdz", "-topic", "robot_description"],
            output="screen",
        )
    )

    # Gripper
    # launch_entities.append(
    #     Node(
    #         package="controller_manager",
    #         executable="spawner",
    #         arguments=["gripper_controller", "-c", "/controller_manager"],
    #     )
    # )

    # rviz
    rviz_config_file = os.path.join(get_package_share_directory(kThisPackageName), "rviz", "view_robot_perception.rviz")
    rviz_node = Node(
        package="rviz2",
        executable="rviz2",
        name="rviz2",
        # output="log",
        arguments=["-d", rviz_config_file],
        parameters=[{"use_sim_time": True}],
    )
    launch_entities.append(rviz_node)

    return LaunchDescription(launch_entities)
