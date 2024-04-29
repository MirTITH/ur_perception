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
from ur_perception_scripts import get_robot_description_content

kThisPackageName = "ur_perception_gazebo_sim"


def generate_launch_description():
    launch_entities = []
    actions_after_robot_spawn = []  # Actions to be executed after spawn the robot

    # Arguments start

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

    # Arguments end

    # Gazebo
    gazebo_world = ""
    # gazebo_world = os.path.join(get_package_share_directory(kThisPackageName), "gazebo_world/cafe_earthquake.xml")
    launch_entities.append(
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource([FindPackageShare("gazebo_ros"), "/launch", "/gazebo.launch.py"]),
            launch_arguments={"world": gazebo_world}.items(),
        )
    )

    # Robot State Publisher
    robot_description_content = get_robot_description_content(
        sim_gazebo="true",
        simulation_controllers=PathJoinSubstitution(
            [FindPackageShare(kThisPackageName), "config", "ur_controllers.yaml"],
        ),
    )

    launch_entities.append(
        Node(
            package="robot_state_publisher",
            executable="robot_state_publisher",
            output="both",
            parameters=[
                {
                    "robot_description": robot_description_content,
                    "use_sim_time": use_sim_time,
                }
            ],
        )
    )

    # Spawn robot
    robot_spawn_action = Node(
        package="gazebo_ros",
        executable="spawn_entity.py",
        arguments=["-entity", "hdz", "-topic", "robot_description"],
        output="screen",
    )
    launch_entities.append(
        TimerAction(
            period=2.0,  # Wait for Gazebo to start
            actions=[robot_spawn_action],
        )
    )

    joint_state_broadcaster_spawner = Node(
        package="controller_manager",
        executable="spawner",
        arguments=["joint_state_broadcaster", "--controller-manager", "/controller_manager"],
    )
    actions_after_robot_spawn.append(joint_state_broadcaster_spawner)

    # There may be other controllers of the joints, but this is the initially-started one
    initial_joint_controller = LaunchConfiguration("initial_joint_controller")
    actions_after_robot_spawn.append(
        Node(
            package="controller_manager",
            executable="spawner",
            arguments=[initial_joint_controller, "-c", "/controller_manager"],
            condition=IfCondition(LaunchConfiguration("start_joint_controller")),
        )
    )
    actions_after_robot_spawn.append(
        Node(
            package="controller_manager",
            executable="spawner",
            arguments=[initial_joint_controller, "-c", "/controller_manager", "--stopped"],
            condition=UnlessCondition(LaunchConfiguration("start_joint_controller")),
        )
    )

    # Gripper
    # delayed_actions2.append(
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
    actions_after_robot_spawn.append(rviz_node)

    actions_after_robot_spawn.append(LogInfo(msg="Robot spawned"))

    launch_entities.append(
        RegisterEventHandler(
            OnExecutionComplete(
                target_action=robot_spawn_action,
                on_completion=actions_after_robot_spawn,
            )
        )
    ),

    return LaunchDescription(launch_entities)
