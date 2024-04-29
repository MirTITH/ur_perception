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
from launch.event_handlers import OnExecutionComplete
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare

from launch.substitutions import Command, FindExecutable, PathJoinSubstitution

kThisPackageName = "dh_robotics_ag95_description"


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

    # Arguments end

    # Gazebo
    gazebo_world = ""
    launch_entities.append(
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource([FindPackageShare("gazebo_ros"), "/launch", "/gazebo.launch.py"]),
            launch_arguments={"world": gazebo_world}.items(),
        )
    )

    # Robot Description
    xacro_path = os.path.join(get_package_share_directory(kThisPackageName), "urdf", "dh_robotics_ag95_gripper.xacro")
    robot_description_content = Command(
        [
            PathJoinSubstitution([FindExecutable(name="xacro")]),
            " ",
            xacro_path,
        ]
    )

    # Robot State Publisher
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
        arguments=["-entity", "dh", "-topic", "robot_description"],
        output="screen",
    )
    launch_entities.append(
        TimerAction(
            period=2.0,  # Wait for Gazebo to start
            actions=[robot_spawn_action],
        )
    )

    # actions_after_robot_spawn.append(
    #     Node(
    #         package="controller_manager",
    #         executable="spawner",
    #         arguments=["gripper_joint_state_broadcaster", "--controller-manager", "/controller_manager"],
    #     )
    # )

    # actions_after_robot_spawn.append(
    #     Node(
    #         package="controller_manager",
    #         executable="spawner",
    #         arguments=["gripper_controller", "-c", "/controller_manager"],
    #     )
    # )

    # rviz
    rviz_config_file = os.path.join(get_package_share_directory(kThisPackageName), "config", "view_robot.rviz")
    rviz_node = Node(
        package="rviz2",
        executable="rviz2",
        name="rviz2",
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
