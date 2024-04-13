from launch import LaunchDescription
from launch.substitutions import Command, PathJoinSubstitution, FindExecutable, LaunchConfiguration
from launch_ros.actions import Node
from launch.actions import ExecuteProcess, DeclareLaunchArgument
from ament_index_python.packages import get_package_share_directory
from ur_perception_scripts import get_robot_description_content

kThisPackageName = "ur_perception_description"


def generate_launch_description():
    """Generate launch description with robot state publisher node.
    This will set argument 'robot_description_content' where you can get robot_description_content."""
    launch_entities = []

    DeclareLaunchArgument(
        "use_sim_time",
        description="Use simulation (Gazebo) clock if true",
        choices=["true", "false"],
        default_value="false",
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

    launch_entities.append(
        DeclareLaunchArgument(
            "simulation_controllers",
            description="simulation_controllers in xacro file",
            default_value="",
        )
    )

    robot_description_content = get_robot_description_content(
        sim_gazebo=LaunchConfiguration("sim_gazebo"),
        sim_ignition=LaunchConfiguration("sim_ignition"),
        simulation_controllers=LaunchConfiguration("simulation_controllers"),
    )

    launch_entities.append(
        DeclareLaunchArgument(
            "robot_description_content",
            default_value=robot_description_content,
        )
    )

    launch_entities.append(
        Node(
            package="robot_state_publisher",
            executable="robot_state_publisher",
            output="both",
            parameters=[
                {
                    "robot_description": LaunchConfiguration("robot_description_content"),
                    "use_sim_time": use_sim_time,
                }
            ],
        )
    )

    return LaunchDescription(launch_entities)
