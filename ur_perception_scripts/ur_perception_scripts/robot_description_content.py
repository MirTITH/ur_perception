from launch.substitutions import Command, FindExecutable, PathJoinSubstitution
import os
from ament_index_python.packages import get_package_share_directory


def get_robot_description_content(
    xacro_path=os.path.join(
        get_package_share_directory("ur_perception_description"), "urdf/hdz_with_ur.urdf.xacro"
    ),
    sim_gazebo=None,
    sim_ignition=None,
    simulation_controllers=None,
):
    command_list = [
        PathJoinSubstitution([FindExecutable(name="xacro")]),
        " ",
        xacro_path,
    ]

    if sim_gazebo != None:
        command_list.extend(
            [
                " ",
                "sim_gazebo:=",
                sim_gazebo,
            ]
        )

    if sim_ignition != None:
        command_list.extend(
            [
                " ",
                "sim_ignition:=",
                sim_ignition,
            ]
        )

    if simulation_controllers != None:
        command_list.extend(
            [
                " ",
                "simulation_controllers:=",
                simulation_controllers,
            ]
        )

    return Command(command_list)
