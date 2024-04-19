from launch.substitutions import Command, FindExecutable, PathJoinSubstitution
import os
from ament_index_python.packages import get_package_share_directory


def get_robot_description_content(
    xacro_path=os.path.join(get_package_share_directory("ur_perception_description"), "urdf/hdz_with_ur.urdf.xacro"),
    **kwargs,
):
    command_list = [
        PathJoinSubstitution([FindExecutable(name="xacro")]),
        " ",
        xacro_path,
    ]

    for key, value in kwargs.items():
        command_list.extend(
            [
                " ",
                key,
                ":=",
                value,
            ]
        )

    return Command(command_list)
