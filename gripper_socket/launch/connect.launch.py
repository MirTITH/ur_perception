
from launch import LaunchDescription
from launch_ros.actions import Node


kThisPackageName = "gripper_socket"

def generate_launch_description():
    launch_entities = []
    launch_entities.append(
        Node(
            package=kThisPackageName,
            executable="gripper_socket",
            output="both",
            parameters=[
                {
                    "remote_host": "192.168.1.150",
                    "remote_port": "8848",
                }
            ],
        )
    )
    return LaunchDescription(launch_entities)