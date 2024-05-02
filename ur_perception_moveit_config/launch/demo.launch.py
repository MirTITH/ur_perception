from moveit_configs_utils import MoveItConfigsBuilder
from moveit_configs_utils.launches import generate_demo_launch


def generate_launch_description():
    moveit_config = MoveItConfigsBuilder("hdz_with_ur", package_name="ur_perception_moveit_config").to_moveit_configs()
    print(moveit_config)
    exit()
    # return generate_demo_launch(moveit_config)
