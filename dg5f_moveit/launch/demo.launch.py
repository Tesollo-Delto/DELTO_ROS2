from moveit_configs_utils import MoveItConfigsBuilder
from moveit_configs_utils.launches import generate_demo_launch


def generate_launch_description():
    moveit_config = MoveItConfigsBuilder("delto_gripper_5f", package_name="dg5f_moveit").to_moveit_configs()
    return generate_demo_launch(moveit_config)