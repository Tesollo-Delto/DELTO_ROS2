from moveit_configs_utils import MoveItConfigsBuilder
from moveit_configs_utils.launches import generate_spawn_controllers_launch


def generate_launch_description():
    moveit_config = MoveItConfigsBuilder("delto_description", package_name="delto_2f_moveit").to_moveit_configs()
    return generate_spawn_controllers_launch(moveit_config)
