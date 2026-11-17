from moveit_configs_utils import MoveItConfigsBuilder
from jaka_moveit_launches.launches import generate_move_group_launch


def generate_launch_description():
    moveit_config = MoveItConfigsBuilder("jaka_c12", package_name="jaka_c12_moveit_config").to_moveit_configs()
    return generate_move_group_launch(moveit_config)
