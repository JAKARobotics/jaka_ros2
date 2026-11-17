from moveit_configs_utils import MoveItConfigsBuilder
from jaka_moveit_launches.launches import generate_rsp_launch


def generate_launch_description():
    moveit_config = MoveItConfigsBuilder("jaka_zu18", package_name="jaka_zu18_moveit_config").to_moveit_configs()
    return generate_rsp_launch(moveit_config)
