from moveit_configs_utils import MoveItConfigsBuilder
from jaka_moveit_launches.launches import generate_setup_assistant_launch


def generate_launch_description():
    moveit_config = MoveItConfigsBuilder("jaka_minicobo", package_name="jaka_minicobo_moveit_config").to_moveit_configs()
    return generate_setup_assistant_launch(moveit_config)
