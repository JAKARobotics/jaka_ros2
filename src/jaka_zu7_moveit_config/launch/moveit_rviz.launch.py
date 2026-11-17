from moveit_configs_utils import MoveItConfigsBuilder
from jaka_moveit_launches.launches import generate_moveit_rviz_launch


def generate_launch_description():
    moveit_config = MoveItConfigsBuilder("jaka_zu7", package_name="jaka_zu7_moveit_config").to_moveit_configs()
    return generate_moveit_rviz_launch(moveit_config)
