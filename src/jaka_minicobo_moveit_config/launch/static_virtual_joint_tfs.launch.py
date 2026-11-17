from moveit_configs_utils import MoveItConfigsBuilder
from jaka_moveit_launches.launches import generate_static_virtual_joint_tfs_launch


def generate_launch_description():
    moveit_config = MoveItConfigsBuilder("jaka_minicobo", package_name="jaka_minicobo_moveit_config").to_moveit_configs()
    return generate_static_virtual_joint_tfs_launch(moveit_config)
