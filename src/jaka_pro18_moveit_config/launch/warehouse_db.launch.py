from moveit_configs_utils import MoveItConfigsBuilder
from jaka_moveit_launches.launches import generate_warehouse_db_launch


def generate_launch_description():
    moveit_config = MoveItConfigsBuilder("jaka_pro18", package_name="jaka_pro18_moveit_config").to_moveit_configs()
    return generate_warehouse_db_launch(moveit_config)
