from moveit_configs_utils import MoveItConfigsBuilder
from jaka_moveit_launches.launches import generate_demo_gazebo_launch


def generate_launch_description():
    moveit_config = MoveItConfigsBuilder("jaka_s7", package_name="jaka_s7_moveit_config").to_moveit_configs()
    return generate_demo_gazebo_launch(moveit_config)