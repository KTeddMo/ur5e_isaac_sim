from moveit_configs_utils import MoveItConfigsBuilder
from moveit_configs_utils.launches import generate_moveit_rviz_launch


def generate_launch_description():
    moveit_config = MoveItConfigsBuilder("ur5e_gripper", package_name="moveit2_ur5e").to_moveit_configs()
    return generate_moveit_rviz_launch(moveit_config)
