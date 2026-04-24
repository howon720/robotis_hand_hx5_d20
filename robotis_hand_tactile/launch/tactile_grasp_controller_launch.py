from launch import LaunchDescription
from launch_ros.actions import Node
from ament_index_python.packages import get_package_share_directory
import os

def generate_launch_description():
    param_file = os.path.join(
        get_package_share_directory("robotis_hand_tactile"),
        "config",
        "param.yaml",
    )

    tactile_grasp_controller_node = Node(
        package="robotis_hand_tactile",
        executable="tactile_grasp_controller_node",
        name="tactile_grasp_controller_node",
        output="screen",
        parameters=[param_file],
    )

    tactile_force_rviz_node = Node(
        package="robotis_hand_tactile",
        executable="tactile_force_rviz",
        name="tactile_force_rviz",
        output="screen",
    )

    return LaunchDescription([
        tactile_grasp_controller_node,
        tactile_force_rviz_node,
    ])