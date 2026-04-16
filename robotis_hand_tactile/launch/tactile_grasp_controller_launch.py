from launch import LaunchDescription
from launch_ros.actions import Node
import yaml
import os
import datetime

def generate_launch_description():
    tactile_grasp_controller_node = Node(
        package="robotis_hand_tactile",
        executable="tactile_grasp_controller_node",
        name="tactile_grasp_controller_node",
        output="screen",
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