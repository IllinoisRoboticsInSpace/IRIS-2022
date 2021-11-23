"""
Launches rviz2 with a config file to view depth data from the rover simulation
"""
# A bunch of software packages that are needed to launch ROS2
import os
from launch import LaunchDescription
from launch_ros.actions import Node
from ament_index_python.packages import get_package_share_directory

from basic_sim import PACKAGE_NAME

def generate_launch_description():
    pkg_dir = get_package_share_directory(PACKAGE_NAME)

    remappings = [('/cmd_vel', '/rover/cmd_vel')]

    teleop_keyboard_node = Node(
        package='teleop_twist_keyboard',
        executable='teleop_twist_keyboard',
        name='teleop',
        output='screen',
        prefix = 'xterm -e',
        remappings=remappings
    )

    return LaunchDescription([
        teleop_keyboard_node,
    ])