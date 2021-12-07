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

    config = os.path.join(pkg_dir, 'config', 'rover_depth_camera_view.rviz')
    rviz2 = Node(
        package='rviz2',
        executable='rviz2',
        name='rviz2',
        output='screen',
        arguments=['-d', config]
    )
    return LaunchDescription([
        rviz2,
    ])