import os

from ament_index_python.packages import get_package_share_directory

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch_ros.actions import Node
from launch.substitutions import LaunchConfiguration, TextSubstitution


def generate_launch_description():
    # joy_config = LaunchConfiguration('joy_config')
    # realsense = LaunchConfiguration('realsense2_camera')

    # config_filepath = os.path.join(get_package_share_directory('vision'), 
    #     'config', 'xbox.config.yaml')

    return LaunchDescription([
        # DeclareLaunchArgument('realsense', default_value='/dev/input/js0'),
        Node(
            package='realsense2_camera', executable='realsense2_camera_node',
            name='realsense2_camera_node'),
        Node(
            package='vision', executable='listener',
            name='realsense_listener', parameters=[]),
    ])
