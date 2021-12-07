"""
arena.launch.py

Launches an arena with the rover and spawns 3 rock obstacles
"""
import os
from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import ThisLaunchFileDir,LaunchConfiguration
from launch_ros.actions import Node
from launch.actions import ExecuteProcess
from ament_index_python.packages import get_package_share_directory

from basic_sim import PACKAGE_NAME

def generate_launch_description():
    use_sim_time = LaunchConfiguration('use_sim_time', default='True')
    world_file_name = 'arena.world'
    pkg_dir = get_package_share_directory(PACKAGE_NAME)

    os.environ["GAZEBO_MODEL_PATH"] = os.path.join(pkg_dir, 'models')

    world = os.path.join(pkg_dir, 'worlds', world_file_name)
    launch_file_dir = os.path.join(pkg_dir, 'launch')

    gazebo = ExecuteProcess(
            cmd=['gazebo', '--verbose', world, #'-s', 'libgazebo_ros_init.so', 
            '-s', 'libgazebo_ros_factory.so'],
            output='screen')

    config = os.path.join(
        get_package_share_directory(PACKAGE_NAME),
        'config',
        'params.yaml'
    )
    spawn_near_rock = Node(
        package=PACKAGE_NAME,
        executable='spawn_rock',
        name='spawn_near_rock',
        parameters=[{
            'entity_name': 'near_rock',
            'x': -1.8,
            'y': -0.5,
            'z': 0.0
        }]
    )
    spawn_mid_rock = Node(
        package=PACKAGE_NAME,
        executable='spawn_rock',
        name='spawn_mid_rock',
        parameters=[{
            'entity_name': 'mid_rock',
            'x': 0.0,
            'y': 0.2,
            'z': 0.0
        }]
    )
    spawn_far_rock = Node(
        package=PACKAGE_NAME,
        executable='spawn_rock',
        name='spawn_far_rock',
        parameters=[{
            'entity_name': 'far_rock',
            'x': 1.5,
            'y': -0.4,
            'z': 0.0
        }]
    )

    return LaunchDescription([
        gazebo,
        spawn_near_rock,
        spawn_mid_rock,
        spawn_far_rock
    ])