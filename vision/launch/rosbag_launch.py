import launch
import launch.actions
import launch.substitutions
import launch_ros.actions
from launch_ros.actions import Node


def generate_launch_description():
    return launch.LaunchDescription([
    	launch.actions.ExecuteProcess(
    	 cmd = ['ros2', 'bag', 'play', '/home/irisadmin/Desktop/Workspaces/colcon_ws/src/IRIS-2022/vision/data/pointcloud/pointcloud_0.db3'],
    	 output = 'screen'
    	 ),
        Node(
            package='vision', executable='listener',
            name='realsense_listener', parameters=[]),
    ])
    