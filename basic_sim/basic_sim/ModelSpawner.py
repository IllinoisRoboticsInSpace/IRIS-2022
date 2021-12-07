"""
Script used to spawn a model
"""
import os
import sys
import rclpy
from ament_index_python.packages import get_package_share_directory
from gazebo_msgs.srv import SpawnEntity

from rclpy.node import Node
from . import PACKAGE_NAME

class ModelSpawner(Node):
    """ Node that spawns a model from this package at a given location in a gazebo world
        Entity name is based on the sdf model's name
        ros_parameters:
            'model': string model name
            'x': float x-position
            'y': float y-position
            'z': float z-position
    """
    def __init__(self, model_name: str, entity_name: str, default_x: float, default_y: float, default_z: float):
        super().__init__('model_spawner')
        self.declare_parameters(
            namespace='',
            parameters=[
                ('model_name', model_name),
                ('entity_name', entity_name),
                ('x', default_x),
                ('y', default_y),
                ('z', default_z),
            ]
        )

        (self.model_name, self.entity_name, self.x, self.y, self.z) = self.get_parameters(['model_name', 'entity_name', 'x', 'y', 'z'])
        self.spawn_client = self.create_client(SpawnEntity, "/spawn_entity")
        self.get_logger().info("Connecting to `/spawn_entity` service")
        service_ready = self.spawn_client.wait_for_service(5)
        if not service_ready:
            self.get_logger().error("Timeout while connecting to `spawn_entity` service")
        else:
            self.get_logger().info("...connected!")
    
    def spawn_model(self):
        """ Spawns a model into gazebo using the '/spawn_entity' service
        """
        sdf_file_path = os.path.join(
            get_package_share_directory(PACKAGE_NAME), 
            "models", str(self.model_name.value), "model.sdf")
        request = SpawnEntity.Request()
        request.name = self.entity_name.value
        request.xml = open(sdf_file_path, 'r').read()

        request.initial_pose.position.x = self.x.value
        request.initial_pose.position.y = self.y.value
        request.initial_pose.position.z = self.z.value

        self.get_logger().info("Sending service request to `/spawn_entity`")
        future = self.spawn_client.call_async(request)
        rclpy.spin_until_future_complete(self, future)
        if future.result() is not None:
            self.get_logger().info(f"Response: {future.result()}")
        else:
            self.get_logger().error(f"Exception while calling service: {future.exception()}")

def main():
    rclpy.init()
    argv = sys.argv
    model_name = "rock_small"
    entity_name = "rock_small"
    x, y, z = 0.0, 0.0, 0.0
    if len(argv) > 1 and argv[1] != '--ros-args':
        model_name, entity_name = argv[1:3]
        x, y, z = [float(arg) for arg in argv[3:]]
    node = ModelSpawner(model_name, entity_name, x, y, z)
    node.spawn_model()
    node.destroy_node()
    rclpy.shutdown()

def spawn_rock_main():
    rclpy.init()
    argv = sys.argv
    model_name = "rock_small"
    entity_name = "rock_small"
    x, y, z = 0.0, 0.0, 0.0    
    if len(argv) > 1 and argv[1] != '--ros-args':
        entity_name = argv[1]
        x, y, z = [float(arg) for arg in argv[2:]]
    node = ModelSpawner(model_name, entity_name, x, y, z)
    node.spawn_model()
    node.destroy_node()
    rclpy.shutdown()

if __name__ == "__main__":
    main()

