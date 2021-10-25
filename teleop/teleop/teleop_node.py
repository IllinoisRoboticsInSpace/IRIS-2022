import rclpy
from rclpy.node import Node

from geometry_msgs.msg import Twist
from sensor_msgs.msg import Joy


class TeleopNode(Node):

    def __init__(self):
        super().__init__('teleop_node')
        self.subscription = self.create_subscription(
            Joy, '/joy', self.joystick_callback, 1)
        self.get_logger().info(f"Created node {self.get_name()}")

    def joystick_callback(self, msg: Joy):
        self.get_logger().info(str(msg.axes[0]))


def main(args=None):
    rclpy.init(args=args)

    teleop_node = TeleopNode()

    rclpy.spin(teleop_node)

    teleop_node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
