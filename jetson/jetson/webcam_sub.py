import rclpy
from rclpy.node import Node
import cv2
from cv_bridge import CvBridge
from sensor_msgs.msg import Image

class WebcamSubscriber(Node):
    def __init__ (self):
        super().__init__('webcam_subscriber')
        self.subscription = self.create_subscription(Image, 'frames', self.listener_callback, 10)
        self.br = CvBridge()
    def listener_callback(self, data):
        self.get_logger().info('Recieving video frame')
        current_frame = self.br.imgmsg_to_cv2(data)
        cv2.imshow("camera", current_frame)
        cv2.waitKey(1)
        
def main(args=None):
    rclpy.init(args=args)

    contro = WebcamSubscriber()
    try:
        rclpy.spin(contro)
    except KeyboardInterrupt:
        pass
    finally:
        contro.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()