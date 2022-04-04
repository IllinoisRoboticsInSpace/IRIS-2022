import rclpy
from rclpy.node import Node
import cv2
from cv_bridge import CvBridge
from sensor_msgs.msg import Image

class WebcamPublisher(Node):
    def __init__ (self):
        super().__init__('webcam_publisher')
        self.publisher = self.create_publisher(Image, 'frames', 10)
        timer_period = 0.1
        self.timer = self.create_timer(timer_period, self.timer_callback)
        self.cap = cv2.VideoCapture(0)
        self.br = CvBridge()
    def timer_callback(self):
        ret, frame = self.cap.read()
        if ret == True:
            self.publisher.publish(self.br.cv2_to_imgmsg(frame))
        self.get_logger().info('Publishing video frame')
def main(args=None):
    rclpy.init(args=args)

    contro = WebcamPublisher()
    try:
        rclpy.spin(contro)
    except KeyboardInterrupt:
        pass
    finally:
        contro.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()