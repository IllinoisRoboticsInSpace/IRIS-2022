import rclpy
from rclpy.node import Node
from cv_bridge import CvBridge # Package to convert between ROS and OpenCV Images
import cv2 # OpenCV library
from sensor_msgs.msg import Image
    

class MinimalSubscriber(Node):

    def __init__(self):
        super().__init__('minimal_vision_subscriber')
        self.subscription = self.create_subscription(
            Image,
            '/color/image_raw',
            self.listener_callback,
            10)
        self.subscription  # prevent unused variable warning
        # Used to convert between ROS and OpenCV images
        self.br = CvBridge()
        
    def listener_callback(self, msg):
        self.get_logger().info(f'height: {msg.height}, width: {msg.width}')
        # Display the message on the console
        self.get_logger().info('Receiving video frame')
    
        # Convert ROS Image message to OpenCV image
        current_frame = self.br.imgmsg_to_cv2(msg)
        im_rgb = cv2.cvtColor(current_frame, cv2.COLOR_BGR2RGB)
        # Display image
        cv2.imshow("camera", im_rgb)
        
        cv2.waitKey(1)
def main(args=None):
    rclpy.init(args=args)

    minimal_subscriber = MinimalSubscriber()

    rclpy.spin(minimal_subscriber)

    # Destroy the node explicitly
    # (optional - otherwise it will be done automatically
    # when the garbage collector destroys the node object)
    minimal_subscriber.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()