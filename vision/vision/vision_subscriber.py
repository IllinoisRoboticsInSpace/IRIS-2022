import rclpy
from rclpy.node import Node
from cv_bridge import CvBridge # Package to convert between ROS and OpenCV Images
import cv2 # OpenCV library
from sensor_msgs.msg import Image, PointCloud2
from vision.point_cloud2 import read_points
import numpy as np
    

class MinimalSubscriber(Node):

    def __init__(self):
        super().__init__('minimal_vision_subscriber')
        self.subscription = self.create_subscription(
           Image,
            'camera/color/image_raw',
            self.listener_callback,
            10)

        self.depth_subscription = self.create_subscription(
            PointCloud2,
            '/camera/depth/color/points',
            self.pointcloud_callback,
            10)
        
        
        # Used to convert between ROS and OpenCV images
        self.br = CvBridge()
        

        # add another for point cloud data
        # bridge to opencv
        # use other code to convert point cloud data to numpy array
        # print out length of array to check

    def pointcloud_callback(self, pc_msg: PointCloud2):
        self.get_logger().info('Receiving pointcloud data')

        pointcloud_array = np.array(list(read_points(pc_msg)))
        self.get_logger().info(f'Pointcloud array: {pointcloud_array[0,:]}')

        # Convert ROS Image message to OpenCV image
        #current_frame = self.br.imgmsg_to_cv2(msg)
        #im_rgb = cv2.cvtColor(current_frame, cv2.COLOR_BGR2RGB)
        # Display image
        #cv2.imshow("camera", im_rgb)
        #cv2.waitKey(1)


    def listener_callback(self, msg):
        self.get_logger().info(f'height: {msg.height}, width: {msg.width}')
        # Display the message on the console
        self.get_logger().info('Receiving frame')
    
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