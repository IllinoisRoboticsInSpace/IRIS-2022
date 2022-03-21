import rclpy
from rclpy.node import Node
from cv_bridge import CvBridge # Package to convert between ROS and OpenCV Images
import cv2 # OpenCV library
from sensor_msgs.msg import Image, PointCloud2
from vision.point_cloud2 import read_points
import numpy as np
import matplotlib.pyplot as plt
    

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


    def pointcloud_callback(self, pc_msg: PointCloud2):
        self.get_logger().info('Receiving pointcloud data')

        # convert pointcloud data into np array
        pointcloud_array = np.array(list(read_points(pc_msg)))
        self.get_logger().info(f'Pointcloud array: {pointcloud_array[0,:]}')

        # transformation matrix
        t_matrix = np.array([
            [1, 0, 0, -0.059076450765132904],
            [0, 1, 0, -0.0003348544123582542],
            [0, 0, 1, -0.00001691722536634188],
            [0, 0, 0, 1]
        ])

        # projection matrix
        p_matrix = np.array([
            [380.99737548828125, 0, 318.9661865234375], 
            [0, 0, 380.6072082519531],
            [252.41006469726562, 0, 0],
            [0, 1, 0]
        ]).reshape(3,4)
        
        num_pts = pointcloud_array.shape[0] 
        # add row of ones to have correct dimensions to do matrix multiplication
        pts_3d = np.vstack((pointcloud_array.T[:3], np.ones(num_pts)))
        self.get_logger().info(f'3D array: {pts_3d.shape}')

        # transform the 3D points into 2D
        pts_2D = p_matrix @ t_matrix @ pts_3d
        pts_2D[:2,:] /= pts_2D[2,:]

        # get the depths from the 2D array
        depths = pts_2D[2,:]
        self.get_logger().info(f'2D array: {pts_2D.shape}')

        # filter the points to fit within the camera frame
        filter = np.where((pts_2D[0,:] < 640) & (pts_2D[0,:] >= 0)
                    & (pts_2D[1,:] < 480) & (pts_2D[1,:] >=0))[0]
        self.get_logger().info(f'filtered array: {filter.shape}')

        # filter the pixel points
        pixels = pts_2D[:, filter]
        # filter the depths
        depth = depths[filter]
        self.get_logger().info(f'pixel points {pixels.shape}')
        self.get_logger().info(f'depth mean {np.mean(depth)}')

        # create the color map
        color_map = plt.cm.get_cmap('hsv', 256)
        color_map = np.array([color_map(i) for i in range(256)])[:, :3]*255
        depth_map = np.clip(100 / depth, 0, 255).astype(int)
        self.get_logger().info(f'mean color {np.mean(depth_map)}')
        
        # copy the current image
        depth_image = self.current_frame.copy()
        # get the x coordinate pixels
        x_pixels = pixels[0].astype(int)
        # get the y coordinate pixels
        y_pixels = pixels[1].astype(int)

        depth_image[y_pixels, x_pixels] = color_map[depth_map]


        # Convert ROS Image message to OpenCV image
        #current_frame = self.br.imgmsg_to_cv2(msg)
        #im_rgb = cv2.cvtColor(current_frame, cv2.COLOR_BGR2RGB)
        # Display image
        cv2.imshow("camera", depth_image)
        cv2.waitKey(1)


    def listener_callback(self, msg):
        self.get_logger().info(f'height: {msg.height}, width: {msg.width}')
        # Display the message on the console
        self.get_logger().info('Receiving frame')
    
        # Convert ROS Image message to OpenCV image
        self.current_frame = self.br.imgmsg_to_cv2(msg)
        self.current_frame = cv2.cvtColor(self.current_frame, cv2.COLOR_BGR2RGB)
        # Display image
        #cv2.imshow("camera", im_rgb)
        #cv2.waitKey(1)


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