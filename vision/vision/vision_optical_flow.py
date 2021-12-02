import rclpy
from rclpy.node import Node
from cv_bridge import CvBridge # Package to convert between ROS and OpenCV Images
import cv2 # OpenCV library
from sensor_msgs.msg import Image
import numpy as np

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
        self.last_frame = None
        self.p0 = None
    def listener_callback(self, msg):
        self.get_logger().info(f'height: {msg.height}, width: {msg.width}')
        # Display the message on the console
        self.get_logger().info('Receiving video frame')
    
        # Convert ROS Image message to OpenCV image
        current_frame = self.br.imgmsg_to_cv2(msg)
        # im_rgb = cv2.cvtColor(current_frame, cv2.COLOR_BGR2RGB)
        # params for ShiTomasi corner detection
        feature_params = dict( maxCorners = 100,
                            qualityLevel = 0.3,
                            minDistance = 7,
                            blockSize = 7 )

        # Parameters for lucas kanade optical flow
        lk_params = dict( winSize  = (15,15),
                        maxLevel = 2,
                        criteria = (cv2.TERM_CRITERIA_EPS | cv2.TERM_CRITERIA_COUNT, 10, 0.03))

        # Create some random colors
        color = np.random.randint(0,255,(100,3))

        # Create a mask image for drawing purposes
        mask = np.zeros_like(current_frame)

        if self.last_frame is None:
            self.last_frame = current_frame
            last_frame_gray = cv2.cvtColor(self.last_frame,cv2.COLOR_BGR2GRAY)
            self.p0 = cv2.goodFeaturesToTrack(last_frame_gray, mask = None, **feature_params)
        else:
            last_frame_gray = cv2.cvtColor(self.last_frame,cv2.COLOR_BGR2GRAY)
            current_frame_gray = cv2.cvtColor(current_frame, cv2.COLOR_BGR2GRAY)
            # Take first frame and find corners in it
            self.p0 = cv2.goodFeaturesToTrack(last_frame_gray, mask = None, **feature_params)

            # calculate optical flow
            p1, st, err = cv2.calcOpticalFlowPyrLK(last_frame_gray, current_frame_gray, self.p0, None, **lk_params)

            # Select good points
            good_new = p1[st==1]
            good_old = self.p0[st==1]

            # draw the tracks
            for i,(new,old) in enumerate(zip(good_new,good_old)):
                a,b = new.ravel()
                c,d = old.ravel()
                mask = cv2.line(mask, (a,b),(c,d), color[i].tolist(), 2)
                current_frame = cv2.circle(current_frame,(a,b),5,color[i].tolist(),-1)
            img = cv2.cvtColor(cv2.add(current_frame,mask), cv2.COLOR_BGR2RGB)

            # Display image
            cv2.imshow("camera", img)
            self.last_frame = current_frame
            self.p0 = good_new.reshape(-1,1,2)
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
