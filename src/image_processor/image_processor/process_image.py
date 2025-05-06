import cv2
import numpy as np
from cv_bridge import CvBridge

import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image


class ImageProcessor(Node):
    def __init__(self):
        super().__init__('image_processor')

        # Subscriber to simulator image
        self.image_subscriber = self.create_subscription(Image, 'image_raw', self.image_callback, 10)

        # Publisher for visualization
        self.image_proc_pub = self.create_publisher(Image, 'image_proc', 10)

        self.bridge = CvBridge()

    

    def image_callback(self, msg):
        
        try:
            img = self.bridge.imgmsg_to_cv2(msg, desired_encoding='bgr8')

            # Transform image to HSV
            hsv = cv2.cvtColor(img, cv2.COLOR_BGR2HSV)
            # HSV Values for detecting the sphere
            lower_green = np.array([35, 100, 100])
            upper_green = np.array([85, 255, 255])

            mask = cv2.inRange(hsv, lower_green, upper_green)

            result = cv2.bitwise_and(img, img, mask=mask)

            # Publish mask to topic
            new_msg = self.bridge.cv2_to_imgmsg(result, encoding='rgb8')
            self.image_proc_pub.publish(new_msg)
        
        except Exception as e:
            self.get_logger().info(f"ERROR: Image not proccesed {e}")

def main(args=None):
    rclpy.init(args=args)
    node = ImageProcessor()

    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()