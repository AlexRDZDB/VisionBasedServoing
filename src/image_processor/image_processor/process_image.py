import cv2
import numpy as np
from cv_bridge import CvBridge

import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image
from std_msgs.msg import Int32

class ImageProcessor(Node):
    def __init__(self):
        super().__init__('image_processor')

        # Subscriber to simulator image
        self.image_subscriber = self.create_subscription(Image, 'image_raw', self.image_callback, 10)

        # Publisher for visualization
        self.image_proc_pub = self.create_publisher(Image, 'image_proc', 10)

        # Publisher for pixel distance from x center to blob
        self.pixel_distance_pub = self.create_publisher(Int32, 'pxl_dist', 10)

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

            # Find contours
            contours, _ = cv2.findContours(mask, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)

            if contours:
                # Find the largest contour (likely the sphere)
                largest_contour = max(contours, key=cv2.contourArea)
                ((x, y), radius) = cv2.minEnclosingCircle(largest_contour)

                if radius > 5:  # Filter small noise
                    center = (int(x), int(y))
                    image_center = (img.shape[1] // 2, img.shape[0] // 2)
                    offset_x = int(x - image_center[0])
                    offset_y = int(y - image_center[1])

                    # Draw circle and center
                    cv2.circle(result, center, int(radius), (0, 255, 0), 2)
                    cv2.circle(result, image_center, 5, (0, 0, 255), -1)
                    cv2.line(result, image_center, center, (255, 0, 0), 2)

                    self.get_logger().info(f"Offset from center: x={offset_x}, y={offset_y}")
                    pixel_dist = Int32()
                    pixel_dist.data = offset_x

                    self.pixel_distance_pub.publish(pixel_dist)

            # Publish the processed image
            new_msg = self.bridge.cv2_to_imgmsg(result, encoding='bgr8')
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