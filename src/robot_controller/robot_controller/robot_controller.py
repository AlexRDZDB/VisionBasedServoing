import numpy as np

import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist

class RobotController(Node):
    
    def __init__(self):
        super().__init__("robot_controller")

        # Publisher to robot topic
        self.cmd_vel_pub = self.create_publisher(Twist, 'cmd_vel', 10)
        


def main(args=None):
    rclpy.init(args=args)
    node = RobotController()

    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        node.get_logger().info("Shutting off controller")
    finally:
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()