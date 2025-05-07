import numpy as np

import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist
from std_msgs.msg import Int32

class RobotController(Node):
    
    def __init__(self):
        super().__init__("robot_controller")

        # Publisher to robot topic
        self.cmd_vel_pub = self.create_publisher(Twist, 'cmd_vel', 10)

        # Subscriber to pixel distance from center
        self.pxl_dist_sub = self.create_subscription(Int32, 'pxl_dist', self.pxl_dist_callback, 10)

        # Controller constants
        self.declare_parameter('Kp', 0.005)
        self.declare_parameter('Ki', 100)
        self.declare_parameter('Kd', 0.001)

        self.Kp = self.get_parameter('Kp').value
        self.Ki = self.get_parameter('Ki').value
        self.Kd = self.get_parameter('Kd').value

        # PID Variables
        self.prev_error = 0.0
        self.integral_error = 0.0

        # Object variables
        self.pxl_tolerance = 10

        # Timer variables
        use_sim_time = self.get_parameter('use_sim_time').get_parameter_value().bool_value
        self.get_logger().info(f"use_sim_time: {use_sim_time}")


        self.start_time = self.get_clock().now().nanoseconds * 1e-9
        self.prev_time = self.start_time

    def pxl_dist_callback(self, msg):
        pixel_dist = msg.data
        curr_time = self.get_clock().now().nanoseconds * 1e-9

        # If the offset is within tolerance do not worry about it
        if abs(pixel_dist) < self.pxl_tolerance:
            cmd = Twist()
            cmd.angular.z = 0.0
            cmd.linear.x = 0.25
            self.cmd_vel_pub.publish(cmd)
            self.prev_time = curr_time
            return
        
        
        dt = curr_time - self.prev_time
        
        error = -pixel_dist
        self.integral_error += error * dt
        derivative = (error - self.prev_error) / dt if dt > 0 else 0.0

        w = self.Kp * error + self.Ki * self.integral_error + derivative * self.Kd

        cmd = Twist()
        cmd.angular.z = -w
        cmd.linear.x = 0.25
        self.cmd_vel_pub.publish(cmd)
        self.prev_time = curr_time
    


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