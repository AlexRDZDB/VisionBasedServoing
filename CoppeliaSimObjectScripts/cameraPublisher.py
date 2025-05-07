import rclpy
from rclpy.node import Node

from sensor_msgs.msg import Image

class Publisher(Node):
    def __init__(self, activeVisionSensor):
        super().__init__('publisher')
        self.activeVisionSensor = activeVisionSensor
        self.publisher = self.create_publisher(Image, 'image_raw', 10)

    def publish(self):
        # Publish the image of the active vision sensor:
        data, resolution = sim.getVisionSensorImg(self.activeVisionSensor)
        msg = Image()
        msg.header.stamp = self.get_clock().now().to_msg()
        msg.height = resolution[1]
        msg.width = resolution[0]
        msg.encoding = 'rgb8'
        msg.is_bigendian = 1
        msg.step = resolution[0] * 3
        msg.data = data
        self.publisher.publish(msg)

def sysCall_init():
    sim = require('sim')

    self.frontCamera = sim.getObject('/PioneerP3DX/frontCamera')

    rclpy.init()
    self.publisher_node = Publisher(self.frontCamera)

def sysCall_sensing():
    self.publisher_node.publish()
    rclpy.spin_once(self.publisher_node, timeout_sec=0)

def sysCall_cleanup():
    self.publisher_node.destroy_node()
    rclpy.shutdown()