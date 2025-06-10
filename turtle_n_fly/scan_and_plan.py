#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from std_msgs.msg import Float32MultiArray

class ScanAndPlan(Node):
    def __init__(self):
        super().__init__('scan_and_plan')
        self.publisher_ = self.create_publisher(Float32MultiArray, '/takeoff_zone', 10)
        self.timer = self.create_timer(5.0, self.publish_zone)
        self.get_logger().info('TurtleBot3 scan_and_plan node started')

    def publish_zone(self):
        msg = Float32MultiArray()
        msg.data = [1.0, 0.0, 0.0]  # Example position (x, y, z)
        self.publisher_.publish(msg)
        self.get_logger().info(f'Published takeoff zone at: {msg.data}')

def main(args=None):
    rclpy.init(args=args)
    node = ScanAndPlan()
    rclpy.spin(node)
    rclpy.shutdown()

