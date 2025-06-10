#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from std_msgs.msg import Float32MultiArray

class DroneController(Node):
    def __init__(self):
        super().__init__('drone_controller')
        self.subscription = self.create_subscription(
            Float32MultiArray,
            '/takeoff_zone',
            self.zone_callback,
            10
        )
        self.get_logger().info('Drone controller node started')

    def zone_callback(self, msg):
        x, y, z = msg.data
        self.get_logger().info(f'Received takeoff zone at: {x}, {y}, {z}')
        # TODO: Add real drone movement commands here (use Webots API or ROS 2 topics)
        self.get_logger().info('Flying to zone...')

def main(args=None):
    rclpy.init(args=args)
    node = DroneController()
    rclpy.spin(node)
    rclpy.shutdown()

