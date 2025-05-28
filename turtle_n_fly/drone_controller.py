#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist
from ros_gz_interfaces.msg import VelocityCmd

class DroneController(Node):
    def __init__(self):
        super().__init__('drone_controller')
        self.publisher = self.create_publisher(
            VelocityCmd,
            '/model/simple_drone/cmd_vel',
            10
        )
        self.subscription = self.create_subscription(
            Twist,
            '/drone/cmd_vel',
            self.listener_callback,
            10
        )

    def listener_callback(self, msg):
        cmd = VelocityCmd()
        cmd.linear = msg.linear
        cmd.angular = msg.angular
        self.publisher.publish(cmd)

def main(args=None):
    rclpy.init(args=args)
    node = DroneController()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()

