
from rosgraph_msgs.msg import Clock
import rclpy
from rclpy.node import Node

class MavicNode(Node):
    def __init__(self):
        super().__init__('mavic_node')
        self.subscription = self.create_subscription(Clock, 'clock', self.__clock_callback, 1)
        self.publisher = self.create_publisher(Clock, 'custom_clock', 1)
        self.clock = Clock()

    def __clock_callback(self, msg):
        self.clock = msg
        self.publisher.publish(self.clock)
        self.get_logget().info("Im being called")

        
        
def main(args=None):
    rclpy.init(args=args)
    node = MavicNode()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()
