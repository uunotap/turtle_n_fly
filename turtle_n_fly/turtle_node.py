
from rosgraph_msgs.msg import Clock
import rclpy
from rclpy.node import Node

class TurtleNode(Node):
    def __init__(self):
        super().__init__('Turtle_node')
        self.subscription = self.create_subscription(Clock, 'clock', self.__clock_callback, 1)
        self.publisher = self.create_publisher(Clock, 'custom_clock', 1)
        self.clock = Clock()

    def __clock_callback(self, msg):
        self.clock = msg
        self.publisher.publish(self.clock)
        
        
        
def main(args=None):
    rclpy.init(args=args)
    node = TurtleNode()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()
