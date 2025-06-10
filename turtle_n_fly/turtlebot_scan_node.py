import rclpy
from rclpy.node import Node
from sensor_msgs.msg import LaserScan
from geometry_msgs.msg import PointStamped

class AreaScanner(Node):
    def __init__(self):
        super().__init__('area_scanner')
        self.subscriber = self.create_subscription(LaserScan, '/scan', self.scan_callback, 10)
        self.publisher = self.create_publisher(PointStamped, '/takeoff_zone', 10)

    def scan_callback(self, msg):
        threshold = 2.0
        angle_min = msg.angle_min
        angle_increment = msg.angle_increment
        for i, distance in enumerate(msg.ranges):
            if distance > threshold:
                angle = angle_min + i * angle_increment
                target = PointStamped()
                target.header.frame_id = 'base_link'
                target.point.x = 1.5
                target.point.y = 0.0
                target.point.z = 0.0
                self.publisher.publish(target)
                self.get_logger().info(f"[TurtleBot] Published takeoff zone at: {target.point}")
                break

def main(args=None):
    rclpy.init(args=args)
    node = AreaScanner()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

