import rclpy
from rclpy.node import Node
from geometry_msgs.msg import PointStamped, PoseStamped

class DroneNavigator(Node):
    def __init__(self):
        super().__init__('drone_navigator')
        self.subscription = self.create_subscription(PointStamped, '/takeoff_zone', self.zone_callback, 10)
        self.pose_publisher = self.create_publisher(PoseStamped, '/drone/cmd_pose', 10)  # Update this topic if needed

    def zone_callback(self, msg):
        self.get_logger().info(f"[Drone] Received takeoff zone at: {msg.point}")
        pose = PoseStamped()
        pose.header.frame_id = 'map'
        pose.pose.position.x = msg.point.x
        pose.pose.position.y = msg.point.y
        pose.pose.position.z = 1.0  # fly 1m above
        pose.pose.orientation.w = 1.0
        self.pose_publisher.publish(pose)
        self.get_logger().info("[Drone] Published pose to fly to takeoff zone.")

def main(args=None):
    rclpy.init(args=args)
    node = DroneNavigator()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

