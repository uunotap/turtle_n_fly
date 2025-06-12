import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist
from nav_msgs.msg import Odometry
import math
from geometry_msgs.msg import PoseStamped

class DroneController(Node):
    def __init__(self):
        self.goal_pose = None

        super().__init__('drone_controller')

        self.cmd_vel_pub = self.create_publisher(Twist, '/cmd_vel', 10)
        #self.goal_subscription = self.create_subscription(PoseStamped,'/goal',self.goal_callback)

        self.timer = self.create_timer(0.1, self.timer_callback)  # 10 Hz

        # Time and state tracking
        self.start_time = self.get_clock().now()
        self.state = 'WAITING'

        # Control parameters
        self.liftoff_duration = 3.0
        self.hover_duration = 10.0
        self.fly_duration = 5.0

        self.target_altitude = 0.3
        self.kp = 1.0
        self.current_altitude = 0.0

    def goal_callback(self, msg):
        self.goal_pose = msg
        self.state = msg.header.id
        self.get_logger().info(f"Received new goal: {msg.pose.position}")


    def timer_callback(self):
        elapsed = (self.get_clock().now() - self.start_time).nanoseconds / 1e9
        twist = Twist()

        if self.goal_pose:
            goal_x = self.goal_pose.pose.position.x
            goal_y = self.goal_pose.pose.position.y
            goal_z = self.goal_pose.pose.position.z

            # Compare with current pose from odometry
            # and publish velocity command to /mavic/cmd_vel


        if self.state == 'LIFTOFF':
            twist.linear.z = 0.3
            if elapsed > self.liftoff_duration:
                self.state = 'HOVERING'
                self.start_time = self.get_clock().now()
                self.get_logger().info('State changed to HOVERING')

        elif self.state == 'HOVERING':
            error = self.target_altitude - self.current_altitude
            twist.linear.z = self.kp * error
            twist.linear.z += 0.02 * math.sin(elapsed * 2.0)  # gentle oscillation

            if elapsed > self.hover_duration:
                self.state = 'FLYING'
                self.start_time = self.get_clock().now()
                self.get_logger().info('State changed to FLYING')

        elif self.state == 'FLYING':
            error = self.target_altitude - self.current_altitude
            twist.linear.z = self.kp * error
            twist.linear.x = 0.2  # constant forward motion

            if elapsed > self.fly_duration:
                self.state = 'LANDED'
                self.start_time = self.get_clock().now()
                self.get_logger().info('State changed to LANDING')

        elif self.state == 'LANDED':
            # Descend slowly
            twist.linear.z = 0.0
            twist.linear.y = 0.0
            twist.linear.x = 1.0           
            twist.angular.z = 0.0
            twist.angular.y = 0.0
            twist.angular.x = 0.0

        self.cmd_vel_pub.publish(twist)

def main(args=None):
    rclpy.init(args=args)
    node = DroneController()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
