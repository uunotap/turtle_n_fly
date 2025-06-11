import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist
from nav_msgs.msg import Odometry
import math

class DroneController(Node):
    def __init__(self):
        super().__init__('drone_controller')

        self.cmd_vel_pub = self.create_publisher(Twist, '/mavic2pro/cmd_vel', 10)

        # Subscribe to odometry to get current altitude
        self.odom_sub = self.create_subscription(
            Odometry,
            '/mavic2pro/odom',
            self.odom_callback,
            10)

        self.timer = self.create_timer(0.1, self.timer_callback)

        self.start_time = self.get_clock().now()
        self.liftoff_duration = 3.0  # seconds
        self.hover_duration = 10.0
        self.state = 'LIFTOFF'

        self.target_altitude = 0.2  # meters target hover altitude
        self.current_altitude = 0.0
        self.kp = 1.0

    def odom_callback(self, msg: Odometry):
        # Update current altitude from odometry message
        self.current_altitude = msg.pose.pose.position.z
        self.get_logger().debug(f"Current altitude: {self.current_altitude:.3f}")

    def timer_callback(self):
        elapsed = (self.get_clock().now() - self.start_time).nanoseconds / 1e9

        twist = Twist()

        if self.state == 'LIFTOFF':
            # Send constant upward velocity for liftoff
            twist.linear.z = 0.3

            if elapsed > self.liftoff_duration:
                self.state = 'HOVERING'
                self.start_time = self.get_clock().now()
                self.get_logger().info('State changed to HOVERING')

        elif self.state == 'HOVERING':
            # Proportional controller to maintain target altitude
            altitude_error = self.target_altitude - self.current_altitude
            twist.linear.z = self.kp * altitude_error

            # Small oscillation
            twist.linear.z += 0.05 * math.sin(elapsed * 2.0)

            if elapsed > self.hover_duration:
                self.state = 'FLYING'
                self.start_time = self.get_clock().now()
                self.get_logger().info('State changed to FLYING')

        elif self.state == 'FLYING':
            fly_duration = 5.0
            t = min(elapsed / fly_duration, 1.0)

            twist.linear.x = 0.2 * t
            # Maintain altitude with proportional controller
            altitude_error = self.target_altitude - self.current_altitude
            twist.linear.z = self.kp * altitude_error

            if t >= 1.0:
                self.state = 'LANDED'
                self.get_logger().info('State changed to LANDED')

        elif self.state == 'LANDED':
            # Descend slowly
            twist.linear.z = -0.2

        self.cmd_vel_pub.publish(twist)

        # Optional debug log for commands and altitude
        self.get_logger().debug(
            f"State: {self.state}, Altitude: {self.current_altitude:.3f}, "
            f"cmd_vel: x={twist.linear.x:.2f}, z={twist.linear.z:.2f}")

def main(args=None):
    rclpy.init(args=args)
    drone_controller = DroneController()
    rclpy.spin(drone_controller)
    drone_controller.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
