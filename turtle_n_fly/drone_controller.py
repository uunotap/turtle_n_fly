import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile,QoSReliabilityPolicy,QoSHistoryPolicy
from geometry_msgs.msg import Twist,PointStamped
from nav_msgs.msg import Odometry
import math
from geometry_msgs.msg import PoseStamped
from sensor_msgs.msg import Imu
from scipy.spatial.transform import Rotation 

class DroneController(Node):
    def __init__(self):
        self.goal_pose = None

        qos_profile = QoSProfile(reliability = QoSReliabilityPolicy.RELIABLE,history=QoSHistoryPolicy.KEEP_LAST,depth=10)

        super().__init__('drone_controller')
        self.get_logger().info("Am i even running or nah?")

        self.cmd_vel_pub = self.create_publisher(Twist, '/cmd_vel', QoSProfile(depth=10))
        self.goal_subscription = self.create_subscription(PoseStamped,'/goal',self.goal_callback,qos_profile)
        self.gps_subscription = self.create_subscription(PointStamped,'/mavic2pro/gps',self.gps_callback,10)
        self.imu_subscription = self.create_subscription(Imu,'/imu',self.imu_callback,10)





        self.timer = self.create_timer(0.1, self.timer_callback)  # 10 Hz

        # Time and state tracking
        self.start_time = self.get_clock().now()
        self.state = 'WAITING'

        # Control parameters
        self.liftoff_duration = 3.0
        self.hover_duration = 10.0
        self.fly_duration = 5.0

        self.target_altitude = 0.1
        self.landing_speed = -0.1
        self.kp = 1.0
        self.current_altitude = 0.0
        self.current_x = 0.0
        self.current_y = 0.0
        self.current_z = 0.0
        self.current_yaw = 0.0


    def goal_callback(self, msg):
        self.get_logger().info("I got called!")
        self.goal_pose = msg
        #self.state = msg.header.frame_id
        self.get_logger().info(f"Received new goal: {msg.pose.position}")

    def gps_callback(self,msg):
        self.current_x=msg.point.x
        self.current_y=msg.point.y
        self.current_z=msg.point.z
    
    def imu_callback(self,msg):
        q = msg.orientation
        quaternion = Rotation.from_quat([q.x, q.y,q.z,q.w])
        roll, pitch, yaw = quaternion.as_euler("xyz", degrees=False)
        self.current_yaw = yaw

    
    def angle_diff(self,a,b):
        diff = a-b
        while diff > math.pi:
            diff -= 2*math.pi
        while diff < -math.pi:
            diff += 2*math.pi
        return diff
    def distance(self,p1,p2,p3,p4):
        return math.sqrt((p1-p3)**2 +(p2-p4)**2)
    



    def timer_callback(self):
        twist = Twist()
        self.angle = 0.0
        

        if self.goal_pose:
            goal_x = self.goal_pose.pose.position.x
            goal_y = self.goal_pose.pose.position.y
            goal_z = self.goal_pose.pose.position.z
            self.dx = goal_x - self.current_x
            self.dy = goal_y -self.current_y
            self.angle = math.atan2(self.dy,self.dx)
            yaw_error = self.angle_diff(self.angle,self.current_yaw)
            kp_yaw = 0.8
            twist.angular.z = kp_yaw * yaw_error

            if abs(yaw_error) < 0.1:
                twist.angular.z = 0.0
                twist.linear.x = 0.2
            if self.distance(self.current_x,self.current_y,goal_x,goal_y)<= 0.1:
                self.get_logger().info(f"We are about to land!")
                twist.linear.x = 0.0
                twist.linear.y = 0.0
                twist.angular.x = 0.0
                twist.angular.y = 0.0
                twist.angular.z = 0.0

                if self.current_z > self.target_altitude:
                    self.get_logger().info("We giving us some descent speed!")
                    twist.linear.z = self.landing_speed
                else:
                    twist.linear.z = 0.0
                    self.get_logger().info("TOUCHDOWN")
                    self.get_logger().info("Shutting down drone node...")
                    self.destroy_node()
                self.get_logger().info(f"My descent speed is {twist.linear.z} and current z: {self.current_z}")


            self.cmd_vel_pub.publish(twist)
            self.get_logger().info(f"THe goal is {goal_x} and {goal_y} and self is {self.current_x} and {self.current_y} The angle between me and goal is: {self.angle}")
            
        
            

            # Compare with current pose from odometry
            # and publish velocity command to /mavic/cmd_vel
        
        



        # if self.state == 'LIFTOFF':
        #     twist.linear.z = 0.3
        #     if elapsed > self.liftoff_duration:
        #         self.state = 'HOVERING'
        #         self.start_time = self.get_clock().now()
        #         self.get_logger().info('State changed to HOVERING')

        # elif self.state == 'HOVERING':
        #     error = self.target_altitude - self.current_altitude
        #     twist.linear.z = self.kp * error
        #     twist.linear.z += 0.02 * math.sin(elapsed * 2.0)  # gentles oscillation

        #     if elapsed > self.hover_duration:
        #         self.state = 'FLYING'
        #         self.start_time = self.get_clock().now()
        #         self.get_logger().info('State changed to FLYING')

        # elif self.state == 'FLYING':
        #     error = self.target_altitude - self.current_altitude
        #     twist.linear.z = self.kp * error
        #     twist.linear.x = 0.2  # constant forward motion

        #     if elapsed > self.fly_duration:
        #         self.state = 'LANDED'
        #         self.start_time = self.get_clock().now()
        #         self.get_logger().info('State changed to LANDING')

        # elif self.state == 'LANDED':
        #     # Descend slowly
        #     twist.linear.z = 0.0
        #     twist.linear.y = 0.0
        #     twist.linear.x = 1.0           
        #     twist.angular.z = 0.0
        #     twist.angular.y = 0.0
        #     twist.angular.x = 0.0

        # self.cmd_vel_pub.publish(twist)

def main(args=None):
    rclpy.init(args=args)
    node = DroneController()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
