import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile,QoSReliabilityPolicy,QoSHistoryPolicy
from geometry_msgs.msg import Twist,PointStamped
from nav_msgs.msg import Odometry
import math
from geometry_msgs.msg import PoseStamped
from sensor_msgs.msg import Imu
from scipy.spatial.transform import Rotation 

import cv2
import numpy as np
from cv_bridge import CvBridge
from sensor_msgs.msg import Image


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


    #image stuff
        self.bridge= CvBridge()
        cv2.namedWindow("drone_view", cv2.WINDOW_NORMAL)
        self.camfeed = self.create_subscription(Image, "/mavic2pro/camera/image_color", self.image_callback,qos_profile)
        self.turtle_det = False
        self.image_width=400
        self.image_height=240


    def image_callback(self, msg):
        cv_image=self.bridge.imgmsg_to_cv2(msg, desired_encoding='bgra8')
        
        colorshift=cv2.cvtColor(cv_image, cv2.COLOR_BGRA2BGR)
        lower=np.array([5,5,5], dtype="uint8")
        higher=np.array([60,60,65], dtype="uint8")
        turtle_mask=cv2.inRange(colorshift,lower,higher)
        contours, _= cv2.findContours(turtle_mask, cv2.RETR_EXTERNAL,cv2.CHAIN_APPROX_SIMPLE)
        
        dimensions=colorshift.shape
        
        if contours:
            largest=max(contours, key=cv2.contourArea)
            if cv2.contourArea(largest)>50:    
                x, y, w, h= cv2.boundingRect(largest)
                cv2.rectangle(colorshift, (x, y),(x+w,y+h), (0,125,125), 2)
                cx, cy = int(x+w/2),int(y+h/2)
    

                self.turtle_det=True
            
        # Compute offset from center of image
                offset_x = cx - self.image_width // 2
                offset_y = cy - self.image_height // 2
                self.turtle_offset = (offset_x, offset_y)
    
        else:
            self.turtle_det=False

            self.turtle_offset = (0, 0)
    
    
    
        cv2.imshow("drone_view",colorshift)
        cv2.waitKey(1)
    #image stuff


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
                    twist.linear.z = -0.05
                    self.get_logger().info("TOUCHDOWN")
                    self.get_logger().info("Shutting down drone node...")
                    self.destroy_node()
                self.get_logger().info(f"My descent speed is {twist.linear.z} and current z: {self.current_z}")


            self.cmd_vel_pub.publish(twist)
            self.get_logger().info(f"THe goal is {goal_x} and {goal_y} and self is {self.current_x} and {self.current_y} The angle between me and goal is: {self.angle}")
            
        else:
            if self.turtle_det:
                offset_x, offset_y = self.turtle_offset

                # Normalize offset
                norm_x = offset_x / (self.image_width / 2)
                norm_y = offset_y / (self.image_height / 2)

                # Simple proportional control to center the turtle
                kp_cam_yaw = 0.3
                kp_cam_pitch = 0.1

                twist.angular.z = -kp_cam_yaw * norm_x  # turn to center x
                twist.linear.z = -kp_cam_pitch * norm_y  # optional: adjust altitude to center y

                self.get_logger().info(f"Centering turtle: offset_x={offset_x}, offset_y={offset_y}, norm_x={norm_x:.2f}, norm_y={norm_y:.2f}")
            else:
                twist.angular.y = -0.5  # slow hover or scan
                twist.linear.x = 0.0
                twist.linear.z = -self.landing_speed

            self.cmd_vel_pub.publish(twist)
        
        



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
