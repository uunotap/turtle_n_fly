
from rosgraph_msgs.msg import Clock
import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist,PoseStamped
from sensor_msgs.msg import PointCloud2
from sensor_msgs.msg import LaserScan
import sensor_msgs_py.point_cloud2 as pc2
from nav_msgs.msg import Odometry
import math
import numpy as np


class TurtleNode(Node):
	def __init__(self):
		super().__init__('Turtle_node')
		self.clock_subscription = self.create_subscription(Clock, '/clock', self.clock_callback, 1)
		self.publisher = self.create_publisher(Clock, 'custom_clock', 1)
		self.clock = Clock()
		self.cmd_publisher = self.create_publisher(Twist, '/turtlebot3/cmd_vel', 10)
		self.pc_subscription = self.create_subscription(PointCloud2, '/turtlebot3/lidar/point_cloud',self.pc_callback,10)
		self.scan_subscription = self.create_subscription(LaserScan, 'turtlebot3/scan',self.scan_callback,10)
		self.odom_subscription = self.create_subscription(Odometry,'/turtlebot3/odom',self.odom_callback,10)
		self.goal_publisher = self.create_publisher(PoseStamped, '/goal',10)
		
		self.moving_duration = 30
		self.moving = True
		self.timer_period = 0.1
		self.timer = self.create_timer(self.timer_period, self.timer_callback)

		self.max_side = None
		self.turning = False
		self.front_clear_count = 0
		
		self.start_time = self.get_clock().now()
		self.position = None

	def scan_callback(self,msg):

		self.get_logger().info("we logging")

		if self.turning:
			self.get_logger().info("WE here")
			return
		angle_inc = msg.angle_increment
		angle_min = msg.angle_min
		ranges = msg.ranges

		left_index = int((1.5708-angle_min)/angle_inc)
		right_index = int((-1.5708-angle_min)/angle_inc)

		left_index = max (0,min(left_index,len(ranges)-1))
		right_index =max(0,min(right_index,len(ranges)-1))

		left_distance = ranges[left_index]
		right_distance= ranges[right_index]

		if not math.isinf(left_distance) and not math.isinf(right_distance):
			if left_distance > right_distance:
				self.max_side = "left"
			else:
				self.max_side ="right"
		elif math.isinf(left_distance) and not math.isinf(right_distance):
			self.max_side ="right"
		elif math.isinf(right_distance) and not math.isinf(left_distance):
			self.max_side="left"
		else:
			
			self.get_logger().info("I CANT TURNNNN")
			self.max_side = "left"

			

		self.get_logger().info(f'Left (90°): {left_distance:.2f} m, Right (-90°): {right_distance:.2f} m')
		self.get_logger().info(f'The further side is {self.max_side}')


	def clock_callback(self, msg):
		self.clock = msg
		self.publisher.publish(self.clock) 
		
	def timer_callback(self):
		elapsed = (self.get_clock().now() -self.start_time).nanoseconds/1e9
		
		twist = Twist()
		
		if self.moving and elapsed < self.moving_duration:
			twist.linear.x = 0.0
			
			self.cmd_publisher.publish(twist)
	def pc_callback(self,msg):
		self.get_logger().info("We in pc callback")
		twist2 = Twist()
		points = list(pc2.read_points(msg, skip_nans=True))
   
		if not points:
			self.get_logger().info("No points are coming through")
			return
		distances= [(x**2 + y**2 +z**2)**0.5 for x, y, z, *_ in points]
		
		front_points = []
		

		for x,y,z in points:
			distance = math.sqrt(x**2 + y**2)
			angle = math.atan2(y,x)
			if distance < 2.0 and -math.radians(9) <=angle <= math.radians(9):
				front_points.append(distance)
		if front_points:
		
			min_in_front = min(front_points)
			self.get_logger().info(f"Min in front {min_in_front} ")
			if min_in_front > 0.20:
					self.front_clear_count += 1
					print(f"Waiting {self.front_clear_count}")
					if self.front_clear_count > 5:

						self.turning = False
						self.get_logger().info("Forward!")
						twist2.angular.z = 0.0
						twist2.linear.x = 0.2
						self.cmd_publisher.publish(twist2)
					else: 
						self.get_logger().info("Waitign for clear front")
			else:
				self.front_clear_count = 0
				self.turning = True
				twist2.linear.x=0.0
				twist2.angular.z=1.0 if self.max_side == "left" else -1.0
				self.cmd_publisher.publish(twist2)


		else:
			self.get_logger().warn("Nothing visbile in the front")
		
  
			
			 
			
		
		min_dist = min(distances)
		max_dist = max(distances)
		
		
		
		self.get_logger().info(f"Point cloud with {len(points)} points. Minimum distance of: {min_dist:.2f}, Maximum distance of: {max_dist: .2f}") 
		#self.get_logger().info(f"The front points: {front_points}")
		if min_dist > 0.4:
			self.get_logger().info("Found an opening!!!!!!!!!!!!!!!!!!!!!")
			self.get_logger().info(f"My position is ({self.position.x}, {self.position.y}, {self.position.z}!")
			pose_msg = PoseStamped()
			pose_msg.header.stamp = self.get_clock().now().to_msg()
			pose_msg.header.frame_id = 'LIFTOFF'
			pose_msg.pose.position.x = self.position.x
			pose_msg.pose.position.y = self.position.y

			self.goal_publisher.publish(pose_msg)


			self.destroy_node()

	def odom_callback(self,msg):
		self.position = msg.pose.pose.position
	

	
		
			
				 
		
		
def main(args=None):
	rclpy.init(args=args)
	node = TurtleNode()
	rclpy.spin(node)
	node.destroy_node()
	rclpy.shutdown()

