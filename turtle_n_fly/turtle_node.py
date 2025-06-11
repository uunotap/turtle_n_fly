
from rosgraph_msgs.msg import Clock
import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist
from sensor_msgs.msg import PointCloud2
import sensor_msgs_py.point_cloud2 as pc2
import math

class TurtleNode(Node):
	def __init__(self):
		super().__init__('Turtle_node')
		self.clock_subscription = self.create_subscription(Clock, '/clock', self.clock_callback, 1)
		self.publisher = self.create_publisher(Clock, 'custom_clock', 1)
		self.clock = Clock()
		self.cmd_publisher = self.create_publisher(Twist, '/turtlebot3/cmd_vel', 10)
		self.pc_subscription = self.create_subscription(PointCloud2, '/turtlebot3/lidar/point_cloud',self.pc_callback,10)
		
		self.moving_duration = 30
		self.moving = True
		self.timer_period = 0.1
		self.timer = self.create_timer(self.timer_period, self.timer_callback)
		
		self.start_time = self.get_clock().now()

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
			if distance < 2.0 and -math.radians(10) <=angle <= math.radians(10):
				front_points.append(distance)
		if front_points:
		
			min_in_front = min(front_points)
			self.get_logger().info(f"Min in front {min_in_front}")
			if min_in_front > 0.3:
					self.get_logger().info("Forward!")
					twist2.angular.z = 0.0
					twist2.linear.x = 0.2
					self.cmd_publisher.publish(twist2)
			else:
				self.get_logger().info("Turning!!")
				twist2.linear.x=0.0
				twist2.angular.z = 1.0
				self.cmd_publisher.publish(twist2)
		else:
			self.get_logger().warn("Nothing visbile in the front")
		
  
			
			 
			
		
		min_dist = min(distances)
		max_dist = max(distances)
		
		
		
		self.get_logger().info(f"Point cloud with {len(points)} points. Minimum distance of: {min_dist:.2f}, Maximum distance of: {max_dist: .2f}") 
		#self.get_logger().info(f"The front points: {front_points}")
		if min_dist > 1.84:
			self.get_logger().info("Found an opening!!!!!!!!!!!!!!!!!!!!!")
			self.get_logger().info("Found an opening!!!!!!!!!!!!!!!!!!!!!")
			self.get_logger().info("Found an opening!!!!!!!!!!!!!!!!!!!!!")
			self.get_logger().info("Found an opening!!!!!!!!!!!!!!!!!!!!!")
			self.get_logger().info("Found an opening!!!!!!!!!!!!!!!!!!!!!")
			self.destroy_node()
		
			
				 
		
		
def main(args=None):
	rclpy.init(args=args)
	node = TurtleNode()
	rclpy.spin(node)
	node.destroy_node()
	rclpy.shutdown()
