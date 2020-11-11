#!/usr/bin/env python
import rospy
from geometry_msgs.msg import Twist
from world import World
from nav_msgs.msg import Odometry
import tf

# TARGET.PY 
# class for the target object 

FREQ = 10 # Hz
SLEEP = 2
pi = 3.14

class Target:
	def __init__(self):
		rospy.init_node("target") # target node 
		self.pub = rospy.Publisher("robot_1/cmd_vel", Twist, queue_size=0)
		self.sub = rospy.Subscriber("robot_1/odom", Odometry, self.odom_callback)
		rospy.sleep(SLEEP)

		self.posx = 0 # initalizing target x position at 0
		self.posy = 0 # initializing target y position at 0
		self.linxvel = 0 # initializing linear velocity at 0 
		self.angzvel = 0 # initializing angular velocity at 0 
		self.angle = 0 # current pose angle
		
		self.vel_msg = Twist() # creating inital publish message, which is altered in the main
		
		# creating a subscriber for the map
		#self.map = None
		#self.subscriber = rospy.Subscriber("map", OccupancyGrid, self.map_callback, queue_size=1)
		
		rospy.sleep(2)
		
	# from Archita pa1, modified slightly
	def odom_callback(self, odom_message):
		# getting all of the odom information on the current pose of the robot
		self.posx = odom_message.pose.pose.position.x
		self.posy = odom_message.pose.pose.position.y
		# from https://answers.ros.org/question/11545/plotprint-rpy-from-quaternion/#17106
		(roll, pitch, yaw) = tf.transformations.euler_from_quaternion([odom_message.pose.pose.orientation.x, odom_message.pose.pose.orientation.y, odom_message.pose.pose.orientation.z, odom_message.pose.pose.orientation.w])
		self.angle = yaw

	# function to rotate
	def rotate(self):
		self.linxvel = 0
		self.angzvel = 0.2

	# move forward
	def forward(self):
		self.linxvel = 0.2
		self.angzvel = 0


	def no_move(self):
		self.linxvel = 0
		self.angzvel = 0

	# move function that dictates next target velocities and locations without knowing map immediately before
	def move(self):
		print self.posx
		print self.posy
		print self.angle

		# needs to move up 2m 
		if -0.1 < self.posx and self.posx < 0.1 and self.posy < 2:
			# at correct angle of 90 to odom, just move up
			if pi/2 - 0.02 < self.angle and self.angle < pi/2 + 0.02:
				self.forward()
			else:
				self.rotate()
	
			return

		# move right 3.5m
		if self.posx < 3.5 and 1.9 < self.posy and self.posy < 2.1:
			# at correct angle of 0, just move up
			if  -0.05 < self.angle and self.angle < 0.02:
				self.forward()	
			else:
				self.rotate()
			return

		# move up 2.5m 
		if 3.4 < self.posx and self.posx < 3.6  and self.posy < 4.5:
			# at correct angle of 90 to odom, just move up
			if pi/2 - 0.02  < self.angle and self.angle < pi/2 + 0.02:
				self.forward()
			else:
				self.rotate()
			return

		# move right 4m
		if self.posx < 7.2 and 4.4 < self.posy and self.posy < 4.6:
			# at correct angle of 0, just move up
			if - 0.02  < self.angle and self.angle < 0.02:
				self.forward()	
			else:
				self.rotate()
			return
	
		# move down 4.5m 
		if 7.1 < self.posx and self.posx < 7.3  and self.posy < 4.6 and self.posy > 0:
			# at correct angle of -90 to odom, just move up
			if -pi/2 - 0.02  < self.angle and self.angle < -pi/2 + 0.02:
				self.forward()
			else:
				self.rotate()
			return
	
		# move right 1m
		if self.posx < 8.3 and -0.2 < self.posy and self.posy < 0.2:
			# at correct angle of 0, just move up
			if - 0.02  < self.angle and self.angle < 0.02:
				self.forward()
			else:
				self.rotate()	
			return

		# move up 8.5 m
		if 8.2 < self.posx and self.posx < 8.4  and self.posy < 8.2:
			# at correct angle of 90 to odom, just move up
			if pi/2 - 0.02  < self.angle and self.angle < pi/2 + 0.02:
				self.forward()
			else:
				self.rotate()	
			return

		# move left 8.5m
		if self.posx > 0 and 8.05 < self.posy and self.posy < 8.35:
			# at correct angle of 180, just move up
			if pi-0.04 < self.angle and self.angle < pi+0.04:
				self.forward()
			else:
				self.rotate()	
			return

		if self.posx < 0 and self.posy and self.posy < 8.35:
			self.no_move()
		
		
	def main(self):

		# setup code
		rate = rospy.Rate(FREQ)

		while not rospy.is_shutdown():
			self.move() # calling the move function each iteration

			# altering the message to publish the linear and angular velocity decided
			self.vel_msg.linear.x = self.linxvel
			self.vel_msg.angular.z = self.angzvel
			self.pub.publish(self.vel_msg)
			rate.sleep()




# class for the points on grid, copied from pa3 Archita
class Node:
	# initializing basic values of the node
	def __init__(self, x, y, grid, goalx, goaly):
		#self.value = grid.get_cell(x, y)
		self.x = x
		self.y = y
		self.prev = None
	
	# setting the previous node
	def set_prev(self, other_node):
		self.prev = other_node

if __name__ == "__main__":
	# we'll probably set up target like this from main.py?
	t = Target()
	t.main()
