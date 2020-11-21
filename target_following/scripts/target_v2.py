#!/usr/bin/env python
import rospy
from geometry_msgs.msg import Twist
from std_msgs.msg import Bool
from world import World
from nav_msgs.msg import Odometry
import tf
from sensor_msgs.msg import LaserScan # laser
from predictor import Predictor
import numpy as np
import math

# TARGET.PY
# class for the target object, now not using hardcoded trajectories but more of motion models
# using code from PA2 (Archita)

class Target:

	FREQ = 10  # Hz
	SLEEP = 2
	ANGVELOCITY = 0.4

	def __init__(self):
		rospy.init_node("target") # feel free to rename

		self.pub = rospy.Publisher("robot_1/cmd_vel", Twist, queue_size=0)
		self.stat_sub = rospy.Subscriber("visible_status", Bool, self.visibility_callback, queue_size=1) # only care about most recent msg
		self.odomsub = rospy.Subscriber("robot_1/odom", Odometry, self.odom_callback)
		# from pa2
		self.lasersub = self.subscriber = rospy.Subscriber("robot_1/base_scan", LaserScan, self.laserscan_callback)
		rospy.sleep(Target.SLEEP)

		self.posx = 0 # initalizing target x position at 0
		self.posy = 0 # initializing target y position at 0
		self.linxvel = 0 # initializing linear velocity at 0
		self.angzvel = 0 # initializing angular velocity at 0
		self.angle = 0 # current pose angle

		self.distance_front = 0 # current distance in front of the target, based on the laser sensor readings  
		self.distance_frontangle = None # angle at which the minimum distance is

		self.vel_msg = Twist() # creating inital publish message, which is altered in the main

		self.is_lost = 0 #1 if the robot is lost from the target
		self.time_lost = 0

		self.turn_direction = 0 # left if 0, right if 1
		self.state = 0 # if the target is moving according to the model, 1 if the target has an obstacle nearby

		rospy.sleep(2)

	def visibility_callback(self, msg):
		# if lost,
		print msg.data
		if msg.data==False:
			self.is_lost=1
		# if not lost,
		else:
			self.is_lost=0

	# from Archita pa1, modified slightly
	def odom_callback(self, odom_message):
		# getting all of the odom information on the current pose of the robot
		self.posx = odom_message.pose.pose.position.x
		self.posy = odom_message.pose.pose.position.y
		# from https://answers.ros.org/question/11545/plotprint-rpy-from-quaternion/#17106
		(roll, pitch, yaw) = tf.transformations.euler_from_quaternion([odom_message.pose.pose.orientation.x, odom_message.pose.pose.orientation.y, odom_message.pose.pose.orientation.z, odom_message.pose.pose.orientation.w])
		self.angle = yaw

	def laserscan_callback(self, laserscan_msg):
		# getting the reading for the minimum value to the right of the robot
		numincrements = int(2*laserscan_msg.angle_max/laserscan_msg.angle_increment)
		# diving the laser range into 5ths, and using the middle 5th (the 3rd 5th) to find the minimum distance in front of the target
		index = 2*int(numincrements/5)
		good_index = 0
		min_front = 2
		while index < 3*int(numincrements/5):
	  		if laserscan_msg.ranges[index] < min_front:
				min_front = laserscan_msg.ranges[index]
				good_index = index
			index += 1
		self.distance_front = min_front
		# the angle at which the minimum distance is in front
		self.distance_frontangle = laserscan_msg.angle_min + good_index*laserscan_msg.angle_increment
		# minumum distance is to the left, turn right
		if self.distance_frontangle<=0:
			self.turn_direction=1
		else:
			self.turn_direction=0
			

	# similar to obstacle movements
	def move(self):
		if self.state==1:
			#self.angzvel = Target.ANGVELOCITY*(-2*self.turn_direction + 1) # if left, positive, if right, negative velocity
			self.angzvel = Target.ANGVELOCITY
			self.vel_msg.angular.z = self.angzvel
			self.linxvel = 0
			self.vel_msg.linear.x = self.linxvel
			self.pub.publish(self.vel_msg)

		else:
			time = rospy.get_rostime()
			t = time.to_sec()
			# https://docs.python.org/3/library/math.html
			self.linxvel = math.sin(0.1*time.to_sec())*0.25 + 0.3
			self.angzvel = math.cos(0.25*time.to_sec())*0.3
			self.vel_msg.linear.x = self.linxvel 
			self.vel_msg.angular.z = self.angzvel
			self.pub.publish(self.vel_msg)

	def find_state(self):
		# giving a state of 1 if the target is too close to an obstacle
		if self.distance_front < 0.4:
			self.state=1
		else:
			self.state=0
		
	def main(self):
		rate = rospy.Rate(Target.FREQ)
		while not rospy.is_shutdown():
			self.find_state()
			print self.distance_front
			self.move()
		


	# class for the points on grid, copied from pa3 Archita
class Node:
	# initializing basic values of the node
	def __init__(self, x, y, grid, goalx, goaly):
		# self.value = grid.get_cell(x, y)
		self.x = x
		self.y = y
		self.prev = None

	# setting the previous node
	def set_prev(self, other_node):
		self.prev = other_node

if __name__ == "__main__":
	t = Target()
	t.main()

