#!/usr/bin/env python
import rospy
from geometry_msgs.msg import Twist
from std_msgs.msg import Bool
from world import World
from nav_msgs.msg import Odometry
import tf
from predictor import Predictor
import numpy as np

# TARGET.PY 
# class for the target object 

FREQ = 10  # Hz
SLEEP = 2
PI = np.pi
LINVELOCITY = 0.2
ANGVELOCITY = 0.2

VEL = 0.1


class Target:
	def __init__(self):
		rospy.init_node("target") # feel free to rename
		self.pub = rospy.Publisher("robot_1/cmd_vel", Twist, queue_size=0)
		self.stat_sub = rospy.Subscriber("visible_status", Bool, self.visibility_callback, queue_size=1) # only care about most recent msg
		self.sub = rospy.Subscriber("robot_1/odom", Odometry, self.odom_callback)
		rospy.sleep(SLEEP)

		self.posx = 0 # initalizing target x position at 0
		self.posy = 0 # initializing target y position at 0
		self.linxvel = 0 # initializing linear velocity at 0
		self.angzvel = 0 # initializing angular velocity at 0
		self.angle = 0 # current pose angle

		self.vel_msg = Twist() # creating inital publish message, which is altered in the main

		self.is_lost = 0 #1 if the robot is lost from the target
		self.done = 0 #1 if all the points have been traversed

		#to test predictor
		#self.predictor = Predictor()

		rospy.sleep(2)

	def visibility_callback(self, msg):
		# if lost,
		if msg.data==False:
			self.is_lost=1
		# if not lost
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

	# changing the message to simply rotate the robot
	def curve_right(self, sharp):
		self.vel_msg.angular.z = -ANGVELOCITY
		self.vel_msg.linear.x = LINVELOCITY
		self.linxvel = LINVELOCITY
		self.angzvel = -ANGVELOCITY
		if sharp==1:
			self.vel_msg.angular.z = -ANGVELOCITY*1.25
			self.angzvel = -ANGVELOCITY*1.25

	def curve_left(self, sharp):
		self.vel_msg.angular.z = ANGVELOCITY
		self.vel_msg.linear.x = LINVELOCITY
		self.linxvel = LINVELOCITY
		self.angzvel = ANGVELOCITY
		if sharp==1:
			self.vel_msg.angular.z = ANGVELOCITY*1.25
			self.angzvel = ANGVELOCITY*1.25
 
	def straight(self):
		self.vel_msg.angular.z = 0
		self.vel_msg.linear.x = LINVELOCITY
		self.linxvel = LINVELOCITY
		self.angzvel = 0

	def main(self):
		# setup code
		rate = rospy.Rate(FREQ)
		beginning_time = rospy.get_rostime()

		while self.done==0 and not rospy.is_shutdown():
			# from Josephine's previous commit
			
			while rospy.get_rostime() - beginning_time < rospy.Duration(7) and not rospy.is_shutdown():
				self.curve_left(0)
				self.pub.publish(self.vel_msg)
				print "1"

			beginning_time = rospy.get_rostime()
			while rospy.get_rostime() - beginning_time < rospy.Duration(9) and not rospy.is_shutdown():
				self.straight()
				self.pub.publish(self.vel_msg)
				print "2"

			beginning_time = rospy.get_rostime()
			while rospy.get_rostime() - beginning_time < rospy.Duration(3.75) and not rospy.is_shutdown():
				self.curve_left(0)
				self.pub.publish(self.vel_msg)
				print "3"

			beginning_time = rospy.get_rostime()
			while rospy.get_rostime() - beginning_time < rospy.Duration(8.75) and not rospy.is_shutdown():
				self.curve_right(1)
				self.pub.publish(self.vel_msg)
				print "4"

			beginning_time = rospy.get_rostime()
			while rospy.get_rostime() - beginning_time < rospy.Duration(12.5) and not rospy.is_shutdown():
				self.straight()
				self.pub.publish(self.vel_msg)
				print "5"

			beginning_time = rospy.get_rostime()
			while rospy.get_rostime() - beginning_time < rospy.Duration(8) and not rospy.is_shutdown():
				self.curve_right(0)
				self.pub.publish(self.vel_msg)
				print "6"

			beginning_time = rospy.get_rostime()
			while rospy.get_rostime() - beginning_time < rospy.Duration(12) and not rospy.is_shutdown():
				self.straight()
				self.pub.publish(self.vel_msg)
				print "7"
			
			beginning_time = rospy.get_rostime()
			while rospy.get_rostime() - beginning_time < rospy.Duration(8) and not rospy.is_shutdown():
				self.curve_right(0)
				self.pub.publish(self.vel_msg)
				print "8"

			beginning_time = rospy.get_rostime()
			while rospy.get_rostime() - beginning_time < rospy.Duration(10) and not rospy.is_shutdown():
				self.straight()
				self.pub.publish(self.vel_msg)
				print "9"

			beginning_time = rospy.get_rostime()
			while rospy.get_rostime() - beginning_time < rospy.Duration(10.5) and not rospy.is_shutdown():
				self.curve_left(0)
				self.pub.publish(self.vel_msg)
				print "10"

			beginning_time = rospy.get_rostime()
			while rospy.get_rostime() - beginning_time < rospy.Duration(10) and not rospy.is_shutdown():
				self.straight()
				self.pub.publish(self.vel_msg)
				print "11"

			beginning_time = rospy.get_rostime()
			while rospy.get_rostime() - beginning_time < rospy.Duration(9.75) and not rospy.is_shutdown():
				self.curve_right(0)
				self.pub.publish(self.vel_msg)
				print "12"
			
			beginning_time = rospy.get_rostime()
			while rospy.get_rostime() - beginning_time < rospy.Duration(13) and not rospy.is_shutdown():
				self.straight()
				self.pub.publish(self.vel_msg)
				print "13"

			beginning_time = rospy.get_rostime()
			while rospy.get_rostime() - beginning_time < rospy.Duration(9) and not rospy.is_shutdown():
				self.curve_right(0)
				self.pub.publish(self.vel_msg)
				print "14"

			beginning_time = rospy.get_rostime()
			while rospy.get_rostime() - beginning_time < rospy.Duration(9) and not rospy.is_shutdown():
				self.straight()
				self.pub.publish(self.vel_msg)
				print "15"

			beginning_time = rospy.get_rostime()
			while rospy.get_rostime() - beginning_time < rospy.Duration(6) and not rospy.is_shutdown():
				self.curve_right(0)
				self.pub.publish(self.vel_msg)
				print "16"

			beginning_time = rospy.get_rostime()
			while rospy.get_rostime() - beginning_time < rospy.Duration(4) and not rospy.is_shutdown():
				self.curve_left(0)
				self.pub.publish(self.vel_msg)
				print "17"

			beginning_time = rospy.get_rostime()
			while rospy.get_rostime() - beginning_time < rospy.Duration(4) and not rospy.is_shutdown():
				self.curve_right(1)
				self.pub.publish(self.vel_msg)
				print "18"

			beginning_time = rospy.get_rostime()
			while rospy.get_rostime() - beginning_time < rospy.Duration(1) and not rospy.is_shutdown():
				self.straight()
				self.pub.publish(self.vel_msg)
				print "19"

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
