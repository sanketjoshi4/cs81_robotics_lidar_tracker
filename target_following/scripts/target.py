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

class Target:

	FREQ = 10  # Hz
	SLEEP = 2
	PI = np.pi
	LINVELOCITY = 0.1
	ANGVELOCITY = 0.1

	def __init__(self):
		rospy.init_node("target") # feel free to rename
		self.pub = rospy.Publisher("robot_1/cmd_vel", Twist, queue_size=0)
		self.stat_sub = rospy.Subscriber("visible_status", Bool, self.visibility_callback, queue_size=1) # only care about most recent msg
		self.sub = rospy.Subscriber("robot_1/odom", Odometry, self.odom_callback)
		rospy.sleep(Target.SLEEP)

		self.posx = 0 # initalizing target x position at 0
		self.posy = 0 # initializing target y position at 0
		self.linxvel = 0 # initializing linear velocity at 0
		self.angzvel = 0 # initializing angular velocity at 0
		self.angle = 0 # current pose angle

		self.vel_msg = Twist() # creating inital publish message, which is altered in the main

		self.is_lost = 0 #1 if the robot is lost from the target
		self.done = 0 #1 if all the points have been traversed
		self.time_lost = 0

		#to test predictor
		#self.predictor = Predictor()

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

	# changing the message to simply rotate the robot
	def curve_right(self, sharp):
		self.vel_msg.angular.z = -Target.ANGVELOCITY
		self.vel_msg.linear.x = Target.LINVELOCITY
		self.linxvel = Target.LINVELOCITY
		self.angzvel = -Target.ANGVELOCITY
		if sharp==1:
			self.vel_msg.angular.z = -Target.ANGVELOCITY*1.25
			self.angzvel = -Target.ANGVELOCITY*1.25

	def curve_left(self, sharp):
		self.vel_msg.angular.z = Target.ANGVELOCITY
		self.vel_msg.linear.x = Target.LINVELOCITY
		self.linxvel = Target.LINVELOCITY
		self.angzvel = Target.ANGVELOCITY
		if sharp==1:
			self.vel_msg.angular.z = Target.ANGVELOCITY*1.25
			self.angzvel = Target.ANGVELOCITY*1.25
 
	def straight(self):
		self.vel_msg.angular.z = 0
		self.vel_msg.linear.x = Target.LINVELOCITY
		self.linxvel = Target.LINVELOCITY
		self.angzvel = 0

	def main(self):
		# setup code
		rate = rospy.Rate(Target.FREQ)
		beginning_time = rospy.get_rostime()
	
		while self.done==0 and not rospy.is_shutdown():

			if rospy.get_rostime() - beginning_time - rospy.Duration(self.time_lost) < rospy.Duration(7):
				self.curve_left(0)
				self.pub.publish(self.vel_msg)
				print "1"

			if rospy.Duration(7*2) <= rospy.get_rostime() - beginning_time - rospy.Duration(self.time_lost) and rospy.get_rostime() - beginning_time < rospy.Duration(16*2) - rospy.Duration(self.time_lost):
				self.straight()
				self.pub.publish(self.vel_msg)
				print "2"

			if rospy.Duration(16*2) <= rospy.get_rostime() - beginning_time - rospy.Duration(self.time_lost) and rospy.get_rostime() - beginning_time < rospy.Duration(19.75*2) - rospy.Duration(self.time_lost):
				self.curve_left(0)
				self.pub.publish(self.vel_msg)
				print "3"

			if rospy.Duration(19.75*2) <= rospy.get_rostime() - beginning_time - rospy.Duration(self.time_lost) and rospy.get_rostime() - beginning_time < rospy.Duration(28.5*2) - rospy.Duration(self.time_lost):
				self.curve_right(1)
				self.pub.publish(self.vel_msg)
				print "4"

			if rospy.Duration(28.5*2) <= rospy.get_rostime() - beginning_time - rospy.Duration(self.time_lost) and rospy.get_rostime() - beginning_time < rospy.Duration(41*2) - rospy.Duration(self.time_lost):
				self.straight()
				self.pub.publish(self.vel_msg)
				print "5"

			if rospy.Duration(41*2) <= rospy.get_rostime() - beginning_time - rospy.Duration(self.time_lost) and rospy.get_rostime() - beginning_time < rospy.Duration(49*2) - rospy.Duration(self.time_lost):
				self.curve_right(0)
				self.pub.publish(self.vel_msg)
				print "6"

			if rospy.Duration(49*2) <= rospy.get_rostime() - beginning_time - rospy.Duration(self.time_lost) and rospy.get_rostime() - beginning_time < rospy.Duration(61*2) - rospy.Duration(self.time_lost):
				self.straight()
				self.pub.publish(self.vel_msg)
				print "7"

			if rospy.Duration(61*2) <= rospy.get_rostime() - beginning_time - rospy.Duration(self.time_lost) and rospy.get_rostime() - beginning_time < rospy.Duration(69*2) - rospy.Duration(self.time_lost):
				self.curve_right(0)
				self.pub.publish(self.vel_msg)
				print "8"

			if rospy.Duration(69*2) <= rospy.get_rostime() - beginning_time - rospy.Duration(self.time_lost) and rospy.get_rostime() - beginning_time < rospy.Duration(79*2) - rospy.Duration(self.time_lost):
				self.straight()
				self.pub.publish(self.vel_msg)
				print "9"

			if rospy.Duration(79*2) <= rospy.get_rostime() - beginning_time - rospy.Duration(self.time_lost) and rospy.get_rostime() - beginning_time < rospy.Duration(89.5*2) - rospy.Duration(self.time_lost):
				self.curve_left(0)
				self.pub.publish(self.vel_msg)
				print "10"
		
			if rospy.Duration(89.5*2) <= rospy.get_rostime() - beginning_time - rospy.Duration(self.time_lost) and rospy.get_rostime() - beginning_time < rospy.Duration(99.5*2) - rospy.Duration(self.time_lost):
				self.straight()
				self.pub.publish(self.vel_msg)
				print "11"

			if rospy.Duration(99.5*2) <= rospy.get_rostime() - beginning_time - rospy.Duration(self.time_lost) and rospy.get_rostime() - beginning_time < rospy.Duration(109.25*2) - rospy.Duration(self.time_lost):
				self.curve_right(0)
				self.pub.publish(self.vel_msg)
				print "12"

			if rospy.Duration(109.25*2) <= rospy.get_rostime() - beginning_time - rospy.Duration(self.time_lost) and rospy.get_rostime() - beginning_time < rospy.Duration(122.25*2) - rospy.Duration(self.time_lost):
				self.straight()
				self.pub.publish(self.vel_msg)
				print "13"

			if rospy.Duration(122.25*2) <= rospy.get_rostime() - beginning_time - rospy.Duration(self.time_lost) and rospy.get_rostime() - beginning_time < rospy.Duration(131.25*2) - rospy.Duration(self.time_lost):
				self.curve_right(0)
				self.pub.publish(self.vel_msg)
				print "14"

			if rospy.Duration(131.25*2) <= rospy.get_rostime() - beginning_time - rospy.Duration(self.time_lost) and rospy.get_rostime() - beginning_time < rospy.Duration(140.25*2) - rospy.Duration(self.time_lost):
				self.straight()
				self.pub.publish(self.vel_msg)
				print "15"

			if rospy.Duration(140.25*2) <= rospy.get_rostime() - beginning_time - rospy.Duration(self.time_lost) and rospy.get_rostime() - beginning_time < rospy.Duration(146.25*2) - rospy.Duration(self.time_lost):
				self.curve_right(0)
				self.pub.publish(self.vel_msg)
				print "16"

			if rospy.Duration(146.25*2) <= rospy.get_rostime() - beginning_time - rospy.Duration(self.time_lost) and rospy.get_rostime() - beginning_time < rospy.Duration(150.25*2) - rospy.Duration(self.time_lost):
				self.curve_left(0)
				self.pub.publish(self.vel_msg)
				print "17"

			if rospy.Duration(150.25*2) <= rospy.get_rostime() - beginning_time - rospy.Duration(self.time_lost) and rospy.get_rostime() - beginning_time < rospy.Duration(154.25*2) - rospy.Duration(self.time_lost):
				self.curve_right(1)
				self.pub.publish(self.vel_msg)
				print "18"

			if rospy.Duration(154.25*2) <= rospy.get_rostime() - beginning_time - rospy.Duration(self.time_lost) and rospy.get_rostime() - beginning_time < rospy.Duration(155.25*2) - rospy.Duration(self.time_lost):
				self.straight()
				self.pub.publish(self.vel_msg)
				print "19"

			# if self.is_lost==1:
			# 	current_time = rospy.get_rostime()
			# 	#current_angle = self.angle
			# 	#print current_time.to_sec()
			# while self.is_lost==1 and not rospy.is_shutdown():
			# 	self.vel_msg.angular.z = 0 #Target.ANGVELOCITY*1.5
			# 	self.vel_msg.linear.x = 0
			# 	self.pub.publish(self.vel_msg)
			# 	if self.is_lost==0:
			# 		new_time = rospy.get_rostime()
			# 		print new_time.to_sec()
			# 		self.time_lost += current_time.to_sec() - new_time.to_sec()
			# 		# print self.time_lost
			# 		#while self.angle - current_angle > 0.2 or self.angle - current_angle < -0.2:
			# 			#self.vel_msg.angular.z = Target.ANGVELOCITY*1.5
			# 			#self.vel_msg.linear.x = 0
			# 			#self.pub.publish(self.vel_msg)
			# 		break

	def move_fwd(self):
		curr_time = rospy.get_rostime()
		while not rospy.is_shutdown():
			if rospy.get_rostime() - curr_time > rospy.Duration(1):
				self.vel_msg = Twist()
				self.vel_msg.linear.x = 0.1
				self.pub.publish(self.vel_msg)



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

