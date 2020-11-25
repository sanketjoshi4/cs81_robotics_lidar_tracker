#!/usr/bin/env python
import rospy
from geometry_msgs.msg import Twist
from std_msgs.msg import Bool
from world import World
from nav_msgs.msg import Odometry
from sensor_msgs.msg import LaserScan # laser
import tf
from predictor import Predictor
import numpy as np
import math
import random

# TARGET.PY
# class for the target object, using code from PA2 Archita

class Target:

	FREQ = 10  # Hz
	SLEEP = 2
	PI = np.pi
	LINVELOCITY = 0.2 # mode 1 and 3
	ANGVELOCITY = 0.2 # mode 1
	DUR_MUL = 1

	TANGVELOCITY = 0.4 # mode Two ang velocity

	def __init__(self):

		rospy.init_node("target") # feel free to rename
		mode = input("Enter a mode for the target movement by inputting an interger value, where 1 = hardcoded, 2 = sinusoidal, and 3 = randomized: \n")

		self.mode = mode

		self.pub = rospy.Publisher("robot_1/cmd_vel", Twist, queue_size=0)
		self.stat_sub = rospy.Subscriber("visible_status", Bool, self.visibility_callback, queue_size=1) # only care about most recent msg
		self.sub = rospy.Subscriber("robot_1/odom", Odometry, self.odom_callback)
		self.lasersub = rospy.Subscriber("robot_1/base_scan", LaserScan, self.laserscan_callback)
		rospy.sleep(Target.SLEEP)
	
		self.posx = 0 # initalizing target x position at 0
		self.posy = 0 # initializing target y position at 0
		self.linxvel = 0 # initializing linear velocity at 0
		self.angzvel = 0 # initializing angular velocity at 0
		self.angle = 0 # current pose angle

		# used for modes 2 and 3
		self.distance_front = None # current distance in front of the target, based on the laser sensor readings  
		self.distance_frontangle = None # angle at which the minimum distance is
		self.turn_direction = 0 # left if 0, right if 1
		self.state = 0 # if the target is moving according to the model, 1 if the target has an obstacle nearby

		self.vel_msg = Twist() # creating inital publish message, which is altered in the main

		self.is_lost = 0 #1 if the robot is lost from the target
		self.prev_state = 0
		self.start_loss_time = 0 # the start time of the loss
		self.done = 0 #1 if all the points have been traversed
		self.time_lost = 0

		# for mode 3
		self.prev_accel = 0 
		self.prev_angvel = 0

		rospy.sleep(Target.SLEEP)
		#rospy.sleep(2)

	def visibility_callback(self, msg):
		# if lost,
		if msg.data==False:
			# setting the previous state to be whatever it was before
			self.prev_state = self.is_lost
			self.is_lost=1
		# if not lost,
		else:
			self.prev_state = self.is_lost
			self.is_lost=0

	# from Archita pa1, modified slightly
	def odom_callback(self, odom_message):
		# getting all of the odom information on the current pose of the robot
		self.posx = odom_message.pose.pose.position.x
		self.posy = odom_message.pose.pose.position.y
		# from https://answers.ros.org/question/11545/plotprint-rpy-from-quaternion/#17106
		(roll, pitch, yaw) = tf.transformations.euler_from_quaternion([odom_message.pose.pose.orientation.x, odom_message.pose.pose.orientation.y, odom_message.pose.pose.orientation.z, odom_message.pose.pose.orientation.w])
		self.angle = yaw

	###################### USED FOR MODE 1 ######################

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

	###################### USED FOR MODE 2 ######################

	def laserscan_callback(self, laserscan_msg):
		# getting the reading for the minimum value to the right of the robot
		numincrements = int(2*laserscan_msg.angle_max/laserscan_msg.angle_increment)
		# diving the laser range into 3rds, and using the middle 3rd (the 2rd 3rd) to find the minimum distance in front of the target
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
	def sin_move(self):
		if self.state==1:
			#self.angzvel = Target.TANGVELOCITY*(-2*self.turn_direction + 1) # if left, positive, if right, negative velocity
			self.angzvel = Target.TANGVELOCITY
			self.vel_msg.angular.z = self.angzvel
			self.linxvel = 0
			self.vel_msg.linear.x = self.linxvel
			self.pub.publish(self.vel_msg)

		else:
			time = rospy.get_rostime()
			t = time.to_sec()
			# https://docs.python.org/3/library/math.html
			self.linxvel = math.sin(0.1*time.to_sec())*0.1 + 0.3 # 0.2 to 0.4 velocity
			self.angzvel = math.cos(0.25*time.to_sec())*0.3
			self.vel_msg.linear.x = self.linxvel 
			self.vel_msg.angular.z = self.angzvel
			self.pub.publish(self.vel_msg)

	def find_state(self):
		# giving a state of 1 if the target is too close to an obstacle or if robot lost target
		if self.distance_front < 0.4 or self.is_lost==1:
			self.state=1
		else:
			self.state=0

	###################### USED FOR MODE 3 ######################

	def rand_move(self):
		if self.state==1:
			# stop moving linearly, if once again the robot is lost or the target is close to an obstacle
			self.vel_msg.linear.x = 0
			self.vel_msg.angular.z = Target.TANGVELOCITY
			self.pub.publish(self.vel_msg)

		else:
			# randomizing the third degree
			jerk = random.uniform(-1, 1)
			acc = self.prev_accel + jerk
			# if above 5, making it below 5
			if acc > 5 or acc < -5:
				acc = 1/acc
			vel = self.prev_angvel + acc
			# if above one, making it below 1
			if vel > 1 or vel < -1:
				vel = 1/vel
			self.vel_msg.linear.x = Target.LINVELOCITY
			self.vel_msg.angular.z = vel
			self.pub.publish(self.vel_msg)
			self.prev_accel = acc
			self.prev_angvel = vel

	def main(self):
		# setup code
		rate = rospy.Rate(Target.FREQ)
		beginning_time = rospy.get_rostime()

		while self.done==0 and not rospy.is_shutdown():

			# if in the hardcoded mode
			if self.mode==1:
			
				t = rospy.get_rostime().to_sec()
				#print t
				#print self.time_lost

				# if the target is lost, get the time it was first lost and also stop moving 
				#if self.is_lost==1:
					#if self.prev_state==0:
						#time = rospy.get_rostime()
						#self.start_loss_time = time.to_sec()
					#print "is lost"
					#self.vel_msg.angular.z = 0 #Target.ANGVELOCITY*1.5
					#self.vel_msg.linear.x = 0
					#self.pub.publish(self.vel_msg)
					#continue
				#if self.is_lost==0 and self.prev_state==1: # if previously lost but now found, subtract current time, add to 
					#print "found"
					#new_time = rospy.get_rostime()
					#print new_time.to_sec()
					#print self.start_loss_time
					#self.time_lost += new_time.to_sec() - self.start_loss_time
					#continue

				if rospy.get_rostime() - beginning_time - rospy.Duration(self.time_lost) < rospy.Duration(7):
					self.curve_left(0)
					self.pub.publish(self.vel_msg)
					#print "1"

				if rospy.Duration(7*Target.DUR_MUL) <= rospy.get_rostime() - beginning_time - rospy.Duration(self.time_lost) and rospy.get_rostime() - beginning_time < rospy.Duration(16 * Target.DUR_MUL) - rospy.Duration(self.time_lost):
					self.straight()
					self.pub.publish(self.vel_msg)
					#print "2"

				if rospy.Duration(16 * Target.DUR_MUL) <= rospy.get_rostime() - beginning_time - rospy.Duration(self.time_lost) and rospy.get_rostime() - beginning_time < rospy.Duration(19.75 * Target.DUR_MUL) - rospy.Duration(self.time_lost):
					self.curve_left(0)
					self.pub.publish(self.vel_msg)
					#print "3"

				if rospy.Duration(19.75 * Target.DUR_MUL) <= rospy.get_rostime() - beginning_time - rospy.Duration(self.time_lost) and rospy.get_rostime() - beginning_time < rospy.Duration(28.5 * Target.DUR_MUL) - rospy.Duration(self.time_lost):
					self.curve_right(1)
					self.pub.publish(self.vel_msg)
					#print "4"

				if rospy.Duration(28.5 * Target.DUR_MUL) <= rospy.get_rostime() - beginning_time - rospy.Duration(self.time_lost) and rospy.get_rostime() - beginning_time < rospy.Duration(41 * Target.DUR_MUL) - rospy.Duration(self.time_lost):
					self.straight()
					self.pub.publish(self.vel_msg)
					#print "5"

				if rospy.Duration(41 * Target.DUR_MUL) <= rospy.get_rostime() - beginning_time - rospy.Duration(self.time_lost) and rospy.get_rostime() - beginning_time < rospy.Duration(49 * Target.DUR_MUL) - rospy.Duration(self.time_lost):
					self.curve_right(0)
					self.pub.publish(self.vel_msg)
					#print "6"

				if rospy.Duration(49 * Target.DUR_MUL) <= rospy.get_rostime() - beginning_time - rospy.Duration(self.time_lost) and rospy.get_rostime() - beginning_time < rospy.Duration(61 * Target.DUR_MUL) - rospy.Duration(self.time_lost):
					self.straight()
					self.pub.publish(self.vel_msg)
					#print "7"

				if rospy.Duration(61 * Target.DUR_MUL) <= rospy.get_rostime() - beginning_time - rospy.Duration(self.time_lost) and rospy.get_rostime() - beginning_time < rospy.Duration(69 * Target.DUR_MUL) - rospy.Duration(self.time_lost):
					self.curve_right(0)
					self.pub.publish(self.vel_msg)
					#print "8"

				if rospy.Duration(69 * Target.DUR_MUL) <= rospy.get_rostime() - beginning_time - rospy.Duration(self.time_lost) and rospy.get_rostime() - beginning_time < rospy.Duration(79 * Target.DUR_MUL) - rospy.Duration(self.time_lost):
					self.straight()
					self.pub.publish(self.vel_msg)
					#print "9"

				if rospy.Duration(79 * Target.DUR_MUL) <= rospy.get_rostime() - beginning_time - rospy.Duration(self.time_lost) and rospy.get_rostime() - beginning_time < rospy.Duration(89.5 * Target.DUR_MUL) - rospy.Duration(self.time_lost):
					self.curve_left(0)
					self.pub.publish(self.vel_msg)
					#print "10"

				if rospy.Duration(89.5 * Target.DUR_MUL) <= rospy.get_rostime() - beginning_time - rospy.Duration(self.time_lost) and rospy.get_rostime() - beginning_time < rospy.Duration(99.5 * Target.DUR_MUL) - rospy.Duration(self.time_lost):
					self.straight()
					self.pub.publish(self.vel_msg)
					#print "11"

				if rospy.Duration(99.5 * Target.DUR_MUL) <= rospy.get_rostime() - beginning_time - rospy.Duration(self.time_lost) and rospy.get_rostime() - beginning_time < rospy.Duration(109.25 * Target.DUR_MUL) - rospy.Duration(self.time_lost):
					self.curve_right(0)
					self.pub.publish(self.vel_msg)
					#print "12"

				if rospy.Duration(109.25 * Target.DUR_MUL) <= rospy.get_rostime() - beginning_time - rospy.Duration(self.time_lost) and rospy.get_rostime() - beginning_time < rospy.Duration(122.25 * Target.DUR_MUL) - rospy.Duration(self.time_lost):
					self.straight()
					self.pub.publish(self.vel_msg)
					print "13"

				if rospy.Duration(122.25 * Target.DUR_MUL) <= rospy.get_rostime() - beginning_time - rospy.Duration(self.time_lost) and rospy.get_rostime() - beginning_time < rospy.Duration(131.25 * Target.DUR_MUL) - rospy.Duration(self.time_lost):
					self.curve_right(0)
					self.pub.publish(self.vel_msg)
					print "14"

				if rospy.Duration(131.25 * Target.DUR_MUL) <= rospy.get_rostime() - beginning_time - rospy.Duration(self.time_lost) and rospy.get_rostime() - beginning_time < rospy.Duration(140.25 * Target.DUR_MUL) - rospy.Duration(self.time_lost):
					self.straight()
					self.pub.publish(self.vel_msg)
					print "15"

				if rospy.Duration(140.25 * Target.DUR_MUL) <= rospy.get_rostime() - beginning_time - rospy.Duration(self.time_lost) and rospy.get_rostime() - beginning_time < rospy.Duration(146.25 * Target.DUR_MUL) - rospy.Duration(self.time_lost):
					self.curve_right(0)
					self.pub.publish(self.vel_msg)
					print "16"

				if rospy.Duration(146.25 * Target.DUR_MUL) <= rospy.get_rostime() - beginning_time - rospy.Duration(self.time_lost) and rospy.get_rostime() - beginning_time < rospy.Duration(150.25 * Target.DUR_MUL) - rospy.Duration(self.time_lost):
					self.curve_left(0)
					self.pub.publish(self.vel_msg)
					print "17"

				if rospy.Duration(150.25 * Target.DUR_MUL) <= rospy.get_rostime() - beginning_time - rospy.Duration(self.time_lost) and rospy.get_rostime() - beginning_time < rospy.Duration(154.25 * Target.DUR_MUL) - rospy.Duration(self.time_lost):
					self.curve_right(1)
					self.pub.publish(self.vel_msg)
					print "18"

				if rospy.Duration(154.25 * Target.DUR_MUL) <= rospy.get_rostime() - beginning_time - rospy.Duration(self.time_lost) and rospy.get_rostime() - beginning_time < rospy.Duration(155.25 * Target.DUR_MUL) - rospy.Duration(self.time_lost):
					self.straight()
					self.pub.publish(self.vel_msg)
					print "19"

				
					

			# mode two functions
			if self.mode==2:
				self.find_state()
				print self.distance_front
				self.sin_move()

			# mode two functions
			if self.mode==3:
				self.find_state()
				print self.distance_front
				self.rand_move()
				

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

