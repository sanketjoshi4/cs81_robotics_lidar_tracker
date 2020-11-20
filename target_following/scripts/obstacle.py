#!/usr/bin/env python
import rospy
from geometry_msgs.msg import Twist
from nav_msgs.msg import Odometry
import numpy as np
import tf
import math

# OBSTACLE.PY
# class for the obstacle objects 

class Obstacle:

	FREQ = 10  # Hz
	SLEEP = 2
	PI = np.pi
	LINVELOCITY = 0.2
	ANGVELOCITY = 0.2

	def __init__(self):
		rospy.init_node("obstacles")
		# relevant publishers for each obstacle
		self.pub1 = rospy.Publisher("robot_2/cmd_vel", Twist, queue_size=0)
		self.pub2 = rospy.Publisher("robot_3/cmd_vel", Twist, queue_size=0)
		self.pub3 = rospy.Publisher("robot_4/cmd_vel", Twist, queue_size=0)
		self.pub4 = rospy.Publisher("robot_5/cmd_vel", Twist, queue_size=0)

		# velocity messages
		self.vel_msg1 = Twist()
		self.vel_msg2 = Twist()
		self.vel_msg3 = Twist()
		self.vel_msg4 = Twist()

	def move(self):
		# moving each of the obstacles
		time = rospy.get_rostime()
		self.vel_msg1.linear.x = math.sin(time.to_sec())
		self.pub1.publish(self.vel_msg1)
		self.vel_msg2.linear.x = math.cos(1.2*time.to_sec())
		self.pub2.publish(self.vel_msg2)
		self.vel_msg3.linear.x = math.cos(2*time.to_sec())
		self.pub3.publish(self.vel_msg3)
		self.vel_msg4.linear.x = math.sin(0.85*time.to_sec())
		self.pub4.publish(self.vel_msg4)

	def main(self):
		while not rospy.is_shutdown():
			self.move()
		
# for testing, to call, follow same method
#if __name__ == "__main__":
	#o = Obstacle()
	#o.main()
			
			
	
	
		
	

