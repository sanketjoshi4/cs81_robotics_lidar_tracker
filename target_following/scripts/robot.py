#!/usr/bin/env python
import rospy
from geometry_msgs.msg import Twist

FREQ = 10 # Hz
SLEEP = 2

class Robot:
	def __init__(self):
		rospy.init_node("robot") # feel free to rename	
		self.pub = rospy.Publisher("robot_0/cmd_vel", Twist, queue_size=0)

		# TODO: uncomment as you make these files and classes
		# self.pred = Predictor()
		# self.rec = Recovery()
		# self.id = Identifier()

		rospy.sleep(SLEEP)

	def move(self):
		# setup code, right now just moves
		vel_msg = Twist()
		# TODO: get predicted x and z velocities from Predictor, combine with Identifier info to calculate move

		rate = rospy.Rate(FREQ)
		while not rospy.is_shutdown():
			vel_msg.linear.x = 0.1
			self.pub.publish(vel_msg)
			rate.sleep()

	def main(self):
		self.move()

if __name__ == "__main__":
	# we'll probably set up target like this from main.py?
	r = Robot()
	r.main()
