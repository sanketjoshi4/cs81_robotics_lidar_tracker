#!/usr/bin/env python
import rospy
from geometry_msgs.msg import Twist

FREQ = 10 # Hz
SLEEP = 2

class Target:
	def __init__(self):
		rospy.init_node("target") # feel free to rename
		self.pub = rospy.Publisher("robot_1/cmd_vel", Twist, queue_size=0)
		rospy.sleep(SLEEP)

	def main(self):
		# setup code
		vel_msg = Twist()
		rate = rospy.Rate(FREQ)
		while not rospy.is_shutdown():
			vel_msg.linear.x = 0.1
			self.pub.publish(vel_msg)
			rate.sleep()

if __name__ == "__main__":
	# we'll probably set up target like this from main.py?
	t = Target()
	t.main()
