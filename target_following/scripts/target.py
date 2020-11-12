#!/usr/bin/env python
import rospy
from geometry_msgs.msg import Twist

# TARGET.PY 
# class for the target object 

FREQ = 10 # Hz
SLEEP = 2

VEL = 0.1

class Target:
	def __init__(self):
		rospy.init_node("target") # target node 
		self.pub = rospy.Publisher("robot_1/cmd_vel", Twist, queue_size=0)
		rospy.sleep(SLEEP)

		self.posx = 0 # initalizing target x position at 0
		self.posy = 0 # initializing target y position at 0
		self.linxvel = VEL # initializing linear velocity at 0
		self.angzvel = 0 # initializing angular velocity at 0 
		
		self.vel_msg = Twist() # creating inital publish message, which is altered in the main

	# move function that dictates next target velocities and locations
	def move(self):
		pass

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

if __name__ == "__main__":
	# we'll probably set up target like this from main.py?
	t = Target()
	t.main()
