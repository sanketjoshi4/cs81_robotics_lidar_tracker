#!/usr/bin/env python
import rospy
from geometry_msgs.msg import Twist
import math
#from identifier.py import Identifier # importing Identifier

# PREDICTOR.PY 
# class to predict robot movement

FREQ = 10 # Hz
SLEEP = 2

class Predictor:
	def __init__(self):

		# target poses
		self.poses = []

		# predicted target velocities
		self.predx_vel = 0
		self.predy_vel = 0

	# given pose information from Identifier, updating instance variable for Predictors
	def update_targetpos(self, posx, posy, yaw, linvel, angvel):
		# creating new instance of pose class
		target = Pose(posx, posy, yaw, linvel, angvel)
		# adding new pose to the poses list
		self.poses.append(target)

	# given poses, return robot intended linear and angular velocity
	def predict(self): 
		last_obs = self.poses[-1]
		angle = last_obs.get_yaw()
		## https://docs.python.org/3/library/math.html
		# using kalman filtering model, prediction is the last position + predicted changes in velocity
		self.predx_vel = math.cos(angle)*last_obs.get_linvel()
		self.predy_vel = math.sin(angle)*last_obs.get_linvel()
		print self.predx_vel
		print self.predy_vel

	#def main(self):
		#while not rospy.is_shutdown():
			#self.update_targetpos(self.posx, self.posy, self.linvel, self.angvel)
			#self.predict()

# mini class that has all the current information on target's whereabouts
class Pose:
	def __init__(self, positionx, positiony, theta, linvelocity, angvelocity):
		self.posx = positionx
		self.posy = positiony
		self.yaw = theta # the angle around z axis
		self.linvel = linvelocity
		self.angvel = angvelocity

	# getter functions
	def get_posx(self):
		return self.posx

	def get_posy(self):
		return self.posy

	def get_yaw(self):
		return self.yaw

	def get_linvel(self):
		return self.linvel

	def get_angvel(self):
		return self.angvel
		
if __name__ == "__main__":

	p = Predictor()
	p.main()

	
