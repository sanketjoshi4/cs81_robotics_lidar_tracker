#!/usr/bin/env python
import rospy
from geometry_msgs.msg import Twist
from identifier.py import Identifier # importing Identifier

# PREDICTOR.PY 
# class to predict robot movement

FREQ = 10 # Hz
SLEEP = 2

class Predictor:
	def __init__(self):

		# target poses and velocities
		self.poses = []
		self.linvel = None
		self.angvel = None

	# given pose information from Identifier, updating instance variable for Predictors
	def update_targetpos(self, posx, posy, linvel, angvel):
		# creating new instance of pose class
		target = Pose(posx, posy, linvel, angvel)
		# adding new pose to the poses list
		poses.append(target)

	# given poses, return robot intended linear and angular velocity
	def predict(self):
		continue

# mini class that has all the current information on target's whereabouts
class Pose:
	def __init__(self, positionx, positiony, linvelocity, angvelocity):
		self.posx = positionx
		self.posy = positiony
		self.linvel = linvelocity
		self.angvel = angvelocity

	# getter functions
	def get_posx(self):
		return self.posx

	def get_posy(self):
		return self.posy

	def get_linvel(self):
		return self.linvel

	def ge_angvel(self):
		return self.angvel
		

		
		

	
