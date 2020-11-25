#!/usr/bin/env python
import rospy
from geometry_msgs.msg import Twist
import math
import random
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
	def update_targetpos(self, posx, posy, xvel, yvel):

		# if the target was not found, clearing the poses list and returning
		if None in [posx, posy, xvel, yvel]:
			self.poses = []
			return None

		# creating new instance of pose class
		target = Pose(posx, posy, xvel, yvel)
		last = None

		# with more than one observation, velocities are there
		if len(self.poses) >= 1:
			last = self.poses[-1]
			# getting acceleration
			target.set_xaccel( target.get_xvel() - last.get_xvel() ) 
			target.set_yaccel( target.get_yvel() - last.get_yvel() ) 
	
		# with more than two observations, accelerations are there
		if len(self.poses) >= 2:
			target.set_xthird( target.get_xaccel() - last.get_xaccel() ) 
			target.set_ythird( target.get_yaccel() - last.get_yaccel() )  
			
		# adding new pose to the poses list
		self.poses.append(target)

	# given poses, return robot intended linear and angular velocity
	def predict(self, dt): 

		# if there is nothing in the poses list, simply returning
		if len(self.poses)<1:
			return []

		last_obs = self.poses[-1]
		if len(self.poses) > 1:
			seclast_obs = self.poses[-2]
		## https://docs.python.org/3/library/math.html
		# using kalman filtering model, prediction is the last position + predicted changes in velocity; 80% last obs, 20% obs before that
			self.predx_vel = 0.8*last_obs.get_xvel() + 0.2*seclast_obs.get_xvel()
			self.predy_vel =0.8*last_obs.get_yvel() + 0.2*seclast_obs.get_yvel()
		else: # just usinng last observation
			self.predx_vel = last_obs.get_xvel()
			self.predy_vel = last_obs.get_yvel() 
		predtuple = (self.predx_vel, self.predy_vel)
		# getting five next poses, each over the span of 5 dts
		predposes = []		
		i = 0
		while i < 5:
			xnew = last_obs.get_posx() + (i+1)*self.predx_vel*dt
			ynew = last_obs.get_posy() + (i+1)*self.predy_vel*dt
			predposes.append( (xnew, ynew) ) 		
			i += 1
		# returning the predicted velocities in the tuple and the next five poses
		return (predtuple, predposes)


	# more advanced version of prediiction 
	def predict_hd(self, dt, lookahead):

		# skipping if there is nothing in the list and therefore returning empty list of poses for predict
		if len(self.poses)<1:
			return []

		total_xthird = 0
		total_ythird = 0
		# summing over all the poses in the list
		for pose in self.poses:
			xthird = pose.get_xthird()
			ythird = pose.get_ythird()
			if xthird!=None and ythird!=None:
				total_xthird += xthird 
				total_ythird += ythird
		# averaging the third value over all the values
		avg_xthird = total_xthird/( len(self.poses) )
		avg_ythird = total_ythird/( len(self.poses) )
		# if there were no actual third observations, randomizing
		if avg_xthird==0 and avg_ythird==0:
			# https://docs.python.org/3/library/random.html
			avg_xthird = random.uniform(-1, 1)
			avg_ythird = random.uniform(-1, 1)

		# these above average_third values are jerk values that will be used for the next prediction

		last_obs = self.poses[-1]
		# if the there are more observations in the list, having the predicted values be 'integrals'
		predposes = []	
		predvel = []
		predacc = []
		if len(self.poses) >= 2:
			# creating projected poses, adding the first one
			new_xaccel = last_obs.get_xaccel() + avg_xthird
			new_yaccel = last_obs.get_yaccel() + avg_ythird
			# predicted velocities
			self.predx_vel = last_obs.get_xvel() + new_xaccel
			self.predy_vel = last_obs.get_yvel() + new_yaccel
			# normalizing if either of the velocities are greater than one
			(nx, ny) = self.normalize(self.predx_vel, self.predy_vel)
			self.predx_vel = nx
			self.predy_vel = ny
			predacc.append( (new_xaccel, new_yaccel) ) 
					
		else: # using the last velocity until enough observations
			# just using the last observations velocity as the predicted
			self.predx_vel = last_obs.get_xvel()
			self.predy_vel = last_obs.get_yvel()
			predacc.append( (0, 0) )

		# creating velocities and x
		xnew = last_obs.get_posx() + self.predx_vel*dt
		ynew = last_obs.get_posy() + self.predy_vel*dt
		# appending to poses and velocities
		predposes.append( (xnew, ynew) )
		predvel.append( (self.predx_vel, self.predy_vel) )
		
	
		i = 0
		#already looked ahead once, looking ahead again
		while i < lookahead-1: 
			# adding previously predicted acc to the jerk 
			new_xaccel = predacc[i][0] + avg_xthird
			new_yaccel = predacc[i][1] + avg_ythird
			predacc.append( (new_xaccel, new_yaccel) ) 
			# getting new velocities
			predx_vel = predvel[i][0] + new_xaccel
			predy_vel = predvel[i][1] + new_yaccel
			# normalizing velocities
			(nvx, nvy) = self.normalize(predx_vel, predy_vel)
			predvel.append( (nvx, nvy) )
			# predicting new pose
			xnew = predposes[i][0] + nvx*dt
			ynew = predposes[i][1] + nvy*dt
			predposes.append( (xnew, ynew) )
			i += 1

		# returning the predicted velocities in the tuple and the next N poses
		print predposes
		return  predposes

	# function that normalizes velocity
	def normalize(self, x, y):
		if x > 1 or y > 1:
			newx = x / ( x * x + y * y)
			newy = y / ( x * x + y * y)
			return (newx, newy)
		else: 
			return (x, y)
		

	#def main(self):
		#while not rospy.is_shutdown():
			#self.update_targetpos(1, 1, 1, 0)
			#self.predict_hd(0.1, 5)
			#self.predict_hd(0.1, 5)
			#self.update_targetpos(0, 0, 1, 1)
			#self.predict_hd(0.1, 5)

			#break

# mini class that has all the current information on target's whereabouts
class Pose:
	def __init__(self, positionx, positiony, xvelocity, yvelocity):
		self.posx = positionx
		self.posy = positiony
		self.xvel = xvelocity
		self.yvel = yvelocity
		self.xaccel = None	
		self.yaccel = None
		self.xthird = None
		self.ythird = None


	# getter functions
	def get_posx(self):
		return self.posx

	def get_posy(self):
		return self.posy

	def get_xvel(self):
		return self.xvel

	def get_yvel(self):
		return self.yvel

	def get_xaccel(self):
		return self.xaccel

	def get_yaccel(self):
		return self.yaccel

	def get_xthird(self):
		return self.xthird

	def get_ythird(self):
		return self.ythird

	#setter functions
	def set_xaccel(self, value):
		self.xaccel = value

	def set_yaccel(self, value):
		self.yaccel = value

	def set_xthird(self, value):
		self.xthird = value
	
	def set_ythird(self, value):
		self.ythird = value
		
if __name__ == "__main__":

	p = Predictor()
	p.main()

	
