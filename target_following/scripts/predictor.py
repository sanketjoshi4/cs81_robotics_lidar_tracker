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
		# creating new instance of pose class
		target = Pose(posx, posy, xvel, yvel)
	
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
		print (predtuple, predposes)
		return (predtuple, predposes)


	# more advanced version of prediiction 
	def predict_hd(self, dt, lookahead):

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

		last_obs = self.poses[-1]
		# if the there are more observations in the list, having the predicted values be 'integrals'
		if len(self.poses) >= 2:
		# getting predicted accelerations
			new_xaccel = last_obs.get_xaccel() + avg_xthird
			new_yaccel = last_obs.get_yaccel() + avg_ythird
			# predicted velocities
			predx_vel = last_obs.get_xvel() + new_xaccel
			predy_vel = last_obs.get_yvel() + new_yaccel
			
		else: # using the last velocity until enough observations
			predx_vel = last_obs.get_xvel()
			predy_vel = last_obs.get_yvel() 
		
		# normalizing the velocity, since the max is 1 m/s
		self.predx_vel = predx_vel/ (predx_vel*predx_vel + predy_vel*predy_vel) 
		self.predy_vel = predy_vel/ (predx_vel*predx_vel + predy_vel*predy_vel) 
		predtuple = (self.predx_vel, self.predy_vel)
		
		predposes = []		
		i = 0
		while i < lookahead:
			xnew = last_obs.get_posx() + (i+1)*self.predx_vel*dt
			ynew = last_obs.get_posy() + (i+1)*self.predy_vel*dt
			predposes.append( (xnew, ynew) )		
			i += 1
		# returning the predicted velocities in the tuple and the next five poses
		print (predtuple, predposes)
		return (predtuple, predposes)

			
		

	#def main(self):
		#while not rospy.is_shutdown():
			#self.update_targetpos(1, 1, 1, 0)
			#self.predict_hd(0.1)
			#self.update_targetpos(2, 1, 0, 1)
			#self.predict_hd(0.1)
			#self.update_targetpos(2, 2, 0, 0)
			#self.predict_hd(0.1)

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

	
