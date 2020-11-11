#!/usr/bin/env python
import rospy
import tf
import numpy as np
from geometry_msgs.msg import Twist, Point
from nav_msgs.msg import OccupancyGrid
from recovery import Recovery
from world import World

FREQ = 10 # Hz
SLEEP = 2
VEL = 0.1 # m/s

class Robot:
	def __init__(self):
		rospy.init_node("robot") # feel free to rename	
		self.pub = rospy.Publisher("robot_0/cmd_vel", Twist, queue_size=0)
		self.map_sub = rospy.Subscriber("map", OccupancyGrid, self.map_callback, queue_size=1)
		self.lis = tf.TransformListener()
		self.mTo = np.array([[-1, 0, 1, 5], [0, -1, 0, 5], [0, 0, 1, 0], [0, 0, 0, 1]])
		self.bTo = None # odom to base_link
		self.map = None
		self. rcvr = None

		# TODO: uncomment as you make these files and classes
		# self.pred = Predictor()
		# self.id = Identifier()

		rospy.sleep(SLEEP)

	def map_callback(self, msg):
		print("loading map")
		self.map = World(msg.data, msg.info.width, msg.info.height, msg.info.resolution, msg.header.frame_id, msg.info.origin)
		self.rcvr = Recovery(self.map)

	def get_transform(self):
		# get transformation from odom to base_link		
		(trans, rot) = self.lis.lookupTransform('robot_0/base_link', 'robot_0/odom', rospy.Time(0))
		
		# get everything in regular matrix form
		t = tf.transformations.translation_matrix(trans)
		r = tf.transformations.quaternion_matrix(rot)
		self.bTo = t.dot(r)

	def move(self):
		# setup code, right now just moves
		vel_msg = Twist()
		# TODO: get predicted x and z velocities from Predictor, combine with Identifier info to calculate move

		rate = rospy.Rate(FREQ)
		while not rospy.is_shutdown():
			vel_msg.linear.x = VEL
			self.pub.publish(vel_msg)
			rate.sleep()

	def main(self):
		print("in main")
		while self.rcvr is None:
			continue

		self.rcvr.robot_pos = Point()
		p = self.mTo.dot( np.transpose(np.array([0, 0, 0, 1])) )[0:2]
		self.rcvr.robot_pos.x = p[0]
		self.rcvr.robot_pos.y = p[1]
		print(p)
		print('---')

		vel_msg = Twist()
		rate = rospy.Rate(FREQ)

		poses = self.rcvr.predict() # expect [[x,y],[x,y],...]
		print(poses)
		for pose in poses:
			# transform user-given point in odom to base_link, assume ROBOT CAN'T FLY
			self.get_transform()
			v = np.linalg.inv(self.mTo).dot( np.transpose(np.array([pose[0], pose[1], 0, 1])) )
			v = self.bTo.dot(v)
			v = v[0:2] # only need x,y because of assumption above
			print(v)
			# angle to turn i.e. angle btwn x-axis vector and vector of x,y above
			a = np.arctan2(v[1], v[0])
			# euclidean distance to travel, assume no movement in z-axis
			l = np.linalg.norm(np.array([0,0]) - v)
			start_time = rospy.get_rostime()
			if a >= 0: # anticlockwise rot (or no rot)
				start_time = rospy.get_rostime()
				while not rospy.is_shutdown() and rospy.get_rostime() - start_time < rospy.Duration(a / VEL):
					vel_msg.angular.z = VEL
					vel_msg.linear.x = 0
					self.pub.publish(vel_msg)
					rate.sleep()
			else:
				start_time = rospy.get_rostime()
				while not rospy.is_shutdown() and rospy.get_rostime() - start_time < rospy.Duration(-a / VEL):
					vel_msg.angular.z = -VEL
					vel_msg.linear.x = 0
					self.pub.publish(vel_msg)
					rate.sleep()

			start_time = rospy.get_rostime()
			while not rospy.is_shutdown() and rospy.get_rostime() - start_time < rospy.Duration(l / VEL):
				vel_msg.angular.z = 0
				vel_msg.linear.x = VEL
				self.pub.publish(vel_msg)
				rate.sleep()

if __name__ == "__main__":
	# we'll probably set up target like this from main.py?
	r = Robot()
	r.main()
