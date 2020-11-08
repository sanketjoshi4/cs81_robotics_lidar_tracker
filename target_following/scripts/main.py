#!/usr/bin/env python

import rospy
from map import Map

class Main:
	def __init__(self):
		rospy.init_node("main_node")
		self.world = None
		self.robot = Robot()
		self.target = Target()
		self.map_sub = rospy.Subscriber("map", OccupancyGrid, self.map_callback, queue_size=1)
		self.lsr_sub = rospy.Subscriber("robot_0/base_scan", LaserScan, self.laserscan_callback, queue_size=1) # don't clog up subscriber
		
	def map_callback(self, msg):
		self.world = Map(msg.data, msg.info.width, msg.info.height, msg.info.resolution, msg.header.frame_id, msg.info.origin)	
