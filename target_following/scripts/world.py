#!/usr/bin/env python

# Grid class to represent the map read by OccupancyGrid message

import numpy as np
import tf
import math


class Map:
	def __init__(self, data, width, height, reso, frame):
		""" Preliminary info from OccupancyGrid we expect to use """
		self.data = data
		self.width = width
		self.height = height
		self.reso = round(reso, 3)
		self.frame = frame # map frame

	def get_cell(self, x, y):
		""" Return whether cell at x,y index has obstacle; x,y is cell index """
		return self.data[x + y * self.width]

	def on_grid(self, x, y):
		""" Returns whether x,y cell indexes are on grid """
		return 0 <= x < self.width and 0 <= y < self.height
