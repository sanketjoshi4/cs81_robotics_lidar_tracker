#!/usr/bin/env python

# Grid class to represent the map read by OccupancyGrid message by Josephine Nguyen

import numpy as np
import tf
import math


class World:
    def __init__(self, data, width, height, reso, frame, origin):
        """
        Initialize all info needed for transformations btwn grid & map and looking up cells
        @param data: row-major array or list of obstacle info in map, which is divided into grid cells
        @param width: world width
        @param height: world height
        @param reso: map resolution (for grids)
        @param frame: relative frame that determines grid map position
        @param origin: origin of grid map (bot left corner) relative to frame
        """
        self.data = data
        self.width = width
        self.height = height
        self.reso = round(reso, 3)
        self.frame = frame # map frame

        self.origin = [] # [x,y,yaw] representation of maze pose in map frame
        self.origin.append(origin.position.x)
        self.origin.append(origin.position.y)
        rpy = tf.transformations.euler_from_quaternion((origin.orientation.x, origin.orientation.y, origin.orientation.z, origin.orientation.w))
        self.origin.append(rpy[2]) # yaw is 3rd element
        self.T = np.array([[np.cos(self.origin[2]), -np.sin(self.origin[2]), 0, self.origin[0]], [np.sin(self.origin[2]), np.cos(self.origin[2]), 0, self.origin[1]], [0, 0, 1, 0], [0, 0, 0, 1]]) # transformation matrix from occ_grid to map frame
        print(self.T)

    def get_cell(self, x, y):
        """
        Return whether cell at x,y index has obstacle
        @param x: cell index in horz direction
        @param y: cell index in vert direction
        """
        return self.data[x + y * self.width]

    def map_to_grid(self, x, y, theta):
        """
        Transform x,y,yaw in map to grid
        @param x: cell index in horz direction
        @param y: cell index in vert direction
        @param theta: yaw to convert       
        """
        grid_pose = np.linalg.inv(self.T).dot(np.transpose(np.array([x, y, 0, 1])))
        grid_theta = theta - self.origin[2]
        return (grid_pose[0], grid_pose[1], grid_theta)

    def grid_to_map(self, x, y, theta):
        """
        Transform x,y,yaw in grid to map
        @param x: cell index in horz direction
        @param y: cell index in vert direction
        @param theta: yaw to convert       
        """
        map_pose = self.T.dot(np.transpose(np.array([x, y, 0, 1])))
        map_theta = theta + self.origin[2]
        return (map_pose[0], map_pose[1], map_theta)

    def grid_to_cell(self, x, y):
        """
        Transform x,y grid coordinates in meters to cell indexes
        @param x: distance in x-axis
        @param y: distance in y-axis
        """
        return (int(math.floor(x / self.reso)), int(math.floor(y / self.reso)))

    def cell_to_grid(self, x , y):
        """
        Transform x,y cell indexes to grid coordinates, rounded to be center of cell
        @param x: cell index in horz direction
        @param y: cell index in vert direction
        """
        return ((x * self.reso) + (self.reso / 2), (y * self.reso) + (self.reso / 2))

    def on_grid(self, x, y):
        """
        Returns whether x,y cell indexes are on grid
        @param x: cell index in horz direction
        @param y: cell index in vert direction
        """
        return 0 <= x < self.width and 0 <= y < self.height
