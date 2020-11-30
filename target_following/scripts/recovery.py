from geometry_msgs.msg import Point, Pose, Quaternion
import utils
import heapq as hq
import numpy as np
import math
import tf


class Recovery:
    EDGE_WEIGHT = 1  # constant edge weight because everything is 1 grid square away from us, may change this
    SEARCH_RANGE = 5  # grid squares
    LIDAR_RADIUS = 2  # m
    RESO = 0.05  # m

    def __init__(self):
        """
        Initialize all instance variables for path finding
        """
        # for path finding
        self.start = []
        self.end = []

        # last known pose of target; right now hard-coded, but later Robot should pass this pose to Recovery object
        self.last_known_pos = Point()
        self.last_known_pos.x = 4
        self.last_known_pos.y = 5

        # robot's current pose (pose at which we adopt recovery mode)
        self.robot_pos = None
        self.robot_ang = 0  # map frame yaw in rads

        # caller must construct local world before path-finding
        self.world = None

        print("init done")

    def create_local_world(self, blobs):
        """
        Construct local world from blobs; must call before calling recover()
        @param blobs: dict of {blob_id:Blob()}
        """
        # map metadata
        world_w = int(Recovery.LIDAR_RADIUS * 2 / Recovery.RESO) + 1  # account for center position where robot is
        world_h = int(Recovery.LIDAR_RADIUS * 2 / Recovery.RESO) + 1
        world_res = Recovery.RESO
        world_frame = "map"

        # array of obstacles info
        world_arr = np.zeros(world_w * world_h, dtype=int)
        world_arr[:] = 2
        cx = int(world_w / 2)
        cy = int(world_h / 2)
        for x in range(cx - int(Recovery.LIDAR_RADIUS / Recovery.RESO),
                       cx + int(Recovery.LIDAR_RADIUS / Recovery.RESO) + 1):
            for y in range(cy - int(Recovery.LIDAR_RADIUS / Recovery.RESO),
                           cy + int(Recovery.LIDAR_RADIUS / Recovery.RESO) + 1):
                if (x - cx) ** 2 + (y - cy) ** 2 <= (Recovery.LIDAR_RADIUS / Recovery.RESO) ** 2:
                    world_arr[x + y * world_w] = 0

        for blob_id in blobs:
            arr = blobs[blob_id].arr
            for pos_x, pos_y in arr:
                x = round((pos_x - (self.robot_pos.x - Recovery.LIDAR_RADIUS)) / Recovery.RESO)
                y = round((pos_y - (self.robot_pos.y - Recovery.LIDAR_RADIUS)) / Recovery.RESO)
                # print("obs at", x, y)
                world_arr[x + y * world_w] = 1

        # info about grid map's pose relative to 'map' frame
        origin = Pose()
        origin.position = Point()
        origin.position.x = self.robot_pos.x - Recovery.LIDAR_RADIUS
        origin.position.y = self.robot_pos.y - Recovery.LIDAR_RADIUS
        origin.position.z = 0

        # no rotation
        origin.orientation = Quaternion()
        quaternion = tf.transformations.quaternion_from_euler(0, 0, 0)
        origin.orientation.x = quaternion[0]
        origin.orientation.y = quaternion[1]
        origin.orientation.z = quaternion[2]
        origin.orientation.w = quaternion[3]

        self.world = World(world_arr, world_w, world_h, world_res, world_frame, origin)

    def recover(self):
        """
        Returns a list of poses in map frame that robot needs to be in, in order to move to last_known_pos
        MUST update self.last_known_pos and self.robot_pos (from Robot object) before calling this!
        """
        # assume in map frame
        self.end = [self.last_known_pos.x, self.last_known_pos.y]
        self.start = [self.robot_pos.x, self.robot_pos.y]
        print("finding path from ", (utils.show(self.start[0]), utils.show(self.start[1])), "to ",
              (utils.show(self.end[0]), utils.show(self.end[1])))

        print(self.world.T)

        # convert from map to grid; theta doesn't matter here
        start_grid_x, start_grid_y, theta = self.world.map_to_grid(self.start[0], self.start[1], 0)
        end_grid_x, end_grid_y, theta = self.world.map_to_grid(self.end[0], self.end[1], 0)

        # convert from grid to cell
        start_cell_x, start_cell_y = self.world.grid_to_cell(start_grid_x, start_grid_y)
        end_cell_x, end_cell_y = self.world.grid_to_cell(end_grid_x, end_grid_y)

        if self.is_near_obs(end_cell_x, end_cell_y):
            end_cell_x, end_cell_y = self.get_nearest_free(end_cell_x, end_cell_y)
            if end_cell_x == -1:
                print("error getting nearest free cell")
                return []

        print(start_cell_x, start_cell_y)
        print(end_cell_x, end_cell_y)

        self.start = [start_cell_x, start_cell_y]
        self.end = [end_cell_x, end_cell_y]

        # check coords are actually on grid
        if not self.world.on_grid(start_cell_x, start_cell_y) or not self.world.on_grid(end_cell_x, end_cell_y):
            print("not on grid")
            return []
        # if start or end coordinates have obstacle in them, then it's bad input
        if self.world.get_cell(start_cell_x, start_cell_y) or self.world.get_cell(end_cell_x, end_cell_y):
            print("inside obstacle")
            return []

        # self.world.data[start_cell_x + start_cell_y * self.world.width] = 5
        # self.world.data[end_cell_x + end_cell_y * self.world.width] = 7

        # print("MAP IS")
        # for h in range(self.world.height):
        #    s = "["
        #    for w in range(self.world.width):
        #        s += str(self.world.data[w + h * self.world.width]) + ","
        #    s += "]"
        #    print(s)

        path = self.a_star()
        return self.get_path_poses(path)

    def get_nearest_free(self, x, y):
        """
        Returns the nearest cell index x,y that's not within SEARCH_RANGE of an obstacle
        @param x: x index
        @param y: y index
        """
        for ny in range(max(0, y - Recovery.SEARCH_RANGE * 2),
                        min(self.world.height, y + Recovery.SEARCH_RANGE * 2 + 1)):
            for nx in range(max(0, x - Recovery.SEARCH_RANGE * 2),
                            min(self.world.width, x + Recovery.SEARCH_RANGE * 2 + 1)):
                if ny != y or nx != x:
                    if not self.is_near_obs(nx, ny):
                        return nx, ny

        return -1, -1

    def diagonal(self, curr_x, curr_y):
        """
        Returns diagonal distance from curr_x,curr_y to self.end[0],self.end[1]--BOTH IN CELL COORDS
        @param curr_x: current x cell index whose diagonal distance to end we want
        @param curr_y: current y cell index
        """
        dx = abs(self.end[0] - curr_x)
        dy = abs(self.end[1] - curr_y)
        return (dx + dy) - min(dx, dy)

    def a_star(self):
        """
        Implements A* algorithm to find path from start to goal
        Assumes movement up, down, left, right only
        Actual cost is always 1 because every next cell should be just 1 cell away from curr cell
        """
        start = Cell(self.start)
        goal = Cell(self.end)

        # stores (Cell objs, heuristic)
        open_set = []  # min heap weighted by heuristic/diagonal to goal
        hq.heappush(open_set, (self.diagonal(start.coords[0], start.coords[1]), start))

        closed_set = {}  # dict maps curr_node:parent_node for back tracing later

        closed_set_angles = {}  # maps curr_node:optimal angle to get here from prev node

        closed_set_angles[start] = self.robot_ang

        g_vals = {}  # dict maps node to actual cost to get from start to node
        g_vals[start] = 0

        while len(open_set) > 0:
            # get next lowest in open set; 1st index is Cell obj because 0th index is diagonal
            curr = hq.heappop(open_set)[1]

            # we've found goal, back trace and return
            if curr == goal:
                print("found")
                path = []
                while curr in closed_set:
                    path.append(curr)
                    curr = closed_set[curr]
                print("found 1")
                return path  # [goal, ..., c, b, a] reversed so we can pop() later

            # look at all neighbors in y dir
            cx = curr.coords[0]
            cy = curr.coords[1]
            for ny in range(max(0, cy - 1), min(cy + 2, self.world.height)):
                for nx in range(max(0, cx - 1), min(cx + 2, self.world.width)):
                    # if not current cell and doesn't have obstacle
                    if (cy != ny or cx != nx) and not self.world.get_cell(nx, ny) and not self.is_near_obs(nx, ny):
                        n = Cell([nx, ny])
                        temp_yaw = self.get_rotate_yaw(cx, cy, closed_set_angles[curr], nx, ny)
                        temp_g = g_vals[curr] + Recovery.EDGE_WEIGHT + abs(temp_yaw)
                        # if cost from start to n is lowest so far, or no cost calculated yet
                        if (n in g_vals and temp_g < g_vals[n]) or (n not in g_vals):
                            closed_set[n] = curr  # update shortest path to n
                            closed_set_angles[n] = temp_yaw
                            g_vals[n] = temp_g
                            temp_f = g_vals[n] + self.diagonal(nx, ny)
                            for i in range(len(open_set)):
                                # remove n from open set if already there
                                if open_set[i][1] == n:
                                    del open_set[i]
                                    break
                            hq.heappush(open_set, (temp_f, n))
                            hq.heapify(open_set)

        return []

    def get_rotate_yaw(self, x1, y1, yaw1, x2, y2):
        """
        Get the amount to rotate to cell index (x2,y2) given robot is at index (x1,y1) with yaw1
        @param x1: curr cell x index
        @param y1: curr cell y index
        @param yaw1: yaw in curr cell
        @param x2: next cell x index
        @param y2: next cell y index
        """
        if x2 == x1:
            # next cell below
            if y2 < y1:
                return np.pi / 2 - yaw1
            # next cell above
            if y2 > y1:
                return -np.pi / 2 - yaw1
        if y2 == y1:
            # next cell behind
            if x2 < x1:
                return np.pi - yaw1
            # next cell ahead
            if x2 > x1:
                return 0 - yaw1
        if x2 < x1:
            # next cell top left
            if y2 < y1:
                return np.pi * 0.75 - yaw1
            # next cell bot left
            if y2 > y1:
                return -np.pi * 0.75 - yaw1
        if x2 > x1:
            # next cell top right
            if y2 < y1:
                return np.pi * 0.25 - yaw1
            # next cell bot right
            if y2 > y1:
                return -np.pi * 0.25 - yaw1
        print("ERROR IN GET YAW ROTATE")

    def is_near_obs(self, x, y):
        """
        Check if cell index at x,y is near a rock, to take into account of robot size in map
        @param x: curr cell x index
        @param y: curr cell y index
        """
        # checks within 5 grid squares around curr cell
        for ny in range(max(0, y - Recovery.SEARCH_RANGE), min(self.world.height, y + Recovery.SEARCH_RANGE + 1)):
            for nx in range(max(0, x - Recovery.SEARCH_RANGE), min(self.world.width, x + Recovery.SEARCH_RANGE + 1)):
                if nx != x or ny != y:
                    if self.world.get_cell(nx, ny):  # has obstacle
                        return True
        return False

    def get_path_poses(self, path):
        """
        Get poses in map as list of [x, y, yaw] from list of cell coordinates/indexes
        Assumes movement up, down, left, right only
        @param path: list of cell indexes in format [x, y]
        """
        # returns 2D list where items are [x, y, yaw] in map frame
        poses = []
        px = None
        py = None

        for i in range(len(path)):
            cx = path[i].coords[0]
            cy = path[i].coords[1]

            # don't add redundant cells
            if i > 0 and (px - 2 <= cx <= px + 2 or py - 2 <= cy <= py + 2):
                continue

            # get x,y in grid
            grid_x, grid_y = self.world.cell_to_grid(cx, cy)

            # get x,y in map; yaw doesn't matter
            map_x, map_y, map_yaw = self.world.grid_to_map(grid_x, grid_y, 0)
            poses.append([map_x, map_y])

            # update prev cells
            px = cx
            py = cy
        return poses


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
        self.frame = frame  # map frame

        self.origin = []  # [x,y,yaw] representation of maze pose in map frame
        self.origin.append(origin.position.x)
        self.origin.append(origin.position.y)
        rpy = tf.transformations.euler_from_quaternion(
            (origin.orientation.x, origin.orientation.y, origin.orientation.z, origin.orientation.w))
        self.origin.append(rpy[2])  # yaw is 3rd element
        self.T = np.array([[np.cos(self.origin[2]), -np.sin(self.origin[2]), 0, self.origin[0]],
                           [np.sin(self.origin[2]), np.cos(self.origin[2]), 0, self.origin[1]], [0, 0, 1, 0],
                           [0, 0, 0, 1]])  # transformation matrix from occ_grid to map frame
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

    def cell_to_grid(self, x, y):
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


class Cell:
    def __init__(self, coords):
        """
        Stores only [x,y] coords/indexes in grid
        @param coords: list of form [x,y] indicating cell index
        """
        self.coords = coords  # expect [x,y] list

    def __hash__(self):
        """
        To make checks like 'if c in dict' work
        """
        return hash((self.coords[0], self.coords[1]))

    def __eq__(self, other):
        """
        To make checks like 'if c in dict' and 'if c == c1' work
        @param other: another Cell object to compare to
        """
        return self.coords[0] == other.coords[0] and self.coords[1] == other.coords[1]

    def __ne__(self, other):
        """
        To make checks like 'if c in dict' and 'if c == c1' work
        @param other: another Cell object to compare to
        """
        return self.coords[0] != other.coords[0] or self.coords[1] != other.coords[1]

    def __str__(self):
        """
        For debug prints
        """
        return str(self.coords[0]) + "," + str(self.coords[1])
