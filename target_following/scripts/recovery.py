from geometry_msgs.msg import Point
from cell import Cell
import follower_utils
from world import World
import heapq as hq


LOST_THRESH = 10 # s
EDGE_WEIGHT = 1 # constant edge weight because everything is 1 grid square away from us, may change this
SEARCH_RANGE = 10 # grid squares


class Recovery:
    def __init__(self, world):
        # for path finding
        self.start = []
        self.end = []

        # last known pose of target; right now hard-coded, but later Robot should pass this pose to Recovery object
        self.last_known_pos = Point()
        self.last_known_pos.x = 4.5
        self.last_known_pos.y = 4.5

        # robot's current pose (pose at which we adopt recovery mode)
        self.robot_pos = None

        # output stats for later?
        self.elapsed_lost_time = 0

        # full knowledge of world
        self.world = world

        print("init done")

    def recover(self):
        """
        Returns a list of poses in map frame that robot needs to be in, in order to move to last_known_pos
        MUST update self.last_known_pos and self.robot_pos (from Robot object) before calling this!
        """
        # assume in map frame
        self.end = [self.last_known_pos.x, self.last_known_pos.y]
        self.start = [self.robot_pos.x, self.robot_pos.y]
        print("finding path from ", (follower_utils.show(self.start[0]),follower_utils.show(self.start[1])), "to ",
                  (follower_utils.show(self.end[0]),follower_utils.show(self.end[1])))

        # convert from map to grid; theta doesn't matter here
        start_grid_x, start_grid_y, theta = self.world.map_to_grid(self.start[0], self.start[1], 0)
        end_grid_x, end_grid_y, theta = self.world.map_to_grid(self.end[0], self.end[1], 0)

        # convert from grid to cell
        start_cell_x, start_cell_y = self.world.grid_to_cell(start_grid_x, start_grid_y)
        end_cell_x, end_cell_y = self.world.grid_to_cell(end_grid_x, end_grid_y)

        print(start_cell_x, start_cell_y)
        print(end_cell_x, end_cell_y)

        self.start = [start_cell_x, start_cell_y]
        self.end = [end_cell_x, end_cell_y]

        # check coords are actually on grid
        if not self.world.on_grid(start_cell_x, start_cell_y) or not self.world.on_grid(end_cell_x, end_cell_y):
                return []
        # if start or end coordinates have obstacle in them, then it's bad input
        if self.world.get_cell(start_cell_x, start_cell_y) or self.world.get_cell(end_cell_x, end_cell_y):
                return []

        path = self.a_star()
        return self.get_path_poses(path)
    
    def diagonal(self, curr_x, curr_y):
        """
        Returns diagonal distance from curr_x,curr_y to self.end[0],self.end[1]--BOTH IN CELL COORDS
        params curr_x, curr_y: the current x,y cell coordinate whose diagonal distance to end we want
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
        open_set = [] # min heap weighted by heuristic/diagonal to goal
        hq.heappush(open_set, (self.diagonal(start.coords[0], start.coords[1]), start))

        closed_set = {} # dict maps curr_node:parent_node for back tracing later

        g_vals = {} # dict maps node to actual cost to get from start to node
        g_vals[start] = 0

        while len(open_set) > 0:
                # get next lowest in open set; 1st index is Cell obj because 0th index is diagonal
                curr = hq.heappop(open_set)[1]

                # we've found goal, back trace and return
                if curr == goal:
                        path = []
                        while curr in closed_set:
                                path.append(curr)
                                curr = closed_set[curr]
                        return path # [goal, ..., c, b, a] reversed so we can pop() later

                # look at all neighbors in y dir
                cx = curr.coords[0]
                cy = curr.coords[1]
                for ny in range(max(0, cy - 1), min(cy + 2, self.world.height)):
                        for nx in range(max(0, cx - 1), min(cx + 2, self.world.width)):
                                # if not current cell and doesn't have obstacle
                                if (cy != ny or cx != nx) and not self.world.get_cell(cx, ny):
                                        n = Cell([nx, ny])
                                        temp_g = g_vals[curr] + EDGE_WEIGHT
                                        # if cost from start to n is lowest so far, or no cost calculated yet
                                        if (n in g_vals and temp_g < g_vals[n]) or (n not in g_vals):
                                                closed_set[n] = curr # update shortest path to n
                                                g_vals[n] = temp_g
                                                temp_f = g_vals[n] + self.diagonal(cx, ny)
                                                for i in range(len(open_set)):
                                                        # remove n from open set if already there
                                                        if open_set[i][1] == n:
                                                                del open_set[i]
                                                                break
                                                hq.heappush(open_set, (temp_f, n))
                                                hq.heapify(open_set)

        return []

    def is_near_obs(self, x, y):
        """
        Check if cell index at x,y is near a rock, to take into account of robot size in map
        """
        near = False
        for

    def get_path_poses(self, path):
        """
        Get poses in map as list of [x, y, yaw] from list of cell coordinates/indexes
        Assumes movement up, down, left, right only
        param path: list of cell indexes in format [x, y]
        """
        # returns 2D list where items are [x, y, yaw] in map frame
        poses = []

        for i in range(len(path)):
                cx = path[i].coords[0]
                cy = path[i].coords[1]

                # get x,y in grid
                grid_x, grid_y = self.world.cell_to_grid(cx, cy)

                # get x,y in map; yaw doesn't matter
                map_x, map_y, map_yaw = self.world.grid_to_map(grid_x, grid_y, 0)
                poses.append([map_x, map_y])
        return poses
