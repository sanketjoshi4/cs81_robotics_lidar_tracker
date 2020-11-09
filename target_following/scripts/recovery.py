LOST_THRESH = 10 # s
class Recovery:
	def __init__(self, world):
		self.start = None
		self.end = None
		self.last_known_pos = None
		self.last_known_vel = None
		self.robot_pos = None
		self.elapsed_lost_time = 0
		self.predicted_poses = []
		self.world = world

	def predict(self):
		self.brute_search()

	def brute_search(self):
		self.end = [self.last_known_pos.x, self.last_known_pos.y]
		self.end = [self.robot_pos.x, self.robot_pos.y]
		path = self.a_star()
		return self.get_path_poses(path)
	
	def manhattan(self, curr_x, curr_y):
		"""
		Returns manhattan distance from curr_x,curr_y to self.end[0],self.end[1]--BOTH IN CELL COORDS
		params curr_x, curr_y: the current x,y cell coordinate whose manhattan distance to end we want
		"""
		return abs(self.end[0] - curr_x) + abs(self.end[1] - curr_y)

	def a_star(self):
		"""
		Implements A* algorithm to find path from start to goal
		Assumes movement up, down, left, right only
		Actual cost is always 1 because every next cell should be just 1 cell away from curr cell
		"""
		start = Cell(self.start)
		goal = Cell(self.end)

		# stores (Cell objs, heuristic)
		open_set = [] # min heap weighted by heuristic/manhattan to goal
		hq.heappush(open_set, (self.manhattan(start.coords[0], start.coords[1]), start))

		closed_set = {} # dict maps curr_node:parent_node for back tracing later

		g_vals = {} # dict maps node to actual cost to get from start to node
		g_vals[start] = 0

		while len(open_set) > 0:
			# get next lowest in open set; 1st index is Cell obj because 0th index is manhattan
			curr = hq.heappop(open_set)[1]

			# we've found goal, back trace and return
			if curr == goal:
				path = []
				while curr in closed_set:
					path.append(curr)
					curr = closed_set[curr]
				path.append(curr)
				path.reverse()
				return path # [start, a, b, c, ..., goal]

			# look at all neighbors in y dir
			cx = curr.coords[0]
			cy = curr.coords[1]
			for ny in range(max(0, cy - 1), min(cy + 2, self.map.height)):
				# if not current cell and doesn't have obstacle
				if cy != ny and not self.map.get_cell(cx, ny):
					n = Cell([cx, ny])
					temp_g = g_vals[curr] + EDGE_WEIGHT
					# if cost from start to n is lowest so far, or no cost calculated yet
					if (n in g_vals and temp_g < g_vals[n]) or (n not in g_vals):
						closed_set[n] = curr # update shortest path to n
						g_vals[n] = temp_g
						temp_f = g_vals[n] + self.manhattan(cx, ny)
						for i in range(len(open_set)):
							# remove n from open set if already there
							if open_set[i][1] == n:
								del open_set[i]
								break
						hq.heappush(open_set, (temp_f, n))
						hq.heapify(open_set)

			# look at all neighbors in x dir; similar as for y dir neighbors above
			for nx in range(max(0, cx - 1), min(cx + 2, self.map.width)):
				if cx != nx and not self.map.get_cell(nx, cy):
					n = Cell([nx, cy])
					temp_g = g_vals[curr] + EDGE_WEIGHT
					if (n in g_vals and temp_g < g_vals[n]) or (n not in g_vals):
						closed_set[n] = curr
						g_vals[n] = temp_g
						temp_f = g_vals[n] + self.manhattan(nx, cy)
						for i in range(len(open_set)):
							if open_set[i][1] == n:
								del open_set[i]
								break
						hq.heappush(open_set, (temp_f, n))
						hq.heapify(open_set)

		return None

	def get_path_poses(self, path):
		"""
		Get poses in map as list of [x, y, yaw] from list of cell coordinates/indexes
		Assumes movement up, down, left, right only
		param path: list of cell indexes in format [x, y]
		"""
		# returns 2D list where items are [x, y, yaw] in map frame
		poses = []

		for i in range(len(path)):
			yaw = 0 # if last item, set yaw to 0 i.e. facing positive x in grid
			cx = path[i].coords[0]
			cy = path[i].coords[1]
			# determine yaw based on next cell's direction relative to curr cell
			if i < len(path) - 1:
				nx = path[i+1].coords[0]
				ny = path[i+1].coords[1]
				if cx == nx:
					if ny > cy:
						yaw = math.pi / 2 # 90 deg ccw to look north
					elif ny < cy:
						yaw = -math.pi / 2 # 90 deg cw to look south
				elif cy == ny:
					if nx > cx:
						yaw = 0 # 0 deg to face +ve x axis
					elif nx < cx:
						yaw = math.pi # 180 deg ccw (or cw would work too) to face -ve x axis

			# get x,y in grid
			grid_x, grid_y = self.map.cell_to_grid(cx, cy)

			# get x,y,yaw in map
			map_x, map_y, map_yaw = self.map.grid_to_map(grid_x, grid_y, yaw)
			poses.append([map_x, map_y, map_yaw])
		return poses
