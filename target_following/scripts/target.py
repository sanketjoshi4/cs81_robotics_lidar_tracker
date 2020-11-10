#!/usr/bin/env python
import rospy
from geometry_msgs.msg import Twist
from world import World
from nav_msgs.msg import Odometry

# TARGET.PY 
# class for the target object 

FREQ = 10 # Hz
SLEEP = 2
pi = 3.14

class Target:
	def __init__(self):
		rospy.init_node("target") # target node 
		self.pub = rospy.Publisher("robot_1/cmd_vel", Twist, queue_size=0)
		self.sub = rospy.Subscriber("robot_1/odom", Odometry, self.odom_callback)
		rospy.sleep(SLEEP)

		self.posx = 0 # initalizing target x position at 0
		self.posy = 0 # initializing target y position at 0
		self.linxvel = 0 # initializing linear velocity at 0 
		self.angzvel = 0 # initializing angular velocity at 0 
		self.angle = 0 # current pose angle
		
		self.vel_msg = Twist() # creating inital publish message, which is altered in the main

		self.world = world
		self.target_coordpath = []
		self.target_path = []

	# from Archita pa1, modified slightly
	def odom_callback(self, odom_message):
		# getting all of the odom information on the current pose of the robot
		self.posx = odom_message.pose.pose.position.x
		self.posy = odom_message.pose.pose.position.y
		# from https://answers.ros.org/question/11545/plotprint-rpy-from-quaternion/#17106
		(roll, pitch, yaw) = tf.transformations.euler_from_quaternion([odom_message.pose.pose.orientation.x, odom_message.pose.pose.orientation.y, odom_message.pose.pose.orientation.z, odom_message.pose.pose.orientation.w])
		self.angle = yaw

	# function to rotate
	def rotate(self):
		self.linxvel = 0
		self.angzvel = 0.1

	# move forward
	def forward(self):
		self.linxvel = 0.1
		self.angzvel = 0

	# move function that dictates next target velocities and locations without knowing map immediately before
	def move(self):

		# needs to move up 2m 
		if -0.3 < self.posx and self.posx < 0.3 and self.posy < 2:
			# at correct angle of 90 to odom, just move up
			if pi/2 - 0.2  < self.angle and self.angle < pi/2 + 0.2:
				self.rotate()
			else:
				self.forward()	
			return

		# move right 3.5m
		if self.posx < 3.5 and 1.7 < self.posy and self.posy < 2.3:
			# at correct angle of 0, just move up
			if - 0.2  < self.angle and self.angle < 0.2:
				self.rotate()
			else:
				self.forward()	
			return

		# move up 2.5m 
		if 3.2 < self.posx and self.posx < 3.8  and self.posy < 4.5:
			# at correct angle of 90 to odom, just move up
			if pi/2 - 0.2  < self.angle and self.angle < pi/2 + 0.2:
				self.rotate()
			else:
				self.forward()	
			return

		# move right 4m
		if self.posx < 7.5 and 4.2 < self.posy and self.posy < 4.8:
			# at correct angle of 0, just move up
			if - 0.2  < self.angle and self.angle < 0.2:
				self.rotate()
			else:
				self.forward()	
			return
	
		# move down 4.5m 
		if 7.2 < self.posx and self.posx < 7.8  and self.posy > 0:
			# at correct angle of -90 to odom, just move up
			if -pi/2 - 0.2  < self.angle and self.angle < -pi/2 + 0.2:
				self.rotate()
			else:
				self.forward()	
			return
	
		# move right 1m
		if self.posx < 8.5 and -0.2 < self.posy and self.posy < 0.2:
			# at correct angle of 0, just move up
			if - 0.2  < self.angle and self.angle < 0.2:
				self.rotate()
			else:
				self.forward()	
			return

		# move up 8.5 m
		if 8.2 < self.posx and self.posx < 8.8  and self.posy < 8.5:
			# at correct angle of 90 to odom, just move up
			if pi/2 - 0.2  < self.angle and self.angle < pi/2 + 0.2:
				self.rotate()
			else:
				self.forward()	
			return

		# move left 8.5m
		if self.posx > 0 and 8.2 < self.posy and self.posy < 8.8:
			# at correct angle of 180, just move up
			if pi - 0.2  < self.angle and self.angle < pi + 0.2:
				self.rotate()
			else:
				self.forward()	
			return
		

	############ LATER ############	

	# move_fancy function that dictates next target velocities and locations without knowing map immediately before
	def move_fancy(self):
		# calling breadth first search to create the path in target_coordpath
		self.bfs()
		self.convert_path
		continue

	# breadth first search to find the path; copied from pa3 Archita
	def bfs(self):
		# creating new node for the start of the robot; starting from left bottom corner to top right
		firstnode = Node(-90, -90, self.world, 90, 90)

		# putting element into the frontier
		frontier = [firstnode]
		infrontier = {}
		infrontier[(firstnode.x, firstnode.y)] = 1
		
		# keeping track of nodes that have be n visited
		visited = {}
		# using https://docs.python.org/3/tutorial/datastructures.html
		# while the frontier has more nodes to travel
		while len(frontier)>0:
			# removing the first item in the list 
			node = frontier.pop(0)
			
			# if the node is the goal
			if node.x==90 and node.y==90
				while node!=None:
					# inserting the tuple of points into the goal at the start
					self.target_coordpath.insert(0, (node.x, node.y))
					# making the node the previous node
					node = node.prev
				return

			# if there was no goal, continuing
			# adding the cell into the dictionary with x -> y 
			visited[(node.x, node.y)] = 1
	
			# help with dictionaries https://docs.python.org/3/tutorial/datastructures.html
			# left and right nodes added to frontier
			i = 0
			while i < 4:
				newx = node.x - 1 + i
				#creating right node, if no obstacle at the node
				if self.world.get_cell(newx, node.y)<=65:
					# if not inside infrontier or visited
					if (newx, node.y) not in infrontier and (newx, node.y) not in visited:
						# creating the node 
						newnode = Node(newx, node.y, self.world, 90, 90)
						# setting the previous at this node
						newnode.set_prev(node)
						# adding to frontier
						frontier.append(newnode)
						infrontier[(newnode.x, newnode.y)] = 1
				i += 2

			# top and bottom nodes added to frontier
			j = 0
			while j < 4:
				newy = node.y - 1 + j
				#creating right node, if no obstacle at the node
				if self.world.get_cell(node.x, newy)<=65:
					# if not inside infrontier or visited
					if (node.x, newy) not in infrontier and (node.x, newy) not in visited:
						# creating the node 
						newnode = Node(node.x, newy, self.world, 90, 90)
						# setting the previous at this node
						newnode.set_prev(node)
						# adding to frontier
						frontier.append(newnode)
						infrontier[(newnode.x, newnode.y)] = 1
				j += 2

		# did not find a path, terminate program
		print "No path to goal with bfs" 

	def convert_path(self):
		# for each value that is represented in cells
		for value in self.target_coordpath:
			# converting to the grid
			newx, newy = self.world.cell_to_grid(value[0], value[1])
			# converting to the map
			x, y, theta = self.world.grid_to_map(newx, newy)
			pose = (x, y, theta)
			# inserting into path, now represented in map frame
			self.target_path.insert(0, pose)

	############ END OF LATER ############
		
	def main(self):

		# setup code
		rate = rospy.Rate(FREQ)

		while not rospy.is_shutdown():
			self.move() # calling the move function each iteration

			# altering the message to publish the linear and angular velocity decided
			self.vel_msg.linear.x = self.linxvel
			self.vel_msg.angular.z = self.angzvel
			self.pub.publish(self.vel_msg)
			rate.sleep()

# class for the points on grid, copied from pa3 Archita
class Node:
	# initializing basic values of the node
	def __init__(self, x, y, grid, goalx, goaly):
		self.value = grid.get_cell(x, y)
		self.x = x
		self.y = y
		self.prev = None
	
	# setting the previous node
	def set_prev(self, other_node):
		self.prev = other_node

if __name__ == "__main__":
	# we'll probably set up target like this from main.py?
	t = Target()
	t.main()
