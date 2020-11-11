#!/usr/bin/env python
import rospy
from geometry_msgs.msg import Twist
from world_project import World
from nav_msgs.msg import Odometry
import tf

# TARGET.PY 
# class for the target object 

FREQ = 10 # Hz
SLEEP = 2
pi = 3.14
LINVELOCITY = 0.2
ANGVELOCITY = 0.2

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
		
		self.trans_msg = Twist()
		self.rot_msg = Twist() # creating inital publish message, which is altered in the main
		
		# creating a subscriber for the map
		#self.map = None
		#self.subscriber = rospy.Subscriber("map", OccupancyGrid, self.map_callback, queue_size=1)
		self.points = [(0, 2), (3.5, 2), (3.5, 4.2), (7.2, 4.2), (7.2, 0), (8.3, 0), (8.3, 8.2), (0, 7.9)]
		self.goalangles = [pi/2, 0, pi/2, 0, -pi/2, 0, pi/2, -pi+0.02]
		self.goalangle = self.goalangles[0]
		self.pointnum = 0
		self.goalx = self.points[0][0]
		self.goaly = self.points[0][1]
		
		self.is_lost = 0 #1 if the robot is lost from the target

		rospy.sleep(2)
		
	# from Archita pa1, modified slightly
	def odom_callback(self, odom_message):
		# getting all of the odom information on the current pose of the robot
		self.posx = odom_message.pose.pose.position.x
		self.posy = odom_message.pose.pose.position.y
		# from https://answers.ros.org/question/11545/plotprint-rpy-from-quaternion/#17106
		(roll, pitch, yaw) = tf.transformations.euler_from_quaternion([odom_message.pose.pose.orientation.x, odom_message.pose.pose.orientation.y, odom_message.pose.pose.orientation.z, odom_message.pose.pose.orientation.w])
		self.angle = yaw	
	
	# changing the message to simply rotate the robot
	def rotate(self): 
		self.rot_msg.angular.z = ANGVELOCITY
	
    # changing the message to stop rotating the robot
	def stop_rotate(self): 
		self.rot_msg.angular.z = 0
	
    # changing the message to translate the robot
	def translate(self):
		self.trans_msg.linear.x = LINVELOCITY

    # changing the message to stop translating the robot
	def stop_translate(self):
		self.trans_msg.linear.x = 0

	# checking if the goal angle is equal to the current angle and returning a boolean
	def check_angle(self):
		if -0.02 <= self.angle - self.goalangle <= 0.02:
			return 1
		else: 
			return 0
    
    # verifying that the robot is at the correxct x 
	def check_x(self):
		if -0.1 <= self.posx - self.goalx <= 0.1:
			return 1
		else: 
			return 0

    # verifying that the robot is at the correct y 
	def check_y(self):
		if -0.1 <= self.posy - self.goaly <= 0.1:
			return 1
		else: 
			return 0

	 #updating which points are the goal points
	def update_goal(self):
		# incrementing the number of the point we are on
		self.pointnum = self.pointnum + 1
		# decrementing if is lost
		if self.is_lost==1:
			self.pointnum = self.pointnum-2
		# using the next point's information as the goal
		self.goalx = self.points[self.pointnum][0]
		self.goaly = self.points[self.pointnum][1]
		self.goalangle = self.goalangles[self.pointnum]
		# goal angle is the opposite
		if self.is_lost==1:
			self.goalangle = -1*self.goalangle

	def move(self):
		# checking the angle
		anglecheck = self.check_angle()
		if anglecheck==0:
			# rotate if not correct angle
			self.rotate()
			self.pub.publish(self.rot_msg)
		# if correct angle, check position
		else:
			xcheck = self.check_x()
			ycheck = self.check_y()
			# translate if not correct position
			if xcheck==0 or ycheck==0: 
				self.stop_rotate()
				self.pub.publish(self.rot_msg)
				self.translate()
				self.pub.publish(self.trans_msg)
			else:
				self.stop_translate()
				self.pub.publish(self.trans_msg)
				self.update_goal()

		
	def main(self):

		# setup code
		rate = rospy.Rate(FREQ)

		while not rospy.is_shutdown():
			self.move()



# class for the points on grid, copied from pa3 Archita
class Node:
	# initializing basic values of the node
	def __init__(self, x, y, grid, goalx, goaly):
		#self.value = grid.get_cell(x, y)
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
