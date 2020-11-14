#!/usr/bin/env python

import numpy as np

import follower_utils
from recovery import Recovery
from identifier import Identifier
from world import World

import rospy
import tf

from geometry_msgs.msg import Twist, Point
from nav_msgs.msg import OccupancyGrid, Odometry
from std_msgs.msg import Bool
from sensor_msgs.msg import LaserScan

FREQ = 10  # Hz
SLEEP = 2 # secs
VEL = 0.1  # m/s

SCAN_FREQ = 1  # Hz

PI = np.pi
LASER_RANGE = 20.0

BASE_LINEAR_VELOCITY = 1.0
BASE_ANGULAR_VELOCITY = PI / 2

START_X_MAP = 3.0  # Would change as per the map
START_Y_MAP = 5.0  # Would change as per the map

# START_Z_MAP = 0.0

class Robot:

    def __init__(self):
        rospy.init_node("robot")  # feel free to rename
        self.pub = rospy.Publisher("robot_0/cmd_vel", Twist, queue_size=0)
        self.stat_pub = rospy.Publisher("visible_status", Bool, queue_size=0)  # latest one only
        self.world_sub = rospy.Subscriber("map", OccupancyGrid, self.world_callback, queue_size=1)
        self.odom_sub = rospy.Subscriber("robot_0/odom", Odometry, self.odom_callback)
        self.sub_laser = rospy.Subscriber("robot_0/base_scan", LaserScan, self.laser_scan_callback, queue_size=1)
        self.map = None

        # continually updated info about robot's pose wrt odom
        self.posx = None
        self.posy = None
        self.angle = None
        self.last_posx = None
        self.last_posy = None
        self.last_angle = None

        # useful transformation matrices for movement
        self.mTo = np.array(
            [[-1, 0, 1, START_X_MAP], [0, -1, 0, START_Y_MAP], [0, 0, 1, 0], [0, 0, 0, 1]])  # odom to map
        self.bTo = None  # odom to base_link
        self.world = None  # if we do not use world in here, delete this later

	self.rcvr_poses = [] # all poses to move to in order to get to last detected target pose

        self.lis = tf.TransformListener()
        self.rcvr = None
        self.id = Identifier()
        self.time_last_scan = None

        rospy.sleep(SLEEP)

    def odom_callback(self, msg):
        # getting all of the odom information on the current pose of the robot
        self.posx = msg.pose.pose.position.x
        self.posy = msg.pose.pose.position.y
        # from https://answers.ros.org/question/11545/plotprint-rpy-from-quaternion/#17106
        (roll, pitch, yaw) = tf.transformations.euler_from_quaternion(
            [msg.pose.pose.orientation.x, msg.pose.pose.orientation.y, msg.pose.pose.orientation.z,
             msg.pose.pose.orientation.w])
        self.angle = yaw
        # TEST CODE for visible_status pub

    def pub_visibility(self, visible):
        msg = Bool()
        msg.data = visible  # expect boolean
        self.stat_pub.publish(msg)

    def world_callback(self, msg):
        print("loading map")
        self.world = World(msg.data, msg.info.width, msg.info.height, msg.info.resolution, msg.header.frame_id,
                           msg.info.origin)
        self.rcvr = Recovery(self.world)

    def get_transform(self):
        # get transformation from odom to base_link
        (trans, rot) = self.lis.lookupTransform('robot_0/base_link', 'robot_0/odom', rospy.Time(0))
        # get everything in regular matrix form
        t = tf.transformations.translation_matrix(trans)
        r = tf.transformations.quaternion_matrix(rot)
        self.bTo = t.dot(r)

    def map_callback(self, msg):
        print("loading map")
        self.map = World(msg.data, msg.info.width, msg.info.height, msg.info.resolution, msg.header.frame_id,
                         msg.info.origin)
        self.rcvr = Recovery(self.map)
        print(self.map.T)

    def laser_scan_callback(self, laser_scan_msg):

        curr_time = rospy.get_time()
        if self.time_last_scan is None or (curr_time - self.time_last_scan > 1 / SCAN_FREQ):
            print "{}".format(''.join(['-' for _ in range(100)]))

            # print "Pose: ({}, {}, {})".format(show(self.posx), show(self.posy), show(self.angle))
            # print "LPos: ({}, {}, {})".format(show(self.last_posx), show(self.last_posy), show(self.last_angle))
            if self.posx is not None and self.last_posx is not None:
                self.id.blobify(laser_scan_msg)
                self.id.classify(self.get_movement_transform())

            self.last_posx = self.posx
            self.last_posy = self.posy
            self.last_angle = self.angle
            self.time_last_scan = curr_time

            target_pos = self.get_target_pos(robot_posx=self.posx, robot_posy=self.posy, robot_angle=self.angle,
                                                trans_odom_to_map=self.mTo, frame="ODOM")
            # print "Robot  @ ({}, {})".format(show(self.posx), show(self.posy))
            if target_pos is not None:
                print "Target @ ({},{})".format(follower_utils.show(target_pos[0]), follower_utils.show(target_pos[1]))
            else:
                print "TARGET: Not found"

    def update_rcvr(self):
	# update recovery with robot's current pose
	self.get_transform() # update self.bTo first
	p = np.linalg.inv(self.bTo).dot(np.transpose(np.array([0, 0, 0, 1])))
        p = self.mTo.dot(p)[0:2]
	self.rcvr.robot_pos = Point()
        self.rcvr.robot_pos.x = p[0]
        self.rcvr.robot_pos.y = p[1]


    def move(self):
	rate = rospy.Rate(FREQ)
	vel_msg = Twist()

	while not rospy.is_shutdown():
		lin_x = 0
		ang_z = 0

		# we detect target so decide how to move using PID-like function
		if self.id.get_target_pos(robot_posx=self.posx, robot_posy=self.posy, robot_angle=self.angle,
                                                trans_odom_to_map=self.mTo, frame="ODOM") is not None:
			(lin_x, ang_z) = self.chase_target()

			# recovery object will always have last known target pose to prepare for recovery mode
			target_x, target_y = self.id.get_target_pos(robot_posx=self.posx, robot_posy=self.posy, robot_angle=self.angle,
                                                trans_odom_to_map=self.mTo, frame="MAP")
			self.rcvr.last_known_pos = Point()
			self.rcvr.last_known_pos.x = target_x
			self.rcvr.last_known_pos.y = target_y

			# clear poses for recovery when re-entering regular mode
			if self.rcvr_poses:
				self.rcvr_poses = []
		else: # target is out of sight, go into recovery mode
			# just entering recovery mode from regular mode
			if not self.rcvr_poses:
				# we delete as we go and clear when switch state so should be empty upon switch to RECOVERY
				self.update_rcvr() # remember to update Recovery object's required info first
				self.rcvr_poses = self.rcvr.recover()

			# in the middle of recovery mode
			# separate if statement so we don't have to wait until next loop iteration to start moving once entered recovery mode
			if self.rcvr_poses:
				# essentially we are moving to every position from a list that goes [[goalx, goaly], ..., [startx, starty]], if we encounter
				# target before we finish this list i.e. state changes back to REGULAR, just clear list to prep for next recovery call
				pose = self.rcvr_poses.pop()

				# transform user-given point in odom to base_link, assume ROBOT CAN'T FLY
				self.get_transform()
				v = np.linalg.inv(self.mTo).dot(np.transpose(np.array([pose[0], pose[1], 0, 1])))
				v = self.bTo.dot(v)
				v = v[0:2]  # only need x,y because of assumption above
				# angle to turn i.e. angle btwn x-axis vector and vector of x,y above
				ang_z = np.arctan2(v[1], v[0])
				# euclidean distance to travel, assume no movement in z-axis
				lin_x = np.linalg.norm(np.array([0, 0]) - v)
		
		vel_msg.linear.x = lin_x
		vel_msg.angular.z = ang_z
		self.pub.publish(vel_msg)
		rate.sleep()


    def get_movement_transform(self):

        sz1 = np.sin(self.angle)
        cz1 = np.cos(self.angle)
        mat1 = np.matrix([[cz1, -sz1, 0, self.posx], [sz1, cz1, 0, self.posy], [0, 0, 1, 0], [0, 0, 0, 1]])

        sz2 = np.sin(self.last_angle)
        cz2 = np.cos(self.last_angle)
        mat2 = np.matrix([[cz2, -sz2, 0, self.last_posx], [sz2, cz2, 0, self.last_posy], [0, 0, 1, 0], [0, 0, 0, 1]])

        return mat2.getI().dot(mat1)

    def simple_main(self):
        print(self.posx, self.posy, self.angle)
        vel_msg = Twist()
        rate = rospy.Rate(FREQ)
        start_time = rospy.get_rostime()

        while not rospy.is_shutdown() and rospy.get_rostime() - start_time < rospy.Duration(100):
            vel_msg.angular.z = 0
            vel_msg.linear.x = VEL
            self.pub.publish(vel_msg)
            rate.sleep()

    def get_target_pos(self, frame):

        if self.id.target is None:
            return None

        sz, cz = np.sin(self.angle), np.cos(self.angle)

        x_base_scan, y_base_scan = self.id.target.mean[0], self.id.target.mean[1]
        pos_base_scan = np.array([[x_base_scan], [y_base_scan], [0], [1]])
        if frame == "BASE":
            return float(pos_base_scan[0][0]), float(pos_base_scan[1][0])
        # print "BASE : {}".format(follower_utils.show_pos(pos_base_scan))

        trans_base_scan_to_odom = np.matrix(
            [[cz, -sz, 0, self.posx], [sz, cz, 0, self.posy], [0, 0, 1, 0], [0, 0, 0, 1]])
        pos_odom = trans_base_scan_to_odom.dot(pos_base_scan)
        if frame == "ODOM":
            return float(pos_odom[0][0]), float(pos_odom[1][0])
        # print "ODOM : {}".format(follower_utils.show_pos(pos_odom))

        trans_odom_to_map = self.mTo
        pos_map = trans_odom_to_map.dot(pos_odom)
        if frame == "MAP":
            return float(pos_map[0][0]), float(pos_map[1][0])
        # print "MAP  : {}".format(follower_utils.show_pos(pos_map))

        return None


if __name__ == "__main__":
    # we'll probably set up target like this from main.py?
    r = Robot()
    r.move()
