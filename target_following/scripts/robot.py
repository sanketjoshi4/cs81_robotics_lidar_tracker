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
SLEEP = 2
VEL = 0.1  # m/s

SCAN_FREQ = 1  # Hz
PI = np.pi

# TODO : Figure out a better way to code robot's start pose .. env vars?
START_X_MAP = 3.0  # Would change as per the map
START_Y_MAP = 5.0  # Would change as per the map
START_Z_MAP = 0.0


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
        """ Uses laser scan to update target position """

        curr_time = rospy.get_time()
        if self.time_last_scan is None or (curr_time - self.time_last_scan > 1 / SCAN_FREQ):
            # Scan based on SCAN_FREQ
            # print "{}".format(''.join(['-' for _ in range(100)]))

            if self.posx is not None and self.last_posx is not None:
                # Identify blobs
                self.id.blobify(laser_scan_msg)
                # Classify blobs, set target
                self.id.classify(self.get_movement_transform())

            # Update last pose to current
            self.last_posx = self.posx
            self.last_posy = self.posy
            self.last_angle = self.angle
            self.time_last_scan = curr_time

            # TODO : Remove, debug only
            target_pos = self.id.get_target_pos(robot_posx=self.posx, robot_posy=self.posy, robot_angle=self.angle,
                                                trans_odom_to_map=self.mTo, frame="ODOM")
            if target_pos is not None:
                print "Target @ ({},{})".format(follower_utils.show(target_pos[0]), follower_utils.show(target_pos[1]))
            else:
                print "TARGET @ ???"

    def move(self):
        # TODO: add actual logic to this function
        # TODO: get predicted x and z velocities from Predictor, combine with Identifier info to calculate move

        pass

    def get_movement_transform(self):

        sz1 = np.sin(self.angle)
        cz1 = np.cos(self.angle)
        mat1 = np.matrix([[cz1, -sz1, 0, self.posx], [sz1, cz1, 0, self.posy], [0, 0, 1, 0], [0, 0, 0, 1]])

        sz2 = np.sin(self.last_angle)
        cz2 = np.cos(self.last_angle)
        mat2 = np.matrix([[cz2, -sz2, 0, self.last_posx], [sz2, cz2, 0, self.last_posy], [0, 0, 1, 0], [0, 0, 0, 1]])

        return mat2.getI().dot(mat1)

    def id_test(self):
        """ A simple test to check if target is identified, robot moves in straight line """

        vel_msg = Twist()
        rate = rospy.Rate(FREQ)
        start_time = rospy.get_rostime()

        while not rospy.is_shutdown() and rospy.get_rostime() - start_time < rospy.Duration(100):
            vel_msg.angular.z = 0
            vel_msg.linear.x = VEL
            self.pub.publish(vel_msg)
            rate.sleep()

    def main(self):
        print("in main")
        while self.rcvr is None:
            continue
        self.rcvr.robot_pos = Point()
        p = self.mTo.dot(np.transpose(np.array([0, 0, 0, 1])))[0:2]
        self.rcvr.robot_pos.x = p[0]
        self.rcvr.robot_pos.y = p[1]
        print(p)
        print('---')
        print(self.posx, self.posy, self.angle)
        vel_msg = Twist()
        rate = rospy.Rate(FREQ)
        poses = self.rcvr.predict()  # expect [[x,y],[x,y],...]
        print(poses)
        for pose in poses:
            # transform user-given point in odom to base_link, assume ROBOT CAN'T FLY
            self.get_transform()
            v = np.linalg.inv(self.mTo).dot(np.transpose(np.array([pose[0], pose[1], 0, 1])))
            v = self.bTo.dot(v)
            v = v[0:2]  # only need x,y because of assumption above
            print(v)
            # angle to turn i.e. angle btwn x-axis vector and vector of x,y above
            a = np.arctan2(v[1], v[0])
            # euclidean distance to travel, assume no movement in z-axis
            l = np.linalg.norm(np.array([0, 0]) - v)
            start_time = rospy.get_rostime()
            if a >= 0:  # anticlockwise rot (or no rot)
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
        print(self.posx, self.posy, self.angle)
        self.pub_visibility(True)
        self.pub_visibility(False)


if __name__ == "__main__":
    # we'll probably set up target like this from main.py?
    r = Robot()
    r.id_test()
