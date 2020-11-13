#!/usr/bin/env python

import copy

import rospy
import tf
import numpy as np
from geometry_msgs.msg import Twist, Point
from nav_msgs.msg import OccupancyGrid, Odometry
from recovery import Recovery
from world import World
from sensor_msgs.msg import LaserScan

FREQ = 10  # Hz
SLEEP = 2
VEL = 0.1  # m/s

SCAN_FREQ = 1  # Hz

PI = np.pi
LASER_RANGE = 20.0

BASE_LINEAR_VELOCITY = 1.0
BASE_ANGULAR_VELOCITY = PI / 2

START_X_MAP = 3.0
START_Y_MAP = 5.0


# START_Z_MAP = 0.0

class Robot:
    def __init__(self):
        rospy.init_node("robot")  # feel free to rename
        self.pub = rospy.Publisher("robot_0/cmd_vel", Twist, queue_size=0)
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

    def world_callback(self, msg):
        print("loading map")
        self.world = World(msg.data, msg.info.width, msg.info.height, msg.info.resolution, msg.header.frame_id,
                           msg.info.origin)
        self.rcvr = Recovery(self.world)

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

            target_pos = self.get_target_pos(frame="ODOM")
            # print "Robot  @ ({}, {})".format(show(self.posx), show(self.posy))
            if target_pos is not None:
                print "Target @ ({},{})".format(show(target_pos[0]), show(target_pos[1]))
            else:
                print "TARGET: Not found"

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
        # print "BASE : {}".format(show_pos(pos_base_scan))

        trans_base_scan_to_odom = np.matrix(
            [[cz, -sz, 0, self.posx], [sz, cz, 0, self.posy], [0, 0, 1, 0], [0, 0, 0, 1]])
        pos_odom = trans_base_scan_to_odom.dot(pos_base_scan)
        if frame == "ODOM":
            return float(pos_odom[0][0]), float(pos_odom[1][0])
        # print "ODOM : {}".format(show_pos(pos_odom))

        trans_odom_to_map = self.mTo
        pos_map = trans_odom_to_map.dot(pos_odom)
        if frame == "MAP":
            return float(pos_map[0][0]), float(pos_map[1][0])
        # print "MAP  : {}".format(show_pos(pos_map))

        return None


def show_pos(np_pos):
    return show(float(np_pos[0][0])), show(float(np_pos[1][0]))


class Identifier:
    # The minimum change in laser reading between adjacent angles, to consider them to be coming from different objects
    THRESH_RADIALLY_SEPARATE = 0.5

    # The maximum distance a blob's mean can move to imply that it's stationary
    THRESH_BLOB_STATIC = 0.5

    # The maximum distance a blob's mean can move to imply that it's moving
    # This is not set as the minimum since it is difficult to match a blob with it's last scan
    # Hence, we compare with all blobs in the last scan and the ones that have a large difference are probably
    # Different blobs and the ones with a small distance, but greater than THRESH_BLOB_STATIC, are moving blobs
    THRESH_BLOB_MOVEMENT = 1.5

    # Below are the statuses based on distance from target
    STATUS_CLOSE = 1
    STATUS_OK = 2
    STATUS_FAR = 3
    STATUS_OOR = 4
    STATUS_OOS = 5
    STATUS_ERR = -1

    # Below are the maximum distances of different statuses
    DIST_MAX_CLOSE = 2
    DIST_MAX_OK = 10
    DIST_MAX_FAR = 20

    def __init__(self):  # All in the robot's frame of reference
        self.blobs = {}  # dict of blob_id to Blob item
        self.last_blobs = {}  # blobs during last scan
        self.target = None  # (x, y)
        self.last_target = None  # (x, y)
        self.obs = None  # list of Blob items
        self.status = None  # Amongst

    def blobify(self, laser_scan_msg):

        amin = laser_scan_msg.angle_min  # Minimum angle of overall scan
        incr = laser_scan_msg.angle_increment  # Angle increment of overall scan
        arr = laser_scan_msg.ranges  # List of readings

        self.last_blobs = copy.deepcopy(self.blobs)
        self.blobs = {}

        blob_id = 0

        # Create blobs
        for idx, dist in enumerate(arr):
            angle = amin + incr * idx
            incident = (dist * np.cos(angle), dist * np.sin(angle))

            if idx == 0 or abs(dist - arr[idx - 1]) > Identifier.THRESH_RADIALLY_SEPARATE:
                blob_id += 1
                self.blobs[blob_id] = Blob(blob_id)

            self.blobs[blob_id].add_point(incident)

        # Calculate blob means
        for blob_id, blob in self.blobs.items():
            blob.calculate_mean()

        # Print blobs
        # print [b.show() for b in self.blobs.values()]

    def classify(self, movement_transform):

        # Pick out moving blobs
        obs = []
        target = None
        shift_min_target = 100

        # print "\nBLOBS"
        # print [b.show() for b in self.blobs.values()]
        # print "\nLAST_BLOBS"
        # print [b.show() for b in self.last_blobs.values()]

        for last_blob_id, last_blob in self.last_blobs.items():

            last_mean_now = movement_transform.getI().dot(
                np.array([[last_blob.mean[0]], [last_blob.mean[1]], [0], [1]])
            )

            for blob_id, blob in self.blobs.items():

                # TODO : invert mov trans?
                # if last_blob_id == blob_id:
                #     print "\nBLOB_IDs : {}, {}".format(last_blob_id, blob_id)
                #     print "Last : ({}, {})".format(show(last_mean_now[0][0]), show(last_mean_now[1][0]))
                #     print "Blob : ({}, {})".format(show(blob.mean[0]), show(blob.mean[1]))

                dx = last_mean_now[0][0] - blob.mean[0]
                dy = last_mean_now[1][0] - blob.mean[1]
                shift = float(np.sqrt(dx * dx + dy * dy))

                if shift < Identifier.THRESH_BLOB_STATIC:

                    # print "STATIC"
                    obs.append(blob)
                    # print "obs id'd : {}".format(blob.show())
                    continue

                elif shift < Identifier.THRESH_BLOB_MOVEMENT:

                    if shift < shift_min_target:
                        target = blob
                        shift_min_target = shift
                        # print "MOVE-NEW"
                #     else:
                #         print "MOVE-OLD"
                # else:
                #     print "NO MATCH"

        self.obs = obs
        self.target = target  # in base_scan ref
        # TODO : target.lin_vel <- (target.pos, target.last_pos)

    def status(self):
        if self.target is None:
            return Identifier.STATUS_ERR
        dist = float(np.sqrt(self.target[0] * self.target[0] + self.target[1] * self.target[1]))
        if dist < Identifier.DIST_MAX_CLOSE:
            return Identifier.STATUS_CLOSE
        if dist < Identifier.DIST_MAX_OK:
            return Identifier.STATUS_OK
        if dist < Identifier.DIST_MAX_FAR:
            return Identifier.STATUS_FAR
        return Identifier.STATUS_OOR
        # TODO : Distinguish between OOS and OOR


class Blob:
    def __init__(self, id):
        self.id = id
        self.arr = None  # [coordinates of incident laser rays in bot frame]
        self.mean = None  # [mean coordinate in bot frame]

    def add_point(self, point):
        if self.arr is None:
            self.arr = []
        self.arr.append(point)

    def calculate_mean(self):
        if self.arr is not None:
            self.mean = (
                sum([p[0] for p in self.arr]) / len(self.arr), sum([p[1] for p in self.arr]) / len(self.arr))

    def dist(self, blob2):
        dx = self.mean[0] - blob2.mean[0]
        dy = self.mean[1] - blob2.mean[1]
        return float(np.sqrt(dx * dx + dy * dy))

    def show(self):
        return "{}:[{}]@({},{})".format(self.id, len(self.arr), show(self.mean[0], 1), show(self.mean[1], 1))


def show(x, n=2):
    """
    This formats a number for better readability in the log, so that they are vertically aligned
        show (1.0, n=3)         = " 001.000"       # Padded on both sides
        show (23.1238, n=3)     = " 023.124"       # Rounded an padded
        show (-1.2, n=3)        = "-001.200"       # Aligns negative numbers with positive
        show (1111.0, n=3)      = " 111.000"       # Capped to 3 on both sides of decimal
    """
    if x is None:
        return ""

    neg = x < 0
    x = round(abs(x), n)
    xstr = str(x)
    frac = '.' in xstr
    splits = xstr.split(".")
    pad0 = "".join(["0" for _ in range(n)])
    intpart = (pad0 + splits[0])[-n:]
    fracpart = pad0 if not frac else (splits[1] + pad0)[:n]
    return ("-" if neg else " ") + intpart + "." + fracpart


if __name__ == "__main__":
    # we'll probably set up target like this from main.py?
    r = Robot()
    r.simple_main()
