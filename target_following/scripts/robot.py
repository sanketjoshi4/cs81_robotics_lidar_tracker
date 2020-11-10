#!/usr/bin/env python

import numpy as np
import copy

import rospy
from geometry_msgs.msg import Twist
from sensor_msgs.msg import LaserScan

import tf  # library for transformations.

FREQ = 10  # Hz
SLEEP = 2

SCAN_FREQ = 1  # Hz

PI = np.pi
LASER_RANGE = 20.0

BASE_LINEAR_VELOCITY = 1.0
BASE_ANGULAR_VELOCITY = PI / 2


class Robot:
    def __init__(self):
        rospy.init_node("robot")  # feel free to rename
        self.pub = rospy.Publisher("robot_0/cmd_vel", Twist, queue_size=0)

        self.sub_laser = rospy.Subscriber("robot_0/base_scan", LaserScan, self.laser_scan_callback)
        self.id = Identifier()
        self.listener = tf.TransformListener()
        self.pose = {"x": 0, "y": 0, "z": 0}
        self.pose_last_scan = {"x": 0, "y": 0, "z": 0}
        self.time_last_scan = None

        rospy.sleep(SLEEP)

    def main(self):
        # setup code
        vel_msg = Twist()
        rate = rospy.Rate(FREQ)
        while not rospy.is_shutdown():
            vel_msg.linear.x = 0.1
            self.pub.publish(vel_msg)
            self.update_pose()
            rate.sleep()

    def laser_scan_callback(self, laser_scan_msg):
        curr_time = rospy.get_time()
        if self.time_last_scan is None or (curr_time - self.time_last_scan > 1 / SCAN_FREQ):
            self.time_last_scan = curr_time

            # TODO : calculate the movement_transform matrix, which converts points in current frame to as they would be seen in the last frame when scannr ran
            movement_transform = None
            self.pose_last_scan = self.pose

            self.id.blobify(laser_scan_msg, movement_transform)
            self.id.classify(None)

    def update_pose(self):
        # TODO : Publish to map frame, change base_link to map
        (trans, rot) = self.listener.lookupTransform('base_link', 'base_laser_link', rospy.Time(0))
        trans_mat = tf.transformations.translation_matrix(trans)
        rot_mat = tf.transformations.quaternion_matrix(rot)
        pose = trans_mat.dot(rot_mat).dot(np.transpose(np.array([0, 0, 0, 1])))
        self.pose = {"x": pose[0][0], "y": pose[1][0], "z": pose[2][0]}
        print pose
        pass


class Identifier:
    # The minimum change in laser reading between adjacent angles, to consider them to be coming from different objects
    THRESH_RADIALLY_SEPARATE = 2.0

    # The maximum distance a blob's mean can move to imply that it's stationary
    THRESH_BLOB_STATIC = 0.1

    # The maximum distance a blob's mean can move to imply that it's moving
    # This is not set as the minimum since it is difficult to match a blob with it's last scan
    # Hence, we compare with all blobs in the last scan and the ones that have a large difference are probably
    # Different blobs and the ones with a small distance, but greater than THRESH_BLOB_STATIC, are moving blobs
    THRESH_BLOB_MOVEMENT = 0.1

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

    def __init__(self): # All w.r.t. bot
        self.blobs = {}  # dict of blob_id to Blob item
        self.last_blobs = {}  # blobs during last scan
        self.target = None  # (x, y)
        self.last_target = None  # (x, y)
        self.obs = None # list of Blob items
        self.status = None # Amongst

    def blobify(self, laser_scan_msg):

        # TODO : blobs <- laser_msg
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

    def classify(self, movement_transform=None):

        # TODO : target.pos, [obs] <- blobs
        # Pick out moving blobs
        obs = []
        target = None
        for blob_id, blob in self.blobs.items():
            for last_blob_id, last_blob in self.last_blobs.items():
                last_blob_in_curr_frame = None  # TODO : Apply movement_transform to correct for robot movement
                dist = blob.dist(last_blob_in_curr_frame)
                if dist < Identifier.THRESH_BLOB_STATIC:
                    obs.append(blob)
                    continue
                elif dist < Identifier.THRESH_BLOB_MOVEMENT:
                    target = blob
                    continue

        self.obs = obs
        self.target = target
        # print {i: len(v) for i, v in self.blobs.items()}

        # TODO : target.lin_vel <- (target.pos, target.last_pos)
        # TODO : return target{pos,vel}

    def status(self):
        if self.target is None:
            return Identifier.STATUS_ERR
        dist = float(np.sqrt(self.target[0]*self.target[0]+self.target[1]*self.target[1]))
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
        self.arr = None  # [coordinates of indicent laser rays in bot frame]
        self.mean = None  # [mean coordinate in bot frame]

    def add_point(self, point):
        if self.arr is None:
            self.arr = []
        self.arr.append(point)

    def calculate_mean(self):
        if self.arr is not None:
            self.mean = (sum([p[0] for p in self.arr]) / len(self.arr), sum([p[1] for p in self.arr]) / len(self.arr))

    def dist(self, blob2):
        return True


def show(x, n=3):
    """
    This formats a number for better readability in the log, so that they are vertically aligned
        show (1.0, n=3)         = " 001.000"       # Padded on both sides
        show (23.1238, n=3)     = " 023.124"       # Rounded an padded
        show (-1.2, n=3)        = "-001.200"       # Aligns negative numbers with positive
        show (1111.0, n=3)      = " 111.000"       # Capped to 3 on both sides of decimal
    """

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
    r.main()
