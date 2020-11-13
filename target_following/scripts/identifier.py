#!/usr/bin/env python

import copy
import numpy as np

import follower_utils


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
        self.target = None  # (x, y)
        self.obs = None  # list of Blob items
        self.target_vel = None  # list of Blob items

        self.last_blobs = {}  # blobs during last scan
        self.last_target = None  # (x, y)
        self.last_obs = None  # list of Blob items
        self.last_target_vel = None

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

                # if last_blob_id == blob_id:
                #     print "\nBLOB_IDs : {}, {}".format(last_blob_id, blob_id)
                #     print "Last : ({}, {})".format(follower_utils.show(last_mean_now[0][0]), follower_utils.show(last_mean_now[1][0]))
                #     print "Blob : ({}, {})".format(follower_utils.show(blob.mean[0]), follower_utils.show(blob.mean[1]))

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

        self.last_obs = copy.deepcopy(self.obs)
        self.last_target = copy.deepcopy(self.target)  # in base_scan ref
        self.last_target_vel = copy.deepcopy(self.target_vel)  # in base_scan ref

        self.obs = obs
        self.target = target  # in base_scan ref
        if self.target is not None and self.last_target is not None:
            dx = self.target.mean[0] - self.last_target.mean[0]
            dy = self.target.mean[1] - self.last_target.mean[1]
            self.target_vel = (dx, dy)
        else:
            self.target_vel = None

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
        return "{}:[{}]@({},{})".format(self.id, len(self.arr), follower_utils.show(self.mean[0], 1),
                                        follower_utils.show(self.mean[1], 1))
