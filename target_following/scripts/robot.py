#!/usr/bin/env python

import numpy as np
import math
import copy

import utils
from recovery import Recovery
from identifier import Identifier, Blob
from predictor import Predictor
from world import World
from metrics import Metrics

import rospy
import tf

from geometry_msgs.msg import Twist, Point
from nav_msgs.msg import OccupancyGrid, Odometry
from std_msgs.msg import Bool
from sensor_msgs.msg import LaserScan

PI = np.pi
SIMULATION_TIME = 100

CMD_FREQ = 10  # Hz
SLEEP = 2  # secs
VEL = 0.2  # m/s

# TODO : Figure out a better way to code robot's start pose .. env vars?
START_X_MAP = 3.0  # Would change as per the map
START_Y_MAP = 5.0  # Would change as per the map
START_YAW_MAP = 0


# START_Z_MAP = 0.0

class Robot:
    KP = 0.5

    def __init__(self):
        rospy.init_node("robot")

        self.map = None
        self.world = None  # if we do not use world in here, delete this later

        # continually updated info about robot's pose wrt odom
        self.posx = None
        self.posy = None
        self.angle = None
        self.last_posx = None
        self.last_posy = None
        self.last_angle = None

        # useful transformation matrices for movement
        self.mTo = np.array(
            # corrected!
            [[1, 0, 0, START_X_MAP], [0, 1, 0, START_Y_MAP], [0, 0, 1, 0], [0, 0, 0, 1]])  # odom to map
        self.trans_odom_to_map = np.mat(
            [[1, 0, 0, START_X_MAP],
             [0, 1, 0, START_Y_MAP],
             [0, 0, 1, 0],
             [0, 0, 0, 1]]
        )
        self.bTo = None  # odom to base_link

        self.rcvr_poses = []  # all poses to move to in order to get to last detected target pose
        self.rcvr = None
        self.id = Identifier()
        self.pred = Predictor()
        self.time_last_scan = None
        self.target_ever_found = False

        self.pub = rospy.Publisher("robot_0/cmd_vel", Twist, queue_size=0)
        self.stat_pub = rospy.Publisher("visible_status", Bool, queue_size=0)  # latest one only
        self.world_sub = rospy.Subscriber("map", OccupancyGrid, self.map_callback, queue_size=1)
        self.odom_sub = rospy.Subscriber("robot_0/odom", Odometry, self.odom_callback)
        self.sub_laser = rospy.Subscriber("robot_0/base_scan", LaserScan, self.laser_scan_callback, queue_size=1)
        self.lis = tf.TransformListener()

        self.metrics = Metrics()
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

    def get_transform(self):
        # get transformation from odom to base_link
        (trans, rot) = self.lis.lookupTransform('robot_0/base_link', 'robot_0/odom', rospy.Time(0))
        # get everything in regular matrix form
        t = tf.transformations.translation_matrix(trans)
        r = tf.transformations.quaternion_matrix(rot)
        self.bTo = t.dot(r)

    def map_callback(self, msg):
        if self.map is None:
            print("loading map")
            self.map = World(msg.data, msg.info.width, msg.info.height, msg.info.resolution, msg.header.frame_id,
                             msg.info.origin)
            print(self.map.T)

    def laser_scan_callback(self, laser_scan_msg):
        """ Uses laser scan to update target position """

        curr_time = rospy.get_time()
        if self.time_last_scan is None or (curr_time - self.time_last_scan > 1 / Identifier.SCAN_FREQ):

            if self.posx is not None and self.last_posx is not None:
                self.id.blobify(laser_scan_msg, self)  # Identify blobs
                self.id.classify(self.get_movement_transform())  # Classify blobs, set target
                tpos, tvel = self.id.get_target_pos_vel(robot=self, frame="MAP")
                self.display_target_status(tpos, tvel)

                self.pred.update_targetpos(
                    None if tpos is None else tpos[0],
                    None if tpos is None else tpos[1],
                    None if tvel is None else tvel[0],
                    None if tvel is None else tvel[1]
                )

            # Update last pose to current
            self.last_posx = self.posx
            self.last_posy = self.posy
            self.last_angle = self.angle
            self.time_last_scan = curr_time

    def update_rcvr(self):
        # update recovery with robot's current pose
        self.get_transform()  # update self.bTo first
        p = self.mTo.dot(np.transpose(np.array([self.posx, self.posy, 0, 1])))
        self.rcvr.robot_pos = Point()
        self.rcvr.robot_pos.x = p[0]
        self.rcvr.robot_pos.y = p[1]
        self.rcvr.robot_ang = self.angle + START_YAW_MAP

        while self.id.blobifying: # dict size may change while this flag is true
            continue

        # update local map
        blobs_cp = copy.deepcopy(self.id.blobs)
        blobs = {}  # blob_id:blob_objects
        self.get_transform()  # update again
        for blob_id in blobs_cp:
            blobs[blob_id] = Blob(blob_id)
            blobs[blob_id].arr = []
            arr = blobs_cp[blob_id].arr
            for i in range(len(arr)):
                x, y = arr[i]
                p = np.linalg.inv(self.bTo).dot(np.transpose(np.array([x, y, 0, 1])))
                p = self.mTo.dot(p)[0:2]
                blobs[blob_id].arr.append((p[0], p[1]))

        self.rcvr.create_local_world(blobs)

    def display_target_status(self, tpos, tvel):
        if not self.target_ever_found:
            return

        print "{}".format(''.join(['-' for _ in range(100)]))
        if tpos is not None:
            print "Target Pos      : ({},{})".format(utils.show(tpos[0]), utils.show(tpos[1]))
        if tvel is not None:
            print "Target Vel      : ({},{})".format(utils.show(tvel[0]), utils.show(tvel[1]))
        else:
            print "Target Lost"

    def move(self):
        self.rcvr = Recovery(self.map)

        rate = rospy.Rate(CMD_FREQ)
        vel_msg = Twist()
        print "Searching for target..."

        start_time = rospy.get_time()

        while not rospy.is_shutdown() and rospy.get_time() - start_time < SIMULATION_TIME:

            lin_x = 0
            ang_z = 0

            tpos, tvel = self.id.get_target_pos_vel(robot=self, frame="MAP")
            self.target_ever_found = self.target_ever_found or tpos is not None

            # we detect target so decide how to move using PID-like function
            if tpos is not None and tvel is not None:

                self.pub_visibility(True)  # can see the target
                lin_x, ang_z = self.chase()

                # recovery object will always have last known target pose to prepare for recovery mode
                tpos_map, tvel_map = self.id.get_target_pos_vel(robot=self, frame="MAP")
                self.rcvr.last_known_pos = Point()
                self.rcvr.last_known_pos.x = tpos_map[0]
                self.rcvr.last_known_pos.y = tpos_map[1]

                # clear poses for recovery when re-entering regular mode
                if self.rcvr_poses:
                    self.rcvr_poses = []

            elif rospy.get_time() - start_time < Identifier.ID_INIT_TIME:
                self.pub_visibility(False)
                self.metrics.feed(rospy.get_time())
                continue

            elif not self.target_ever_found:
                self.pub_visibility(False)
                self.metrics.feed(rospy.get_time())
                continue

            # elif self.rcvr is not None:  # target is out of sight, go into recovery mode
            else:  # target is out of sight, go into recovery mode
                # continue

                print "In Recovery : ", (
                    ", ".join(["({},{})".format(utils.show(p[0]), utils.show(p[1])) for p in self.rcvr_poses]))
                self.pub_visibility(False)  # cant see the target
                self.metrics.feed(rospy.get_time())

                if not self.rcvr_poses:
                    # we delete as we go and clear when switch state so should be empty upon switch to RECOVERY
                    self.update_rcvr()  # remember to update Recovery object's required info first
                    self.rcvr_poses = self.rcvr.recover()
                # print("retrieved rcvr_poses", self.rcvr_poses)

                # in the middle of recovery mode
                # separate if statement so we don't have to wait until next loop iteration to start moving once entered recovery mode
                if self.rcvr_poses:
                    print "goal in map frame: ({},{})".format(utils.show(self.rcvr_poses[0][0]),
                                                              utils.show(self.rcvr_poses[0][1]))
                    # essentially we are moving to every position from a list that goes [[goalx, goaly], ..., [startx, starty]], if we encounter
                    # target before we finish this list i.e. state changes back to REGULAR, just clear list to prep for next recovery call
                    pose = self.rcvr_poses[-1]

                    # transform user-given point in odom to base_link, assume ROBOT CAN'T FLY
                    self.get_transform()
                    v = np.linalg.inv(self.mTo).dot(np.transpose(np.array([pose[0], pose[1], 0, 1])))
                    v = self.bTo.dot(v)
                    v = v[0:2]  # only need x,y because of assumption above
                    # remaining angle to turn i.e. angle btwn x-axis vector and vector of x,y above
                    ang = np.arctan2(v[1], v[0])
                    if -0.05 <= ang <= 0.05:
                        # turn finished so start moving in lin x only, if applicable
                        ang_z = 0
                        # remaining euclidean distance to travel, assume no movement in z-axis
                        dis = np.linalg.norm(np.array([0, 0]) - v)
                        if -0.05 <= dis <= 0.05:
                            # lin x move finished, pop this pose
                            self.rcvr_poses.pop()
                            continue  # no need to waste a publication
                        else:
                            lin_x = VEL  # rotation ensures we always move forward
                    else:
                        lin_x = 0
                        if ang < 0:
                            ang_z = -VEL
                        else:  # can only be positive, near 0 is rounded to 0 and handled above
                            ang_z = VEL

            vel_msg.linear.x = lin_x
            # vel_msg.linear.x = 0
            vel_msg.angular.z = ang_z
            # vel_msg.angular.z = 0
            self.pub.publish(vel_msg)
            rate.sleep()

        vel_msg.linear.x = 0
        vel_msg.angular.z = 0
        self.pub.publish(vel_msg)

    def chase(self):

        # TODO: Remove unused variables once done and tested
        lin_x = VEL
        ang_z = 0.0
        target_angle_base = math.atan2(self.id.target[1], self.id.target[0])
        obs_intervals = self.id.obs_intervals

        # init chase angle
        chase_angle = target_angle_base
        print "target angle    : ", utils.show(chase_angle)

        # TODO: Adjust dt and lookahead, find better way to define
        # Incorporate predicted robot position into chase angle
        dt = 0.04
        lookahead = 5
        pred = self.pred.predict_hd(dt, lookahead)
        if pred is not None and pred[1] is not None:
            last_pred_map = pred[1][-1]
            last_pred_base = utils.map_to_base(last_pred_map, self.trans_odom_to_map,
                                               (self.posx, self.posy, self.angle))
            chase_angle = last_pred_base[1]
            print "predicted angle : ", utils.show(chase_angle)

        # Incorporate obstacles into chase angle
        robot_width_angle = 0.1
        colliding_obs = [obs_int for obs_int in obs_intervals if
                         obs_int[0] - robot_width_angle <= chase_angle <= obs_int[1] + robot_width_angle]

        if len(colliding_obs) > 0:
            # colliding, use find tangent
            min_ang_obs, max_ang_obs = colliding_obs[0]

            if math.fabs(chase_angle - target_angle_base) < 0.2:
                # print "target is the obstacle"
                pass
            else:
                print "obs             @ <{}|{}>".format(utils.show(min_ang_obs), utils.show(max_ang_obs))
                go_min_side = math.fabs(chase_angle - min_ang_obs) < math.fabs(chase_angle - max_ang_obs)
                tangent_angle = min_ang_obs if go_min_side else max_ang_obs
                print "tangent angle   : {}".format(utils.show(tangent_angle))

                # Add wiggle room
                # TODO: Does the wiggle function provide substantial benefit over constant wiggle? Is it needed?
                wiggle_angle = 0.1 * (-1 if go_min_side else 1)
                # wiggle_angle = self.get_wiggle((rvx, rvy), (tvx, tvy))
                print "wiggle angle    : {}".format(utils.show(wiggle_angle))
                # wiggle_angle = 0
                chase_angle = tangent_angle + wiggle_angle
                print "final angle     : ", utils.show(chase_angle)

        if self.id.target is not None:
            ang_z = chase_angle * Robot.KP if self.id.target is not None else 0

        dist = math.sqrt(self.id.target[0] * self.id.target[0] + self.id.target[1] * self.id.target[1])
        print "dist            : ", utils.show(dist)
        # TODO: Get linvel from a pid function to avoid collision with target
        if dist < 0.5:
            lin_x = 0

        self.metrics.feed(rospy.get_time(), dist)

        return lin_x, ang_z

    def get_wiggle(self, rv, tv):
        """
            get_wiggle :: -> (rv, tv) angle
            wiggle_angle <- angle_btwn(rv,tv)
            return wiggle angle
        """
        # getting individual components, assuming rv and tv are tuples (rv is robot velocity, tv is target velocity)
        xrv = rv[0]
        yrv = rv[1]
        xtv = tv[0]
        ytv = tv[1]
        # taking dot product
        dot = np.dot([xrv, yrv], [xtv, ytv])

        # getting lengths of both vectors
        lenrv = math.pow(xrv * xrv + yrv * yrv, 0.5)
        lentv = math.pow(xtv * xtv + ytv * ytv, 0.5)
        # getting angle between
        angle = math.acos(dot / (lenrv * lentv))

        # returning the angle between the two vector
        return angle

    def get_movement_transform(self):

        sz1 = np.sin(self.angle)
        cz1 = np.cos(self.angle)
        mat1 = np.matrix([[cz1, -sz1, 0, self.posx], [sz1, cz1, 0, self.posy], [0, 0, 1, 0], [0, 0, 0, 1]])

        sz2 = np.sin(self.last_angle)
        cz2 = np.cos(self.last_angle)
        mat2 = np.matrix([[cz2, -sz2, 0, self.last_posx], [sz2, cz2, 0, self.last_posy], [0, 0, 1, 0], [0, 0, 0, 1]])

        return mat2.getI().dot(mat1)


if __name__ == "__main__":
    # we'll probably set up target like this from main.py?
    r = Robot()
    r.move()
    r.metrics.generate()
