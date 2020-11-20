#!/usr/bin/env python

import numpy as np
import math
import copy

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

CMD_FREQ = 10  # Hz
SLEEP = 2  # secs
VEL = 0.2  # m/s

PI = np.pi

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
        self.bTo = None  # odom to base_link

        self.rcvr_poses = []  # all poses to move to in order to get to last detected target pose
        self.rcvr = None
        self.id = Identifier()
        self.time_last_scan = None
        self.target_ever_found = False

        self.pub = rospy.Publisher("robot_0/cmd_vel", Twist, queue_size=0)
        self.stat_pub = rospy.Publisher("visible_status", Bool, queue_size=0)  # latest one only
        self.world_sub = rospy.Subscriber("map", OccupancyGrid, self.map_callback, queue_size=1)
        self.odom_sub = rospy.Subscriber("robot_0/odom", Odometry, self.odom_callback)
        self.sub_laser = rospy.Subscriber("robot_0/base_scan", LaserScan, self.laser_scan_callback, queue_size=1)
        self.lis = tf.TransformListener()

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
        # print curr_time, self.time_last_scan
        if self.time_last_scan is None or (curr_time - self.time_last_scan > 1 / Identifier.SCAN_FREQ):
            # print 1111
            # Scan based on SCAN_FREQ

            if self.posx is not None and self.last_posx is not None:
                self.id.blobify(laser_scan_msg, self)  # Identify blobs
                self.id.classify(self.get_movement_transform())  # Classify blobs, set target
                tpos, tvel = self.id.get_target_pos_vel(robot=self, frame="MAP")
                self.display_target_status(tpos, tvel)

            # Update last pose to current
            self.last_posx = self.posx
            self.last_posy = self.posy
            self.last_angle = self.angle
            self.time_last_scan = curr_time

    def update_rcvr(self):
        # update recovery with robot's current pose
        self.get_transform()  # update self.bTo first
        p = self.mTo.dot(np.transpose(np.array([self.posx, self.posy, 0, 1])))[0:2]
        self.rcvr.robot_pos = Point()
        self.rcvr.robot_pos.x = p[0]
        self.rcvr.robot_pos.y = p[1]
        self.rcvr.robot_ang = self.angle + START_YAW_MAP

        # update local map
        blobs = copy.deepcopy(self.id.blobs)
        #self.get_transform() # update again
        #for blob_id in blobs:
        #    arr = blobs[blob_id].arr
        #    for i in range(len(arr)):
        #        x, y = arr[i]
        #        p = np.linalg.inv(self.bTo).dot(np.transpose(np.array([x, y, 0, 1])))
        #        p = self.mTo.dot(p)[0:2]
        #        arr[i] = (p[0], p[1])

        self.rcvr.create_local_world(blobs)


    def display_target_status(self, tpos, tvel):
        if not self.target_ever_found:
            return

        print "{}".format(''.join(['-' for _ in range(100)]))
        if tpos is not None:
            print "Target Pos : ({},{})".format(
                follower_utils.show(tpos[0]), follower_utils.show(tpos[1]))
        # if tvel is not None:
        # 	print "Target Vel : ({},{})".format(
        # 		follower_utils.show(tvel[0]), follower_utils.show(tvel[1]))
        else:
            print "Target Lost"

    def move(self):
        self.rcvr = Recovery(self.map)

        rate = rospy.Rate(CMD_FREQ)
        vel_msg = Twist()
        print "Searching for target..."

        start_time = rospy.get_time()

        while not rospy.is_shutdown():
            lin_x = 0
            ang_z = 0

            tpos, tvel = self.id.get_target_pos_vel(robot=self, frame="MAP")
            self.target_ever_found = self.target_ever_found or tpos is not None
            # self.display_target_status(tpos, tvel)

            # we detect target so decide how to move using PID-like function
            if False and tpos is not None and tvel is not None:

                self.pub_visibility(True)  # can see the target
                lin_x, ang_z = self.chase(tpos, tvel)

                # recovery object will always have last known target pose to prepare for recovery mode
                tpos_map, tvel_map = self.id.get_target_pos_vel(robot=self, frame="MAP")
                self.rcvr.last_known_pos = Point()
                self.rcvr.last_known_pos.x = tpos_map[0]
                self.rcvr.last_known_pos.y = tpos_map[1]

                # clear poses for recovery when re-entering regular mode
                if self.rcvr_poses:
                    self.rcvr_poses = []

            elif False and rospy.get_time() - start_time < Identifier.ID_INIT_TIME:
                self.pub_visibility(True)
                continue

            elif False and not self.target_ever_found:
                self.pub_visibility(True)
                continue

            # elif self.rcvr is not None:  # target is out of sight, go into recovery mode
            else:  # target is out of sight, go into recovery mode
                # continue

                print "In Recovery : {}".format(self.rcvr_poses)
                self.pub_visibility(False)  # cant see the target

                if not self.rcvr_poses:
                    # we delete as we go and clear when switch state so should be empty upon switch to RECOVERY
                    self.update_rcvr()  # remember to update Recovery object's required info first
                    #self.rcvr_poses = self.rcvr.recover()
                #print("retrieved rcvr_poses", self.rcvr_poses)

                # in the middle of recovery mode
                # separate if statement so we don't have to wait until next loop iteration to start moving once entered recovery mode
                if self.rcvr_poses:
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

    def chase(self, tpos, tvel):
        lin_x = VEL
        ang_z = 0.0

        rpx = self.posx  # robot pos x
        rpy = self.posy  # robot pos y
        rpz = self.angle  # robot angle

        rvx = (self.posx - self.last_posx) * Identifier.SCAN_FREQ  # robot vel x
        rvy = (self.posy - self.last_posy) * Identifier.SCAN_FREQ  # robot vel y
        rvz = (self.angle - self.last_angle) * Identifier.SCAN_FREQ  # robot ang vel

        tpx = tpos[0]  # target pos x
        tpy = tpos[1]  # target pos y

        tvx = tvel[0]  # target vel x
        tvy = tvel[1]  # target vel y

        obs = self.id.obs

        # print "RP:{},{},{}".format(follower_utils.show(rpx), follower_utils.show(rpy), follower_utils.show(rpz))
        # print "RV:{},{},{}".format(follower_utils.show(rvx), follower_utils.show(rvy), follower_utils.show(rvz))
        # print "TP:{},{}".format(follower_utils.show(tpx), follower_utils.show(tpy))
        # print "TV:{},{}".format(follower_utils.show(tvx), follower_utils.show(tvy))

        if self.id.target is not None:
            # print "PID-TargetY:", self.id.target[1]
            ang_z = self.id.target[1] * Robot.KP if self.id.target is not None else 0

        dist = math.sqrt((rpx - tpx) * (rpx - tpx) + (rpy - tpy) * (rpy - tpy))
        if dist < 0.3:
            lin_x = 0

        """
        speed,  angle_of_pid <- pid_like(rp, rv, tp, tv)
        if not obs @ angle_of_pid:
            return angle_of_pid
        angle_left <- search left
        angle_right <- search left
        angle_of_tangent <- min (a_L,a_R)
        angle_wiggle <- angle + get_wiggle() # angle diff in robot and target vels
        return angle_of_pid + angle_wiggle
        """

        # (angle, vel_tuple) = self.pid_like((rpx, rpy), (rvx, rvy), (tpx, tpy), (tvx, tvy))

        return lin_x, ang_z

    def pid_like(self, rp, rv, tp, tv):
        """
            pid_like :: (rp, rv, tp, tv) -> angle
            split into x and y
            consider from the target's frame of ref
            get the dx and dy
            convert back to robot's frame
        """
        # getting individual components, assuming rv and tv are tuples (rv is robot velocity, tv is predicted target velocity)
        xrv = rv[0]
        yrv = rv[1]
        xtv = tv[0]
        ytv = tv[1]
        # rp is robot position, tp is target position
        xrp = rp[0]
        yrp = rp[1]
        xtp = tp[0]
        ytp = tp[1]
        # direct velocity
        diff_x = xtp - xrp
        diff_y = ytp - yrp
        # normalizing difference, this produces the direct velocity to the target from the robot (aka, this produces
        # a normalized velocity in the direction of the target's current position from the robot's current position)
        direct_velx = diff_x / (math.pow(diff_x, 2) + math.pow(diff_y, 2))
        direct_vely = diff_y / (math.pow(diff_x, 2) + math.pow(diff_y, 2))

        # averaging the direct velocities and the predicted velocities
        robot_velx = (xtv + direct_velx) / 2
        robot_vely = (ytv + direct_vely) / 2
        vel_tuple = (robot_velx, robot_vely)
        angle = math.atan2(robot_vely, robot_velx)
        return (angle, vel_tuple)

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
