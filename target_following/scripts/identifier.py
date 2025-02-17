import copy
import numpy as np
import utils


class Identifier:
    """
    Responsible for using laser scan data to identify obstacles and moving entities
    """

    DEBUG_MODE = True  # To enable debugging with print statements

    SCAN_FREQ = 1  # Hz
    ID_INIT_TIME = 3.0  # seconds before ID first detects the target
    LASER_RANGE = 2.0

    # The minimum change in laser reading between adjacent angles, to consider them to be coming from different objects
    THRESH_RADIALLY_SEPARATE = 0.6

    # The maximum distance a blob's mean can move to imply that it's stationary
    THRESH_BLOB_STATIC = 0.5

    # The maximum distance a blob's mean can move to imply that it's moving
    # This is not set as the minimum since it is difficult to match a blob with it's last scan
    # Hence, we compare with all blobs in the last scan and the ones that have a large difference are probably
    # Different blobs and the ones with a small distance, but greater than THRESH_BLOB_STATIC, are moving blobs
    THRESH_BLOB_MOVEMENT = 1.5

    THRESH_TARGET_SIZE = 0.4

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
        """
        Initializes the instance variables
        """

        self.blobs = {}  # dict of blob_id to Blob item
        self.target = None  # (x, y)
        self.obs_ids = None  # list of Blob item ids
        self.obj_ids = None  # list of moving object ids, minus target
        self.target_vel = None  # list of Blob items
        self.obs_intervals = None

        self.last_blobs = {}  # blobs during last scan
        self.last_target = None  # (x, y)
        self.last_known_target = None  # (x, y)
        self.last_obs = None  # list of Blob items
        self.last_target_vel = None

        self.status = None  # Amongst
        self.blobifying = False  # To track when blobify() is running

    def blobify(self, laser_scan_msg, robot):
        """
        This updates the dict of blobs. Each blob is stored against a place holder id
        @param laser_scan_msg: The laser scan callback data
        @param robot: The robot object for the identifier
        """

        self.blobifying = True  # Whether this function is under execution

        amin = laser_scan_msg.angle_min  # Minimum angle of overall scan
        incr = laser_scan_msg.angle_increment  # Angle increment of overall scan
        arr = laser_scan_msg.ranges  # List of readings

        # save blobs from last scan
        self.last_blobs = copy.deepcopy(self.blobs)
        self.blobs = {}

        blob_id = 0
        # identify blobs
        for idx, dist in enumerate(arr):

            if dist >= Identifier.LASER_RANGE:
                blob_id += 1
                continue

            angle = amin + incr * idx
            incident = (dist * np.cos(angle), dist * np.sin(angle))

            if idx == 0 or abs(dist - arr[idx - 1]) > Identifier.THRESH_RADIALLY_SEPARATE:
                # means that it's two separate blobs
                blob_id += 1
                self.blobs[blob_id] = Blob(blob_id)

            if blob_id not in self.blobs:
                self.blobs[blob_id] = Blob(blob_id)
            self.blobs[blob_id].add_point(incident)

        # Calculate blob means
        for blob_id, blob in self.blobs.items():
            blob.calculate_mean_and_size(incr)

        obs_flag_arr = [i < Identifier.LASER_RANGE for i in arr]
        self.obs_intervals = Identifier.get_obstacle_intervals(obs_flag_arr, amin, incr)

        if Identifier.DEBUG_MODE:
            print ", ".join([b.show(robot) for _, b in self.blobs.items()])

        self.blobifying = False

    def classify(self, movement_transform):
        """
        This is responsible for separating moving blobs from static ones. Saves the blob in motion as the target
        @param movement_transform: The homogeneous transformation matrix between the last and current robot pose
        """

        obs_ids = set()
        obj_ids = set()
        target = None

        for last_blob_id, last_blob in self.last_blobs.items():
            # For every blob in last scan

            last_mean_now = movement_transform.getI().dot(
                np.array([[last_blob.mean[0]], [last_blob.mean[1]], [0], [1]])
            )

            for blob_id, blob in self.blobs.items():
                # Compare with every blob in current scan

                too_large = last_blob.size > Identifier.THRESH_TARGET_SIZE or blob.size > Identifier.THRESH_TARGET_SIZE

                # This is the shift between a pair of blobs
                dx = last_mean_now[0][0] - blob.mean[0]
                dy = last_mean_now[1][0] - blob.mean[1]
                shift = float(np.sqrt(dx * dx + dy * dy))

                is_static = shift <= Identifier.THRESH_BLOB_STATIC
                is_moving = Identifier.THRESH_BLOB_STATIC < shift <= Identifier.THRESH_BLOB_MOVEMENT
                is_different = Identifier.THRESH_BLOB_MOVEMENT < shift

                close_to_last_target = True
                if self.last_known_target is not None:
                    dtx = self.last_known_target[0] - blob.mean[0]
                    dty = self.last_known_target[1] - blob.mean[1]
                    close_to_last_target = np.sqrt(dtx * dtx + dty * dty) < 0.5

                if is_different:
                    # Too far to be the same blob
                    continue

                if too_large:
                    # Too large to be the target
                    if Identifier.DEBUG_MODE:
                        print "OBS   : {}, {}".format(blob_id, last_blob_id)
                    obs_ids.add(blob_id)
                    continue

                if not close_to_last_target:
                    # Too far from last target location to be the target
                    obj_ids.add(blob_id)
                    continue

                if is_moving:
                    # If moved since last position
                    if Identifier.DEBUG_MODE:
                        print "TARGET: {}, {}".format(blob_id, last_blob_id)
                    target = blob
                    continue

                if is_static:
                    # Stable since last position
                    if Identifier.DEBUG_MODE:
                        print "STATIC: {}, {}".format(blob_id, last_blob_id)
                    target = blob
                    pass

        # Save last blobs and target
        self.last_obs = copy.deepcopy(self.obs_ids)
        self.last_target = copy.deepcopy(self.target)  # in base_scan ref
        self.last_known_target = copy.deepcopy(self.target) if self.target is not None else self.last_known_target

        # Update blobs and target
        self.obs_ids = obs_ids
        self.obj_ids = obj_ids
        self.target = target.mean if target is not None else None  # in base_scan ref

    def status(self):
        """
        This returns the FSM state given the target location w.r.t. robot
        """

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

    @staticmethod
    def get_pos(target, robot_x, robot_y, robot_angle, trans_odom_to_map, frame):
        """
        Converts a point in base frame to a given frame
        @param target: The point in question
        @param robot_x: Robot's odom reading (X)
        @param robot_y: Robot's odom reading (Y)
        @param robot_angle: Robot's odom reading (Yaw)
        @param trans_odom_to_map: Homogeneous transformation matrix form odom to map
        @param frame: The output frame
        """

        if target is None:
            return None

        if frame == "BASE":
            return target

        # BASE to ODOM
        sz, cz = np.sin(robot_angle), np.cos(robot_angle)
        pos_odom = np.matrix(
            [[cz, -sz, 0, robot_x], [sz, cz, 0, robot_y], [0, 0, 1, 0], [0, 0, 0, 1]]
        ).dot(np.array([[target[0]], [target[1]], [0], [1]]))

        if frame == "ODOM":
            return float(pos_odom[0][0]), float(pos_odom[1][0])

        pos_map = trans_odom_to_map.dot(pos_odom)
        if frame == "MAP":
            return float(pos_map[0][0]), float(pos_map[1][0])

        return None

    def get_target_pos_vel(self, robot, frame):
        """
        This returns the target position and velocity if identified, in the specified frame of reference
        @robot: The robot's odom pose
        @frame: The output frame
        """

        if self.target is None:
            return None, None

        # Current target position in frame
        target_pos = Identifier.get_pos(
            target=self.target,
            robot_x=robot.posx, robot_y=robot.posy, robot_angle=robot.angle,
            trans_odom_to_map=robot.trans_odom_to_map, frame=frame
        )

        # Last target position in frame
        last_target_pos = Identifier.get_pos(
            target=self.last_target,
            robot_x=robot.last_posx, robot_y=robot.last_posy, robot_angle=robot.last_angle,
            trans_odom_to_map=robot.trans_odom_to_map, frame=frame
        )

        # Only current pose
        if self.last_target is None:
            return target_pos, None

        # Calculate velocity
        target_vel = (
            (target_pos[0] - last_target_pos[0]) * Identifier.SCAN_FREQ,
            (target_pos[1] - last_target_pos[1]) * Identifier.SCAN_FREQ
        )

        return target_pos, target_vel

    def get_obstacles(self):
        """
        References obstacles from lookup
        """

        return [self.blobs[blob_id] for blob_id in (self.obj_ids + self.obs_ids)]

    @staticmethod
    def get_obstacle_intervals(obs_flag_arr, amin, incr):
        """
        Returns the minimum and maximum angles for each identified obstacle
        @param obs_flag_arr: Whether an obstacle is present at a particular laser reading
        @param amin: Minimum laser reading angle
        @param incr: Angle increment between laser readings
        """

        in_obstacle = obs_flag_arr[0] is True
        temp_start = 0
        intervals = []
        for idx, is_obs in enumerate(obs_flag_arr):
            if not in_obstacle and is_obs:
                temp_start = idx
                in_obstacle = True
            elif in_obstacle and not is_obs:
                intervals.append((temp_start, idx - 1))
                in_obstacle = False
        if in_obstacle:
            intervals.append((temp_start, len(obs_flag_arr) - 1))

        return [(amin + incr * interval[0], amin + incr * interval[1]) for interval in intervals]


class Blob:
    """
    Encapsulates a blob identified by the ID
    """

    def __init__(self, blob_id):
        """
        Constructor for a Blob item
        """

        self.id = blob_id
        self.arr = None  # [coordinates of incident laser rays in bot frame]
        self.mean = None  # [mean coordinate in bot frame]
        self.size = None  # [mean coordinate in bot frame]

    def add_point(self, point):
        """
        Adds a point to a blob
        @param point: The point to add
        """

        if self.arr is None:
            self.arr = []
        self.arr.append(point)

    def calculate_mean_and_size(self, increment):
        """
        Calculates the blob mean and apparent size
        @param increment: The angle increment between readings
        """

        if self.arr is not None:
            self.mean = (
                sum([p[0] for p in self.arr]) / len(self.arr), sum([p[1] for p in self.arr]) / len(self.arr))
            dist_mean = np.sqrt(self.mean[0] * self.mean[0] + self.mean[1] * self.mean[1])
            self.size = increment * len(self.arr) * dist_mean

    def dist(self, blob2):
        """
        Distance between two blobs
        @param blob2: Distance between two blobs
        """

        dx = self.mean[0] - blob2.mean[0]
        dy = self.mean[1] - blob2.mean[1]
        return float(np.sqrt(dx * dx + dy * dy))

    def show(self, robot):
        """
        Pretty print a blob
        @param robot: The identifier's robot object
        """

        mean_map = Identifier.get_pos(
            target=self.mean,
            robot_x=robot.posx, robot_y=robot.posy, robot_angle=robot.angle,
            trans_odom_to_map=robot.trans_odom_to_map, frame="MAP"
        )

        return "{}:[{}]@({},{})".format(self.id,
                                        utils.show(self.size),
                                        utils.show(mean_map[0], 1),
                                        utils.show(mean_map[1], 1)
                                        )
