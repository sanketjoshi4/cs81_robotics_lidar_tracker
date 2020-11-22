#!/usr/bin/env python

import numpy as np


def show_pos(np_pos):
    """ A printable version of 1d numpy array for pose """
    return show(float(np_pos[0][0])), show(float(np_pos[1][0]))


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


def map_to_base(pos_from, odom_to_map=None, robot_pose_odom=None):

    if pos_from is None:
        return None

    np_pos_from = np.array([
        [pos_from[0] if pos_from[0] is not None else 0],
        [pos_from[1] if pos_from[1] is not None else 0],
        [0],
        [1],
    ])

    sz, cz = np.sin(robot_pose_odom[2]), np.cos(robot_pose_odom[2])
    odom_to_base = np.matrix([
        [cz, -sz, 0, robot_pose_odom[0]],
        [sz, cz, 0, robot_pose_odom[1]],
        [0, 0, 1, 0],
        [0, 0, 0, 1]
    ]).getI()

    np_pos_to = odom_to_base.dot(odom_to_map.getI()).dot(np_pos_from)
    return float(np_pos_to[0][0]), float(np_pos_to[1][0])

def transform(frame_from, frame_to, pos_from, odom_to_map=None, robot_pose_odom=None):
    # MAP <-> ODOM <-> BASE

    if frame_from == frame_to:
        return pos_from

    if pos_from is None:
        return None

    np_pos_from = np.array([
        [pos_from[0] if pos_from[0] is not None else 0],
        [pos_from[1] if pos_from[1] is not None else 0],
        [0],
        [1],
    ])

    sz, cz = np.sin(robot_pose_odom[2]), np.cos(robot_pose_odom[2])
    base_to_odom = np.matrix([
        [cz, -sz, 0, robot_pose_odom[0]],
        [sz, cz, 0, robot_pose_odom[1]],
        [0, 0, 1, 0],
        [0, 0, 0, 1]
    ])
    odom_to_base = base_to_odom.getI()

    map_to_odom = odom_to_map.getI()

    if frame_from == "MAP":
        if frame_to == "ODOM":
            trans_mat = map_to_odom
        else:
            trans_mat = odom_to_base.dot(map_to_odom)
    elif frame_from == "ODOM":
        if frame_to == "MAP":
            trans_mat = odom_to_map
        else:
            trans_mat = odom_to_base
    else:
        if frame_to == "ODOM":
            trans_mat = base_to_odom
        else:
            trans_mat = odom_to_map.dot(base_to_odom)

    np_pos_to = trans_mat.dot(np_pos_from)

    return float(np_pos_to[0][0]), float(np_pos_to[1][0])

