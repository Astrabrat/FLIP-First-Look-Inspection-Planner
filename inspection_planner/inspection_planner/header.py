#!/usr/bin/env python3
"""
Common imports used across the inspection_planner python nodes.

ROS 1 code used a `header.py` that pulled in `rospy` and message types.
This ROS 2 port keeps the same public surface (so other modules can keep
`from header import *`) but replaces ROS 1 APIs with ROS 2 equivalents.

Important: we intentionally do *not* provide a rospy-compat shim. Instead,
callers should use the Node instance (rclpy) for parameters, pubs/subs, time.
"""

import heapq
import json
import math
import os
import random
import threading
import time
from collections import deque
from itertools import islice, tee

# import matplotlib.pyplot as plt
import numpy as np
import rclpy
import tf2_ros
from angles import shortest_angular_distance
from geometry_msgs.msg import (
    Point,
    PointStamped,
    Pose,
    PoseArray,
    PoseStamped,
    PoseWithCovariance,
    PoseWithCovarianceStamped,
)
from loguru import logger
from sensor_msgs_py.numpy_compat import structured_to_unstructured
from sensor_msgs_py import point_cloud2
from nav_msgs.msg import Odometry, Path
from scipy.spatial import KDTree
from scipy.spatial.transform import Rotation as R
from sensor_msgs.msg import CompressedImage, Image, Imu, PointCloud2, PointField
from std_msgs.msg import (
    Float32MultiArray,
    Float64,
    Float64MultiArray,
    Header,
    MultiArrayDimension,
    String,
    ColorRGBA,
)
from std_srvs.srv import Trigger
from visualization_msgs.msg import Marker, MarkerArray

from numpy import deg2rad, float64, int64

# inspection_msgs (custom)
from inspection_msgs.msg import *  # noqa: F403

from rclpy.qos import QoSProfile, ReliabilityPolicy, DurabilityPolicy, HistoryPolicy
from rclpy.qos import qos_profile_sensor_data

# For sensor subscriptions (PointCloud2, Image, LaserScan, etc.)
# Sensors
QOS_SENSOR = qos_profile_sensor_data

# Visualization
QOS_VIZ = QoSProfile(
    history=HistoryPolicy.KEEP_LAST,
    depth=10,
    reliability=ReliabilityPolicy.BEST_EFFORT,
    durability=DurabilityPolicy.VOLATILE,
)

# Control
QOS_CONTROL = QoSProfile(
    history=HistoryPolicy.KEEP_LAST,
    depth=10,
    reliability=ReliabilityPolicy.RELIABLE,
    durability=DurabilityPolicy.VOLATILE,
)