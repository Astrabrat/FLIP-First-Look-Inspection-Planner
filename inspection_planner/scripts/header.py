#!/usr/bin/env python3

import heapq
import json
import math
import random
from itertools import islice, tee
from collections import deque

# Standard libraries
import os
import time

# Numeric / geometry / data libs
import numpy as np
from numpy import deg2rad, float64, int64
from scipy.spatial import KDTree
from scipy.spatial.transform import Rotation as R
import matplotlib.pyplot as plt

# ROS 2 libraries
import rclpy
from rclpy.node import Node

# TF2 and geometry
import tf_transformations
import tf2_ros
import tf2_geometry_msgs

# ROS 2 message types
from geometry_msgs.msg import (
    Point,
    PointStamped,
    Pose,
    PoseArray,
    PoseStamped,
    PoseWithCovariance,
    PoseWithCovarianceStamped,
)
from nav_msgs.msg import Odometry, Path
from sensor_msgs.msg import CompressedImage, Image, Imu, PointCloud2, PointField
from std_msgs.msg import (
    Float32MultiArray,
    Float64,
    Float64MultiArray,
    Header,
    MultiArrayDimension,
    String,
)
from std_srvs.srv import Trigger
from visualization_msgs.msg import Marker, MarkerArray

# Utilities
from loguru import logger
from angles import shortest_angular_distance
from ros2_numpy import point_cloud2  # use ros2_numpy instead of ros_numpy




from geometry_msgs.msg import PoseStamped
from nav_msgs.msg import Path
from std_srvs.srv import Trigger
from builtin_interfaces.msg import Time
