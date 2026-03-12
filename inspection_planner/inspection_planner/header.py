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


# --------------------------------------------------------------------------
# ROS2-Compatible Timing Utilities
# Keeps ROS1-style `tic()` / `toc()` API but does not depend on ttictoc.
# --------------------------------------------------------------------------

# If ttictoc happens to be installed, we’ll use it. Otherwise, fallback.
try:
    from ttictoc import tic, toc  # type: ignore
except Exception:
    _TIC_STACK = []
    _TIC_LOCK = threading.Lock()

    def tic():
        """
        Start a timer (stack-based, supports nesting).
        Uses high-resolution monotonic clock.
        """
        with _TIC_LOCK:
            _TIC_STACK.append(time.perf_counter())

    def toc(
        msg: str = "",
        *,
        node=None,
        use_ros_time: bool = False,
        log_level: str = "info",
        pop: bool = True,
        silent: bool = False,
    ) -> float:
        """
        Stop timer and return elapsed time in seconds.

        Args:
            msg: Optional message prefix.
            node: Optional rclpy Node. If provided, logs using node.get_logger().
            use_ros_time: If True and node provided, uses ROS clock instead of perf_counter.
                          Note: ROS time requires node and meaningful clock; otherwise falls back.
            log_level: Logging level name ('info', 'debug', 'warn', 'error', ...).
            pop: If True, pops the last tic() (stack behavior). If False, peeks.
            silent: If True, don't log anything (just return dt).

        Returns:
            Elapsed time in seconds.
        """
        with _TIC_LOCK:
            if not _TIC_STACK:
                raise RuntimeError("toc() called before tic()")
            t0 = _TIC_STACK.pop() if pop else _TIC_STACK[-1]

        if use_ros_time and node is not None:
            now = node.get_clock().now().nanoseconds * 1e-9
        else:
            now = time.perf_counter()

        dt = now - t0

        if not silent:
            formatted = f"{msg}: {dt:.6f} s" if msg else f"{dt:.6f} s"

            # Prefer ROS2 logger if node provided; else loguru
            if node is not None:
                ros_logger = node.get_logger()
                # Map common names + tolerate "warning"/"warn"
                level = log_level.lower()
                if level == "warn":
                    level = "warning"
                fn = getattr(ros_logger, level, None)
                if callable(fn):
                    fn(formatted)
                else:
                    ros_logger.info(formatted)
            else:
                # loguru expects uppercase levels typically, but accepts strings
                logger.log(log_level.upper(), formatted)

        return dt

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