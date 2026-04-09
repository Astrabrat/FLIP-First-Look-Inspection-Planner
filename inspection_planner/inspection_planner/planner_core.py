#!/usr/bin/env python3
"""
Core planning logic for FLIP inspection planner.

"""

from inspection_planner.header import *  # TODO: Replace with explicit imports
from inspection_planner.utils import PlannerUtils
import os
import threading

logger.add("inspection_core_loguru.log")


# ============================================================================
# CONSTANTS - Extracted magic numbers with documentation
# ============================================================================
class Constants:
    """Physical and numerical constants for inspection planning."""
    NORMALIZATION_EPSILON = 1e-8      # Minimum norm to avoid division by zero
    DEFAULT_HOVER_OFFSET = 0.5       # Hover offset in meters
    MAX_NORM_LA_HISTORY = 100        # Maximum history for mean calculation
    MIN_POINTS_FOR_KDTREE = 1        # Minimum points needed for valid KDTree


def _strip_leading_slash(name: str) -> str:
    """Strip leading slash from parameter name."""
    return name[1:] if name.startswith('/') else name


class PlannerCore:
    """Core planning logic for inspection view pose generation."""

    def __init__(self, config: dict, node=None):
        self.cwd = os.getcwd()
        self._node = node
        self._lock = threading.Lock()  # Thread safety for instance state
        self.initializeMissionParameters(config)
        logger.info('Planner Core Initialized')

    def _get_param(self, name: str, default):
        """Get parameter value with fallback to default."""
        key = name[1:] if isinstance(name, str) and name.startswith("/") else name

        # Declare if needed (ROS2 requires declare before get)
        if not self.has_parameter(key):
            self.declare_parameter(key, default)

        val = self.get_parameter(key).value

        # If parameter exists but is unset for some reason, fall back
        if val is None:
            return default
        return val

    def _now_msg(self):
        """Get ROS 2 time message for headers."""
        if self._node is not None:
            return self._node.get_clock().now().to_msg()
        # Fallback: wall-clock time if no node provided
        return rclpy.clock.Clock().now().to_msg()

    def initializeMissionParameters(self, config: dict):
        """Initialize and validate mission parameters."""
        
        # ---- VALIDATE REQUIRED PARAMS ----
        if config["inspection_distance"] <= 0:
            raise ValueError(f"inspection_distance must be positive, got {config['inspection_distance']}")
        
        fov = config["fov"]
        if len(fov) != 2 or not (0 < fov[0] < 180) or not (0 < fov[1] < 180):
            raise ValueError(f"FOV values must be 2 elements in (0, 180) degrees, got {fov}")
        
        pp = config["photogrammetric_params"]
        if len(pp) != 2 or not (0 <= pp[0] <= 1) or not (0 <= pp[1] <= 1):
            raise ValueError(f"photogrammetric_params must be 2 elements in [0,1], got {pp}")
        
        # ---- REQUIRED PARAMS ----
        self.desired_viewing_distance = float(config["inspection_distance"])

        self.photogrammetric_params = [float(x) for x in config["photogrammetric_params"]]
        self.fov = [float(x) for x in config["fov"]]

        self.platform_modality = int(config["platform_modality"])
        self.min_pos_upd = float(config["cmd_pos_upd"])
        self.min_yaw_upd = float(config["cmd_yaw_upd"])

        self.sensor_rot = [float(x) for x in config["sensor_rotation"]]

        # Horizons MUST be ints and >= 1
        self.prediction_horizon = max(1, int(config["prediction_horizon"]))
        logger.info(f"pred_horz {self.prediction_horizon}")

        self.confidence_horizon = max(1, int(config["confidence_horizon"]))

        self.world_frame = str(config["world_frame"])
        self.baseLink_frame = str(config["base_link_frame"])

        self.run_mode = int(config["run_mode"])
        self.insp_height = float(config["inspection_height"])

        self.config = config

        self.start_flag = False
        self.execute_plan = False
        self.diagnostic_flag = True
        self.rtb = False
        self.current_mission_status = 'Initialization'

        self.switch = False
        self.vertical_jump = False
        
        # BOUND: Limit history to prevent memory leak
        self.norm_LA = []
        self.mean_norm_LA = 0.0
        
        self.path = Path()

        self.hov = Constants.DEFAULT_HOVER_OFFSET
        self.viewUPvec = np.array([0, 0, 1])

        logger.info("Successfully loaded parameters")

    def nearest_surface(self, raw_points, odom_pose, odom_curr_yaw):
        """
        Find distance to nearest surface point.
        
        Returns:
            tuple: (distance_to_nearest, cropped_points_message)
        """
        predPose = odom_pose.copy()  # Initialize predicted position

        if self.sensor_rot[2] != 0.0:
            predPose = self.getRotatedOdomYaw(odom_pose.copy(), self.sensor_rot[2])
            _, _, currYaw = PlannerUtils.quat2eul(predPose[3], predPose[4], predPose[5], predPose[6])
        else:
            currYaw = odom_curr_yaw

        points = PlannerUtils.pc2_to_xyz(raw_points)

        cpoints, croppedPointsMsg = PlannerUtils.crop_points_within_fov(
            points, predPose, fov=self.fov[0]  # Pass configured FOV
        )

        # Validate we have at least 1 point before KDTree
        usable_points = cpoints if len(cpoints) >= Constants.MIN_POINTS_FOR_KDTREE else points
        if len(usable_points) < Constants.MIN_POINTS_FOR_KDTREE:
            logger.warning("No points available for nearest surface calculation")
            return float('inf'), croppedPointsMsg

        tree = KDTree(usable_points)
        tpoints = usable_points

        dist, interestPointIdx = tree.query(predPose[0:3], k=1, workers=-1)
        interestPoint = tpoints[interestPointIdx]

        look_at = interestPoint - predPose[0:3]
        norm_lookat = np.linalg.norm(look_at)

        return norm_lookat, croppedPointsMsg

    def generateViewPose(self, pos, action, points, pose, pcl_pub_handle):
        """
        Generate next view pose based on inspection action.
        
        Args:
            pos: Current position
            action: "inspect", "maintain", or "vertical"
            points: Point cloud data
            pose: Current pose
            pcl_pub_handle: Point cloud publisher
            
        Returns:
            tuple: (command_position, valid_yaw)
            
        Raises:
            ValueError: If no points available or invalid parameters
        """
        # Validate inputs
        if points is None or len(points) == 0:
            logger.error("generateViewPose called with no points")
            raise ValueError("No points available for view pose generation")
        
        cpoints, croppedPointsMsg = PlannerUtils.crop_points_within_fov(
            points, pose, fov=self.fov[0]  # Pass configured FOV
        )

        # Handle empty point cloud
        usable_points = cpoints if len(cpoints) >= Constants.MIN_POINTS_FOR_KDTREE else points
        if len(usable_points) < Constants.MIN_POINTS_FOR_KDTREE:
            logger.error("No valid points after FOV cropping")
            return pos.copy(), 0.0  # Return safe defaults

        tree = KDTree(usable_points)
        tpoints = usable_points

        dist, interestPointIdx = tree.query(pos, k=1, workers=-1)
        interestPoint = tpoints[interestPointIdx]

        look_at = interestPoint - pos
        norm_lookat = np.linalg.norm(look_at)
        
        # Prevent division by zero
        if norm_lookat < Constants.NORMALIZATION_EPSILON:
            logger.warning("Interest point coincides with position, using default direction")
            dX = np.array([1.0, 0.0, 0.0])  # Default forward direction
            norm_lookat = Constants.NORMALIZATION_EPSILON
        else:
            dX = look_at / norm_lookat

        dY = np.cross(self.viewUPvec, dX, axis=0)
        dZ = np.cross(dX, dY, axis=0)
        
        self.desired_viewing_distance = float(self.config["inspection_distance"])
        diff_view_dist = dist - self.desired_viewing_distance

        # Initialize command position
        command_pos = pos.copy()
            
        with self._lock:  # Thread safety for mutable state
            if action == "inspect":
                
                hov = (2 * norm_lookat * np.tan(np.deg2rad(self.fov[0]) / 2) -
                     self.photogrammetric_params[0] * 2 * norm_lookat * np.tan(np.deg2rad(self.fov[0]) / 2))
                
                self.hov = hov
                
                # Handle first iteration when norm_LA is empty
                if len(self.norm_LA) == 0:
                    self.mean_norm_LA = norm_lookat
                else:
                    vov = (2 * self.mean_norm_LA * np.tan(np.deg2rad(self.fov[1]) / 2) -
                        self.photogrammetric_params[1] * 2 * self.mean_norm_LA * np.tan(np.deg2rad(self.fov[1]) / 2))
                
                if self.switch:
                    tcommand_pos = command_pos + dX * diff_view_dist - dY * hov
                else:
                    tcommand_pos = command_pos + dX * diff_view_dist + dY * hov

                # Bound history to prevent memory leak
                self.norm_LA.append(norm_lookat)
                if len(self.norm_LA) > Constants.MAX_NORM_LA_HISTORY:
                    self.norm_LA.pop(0)
                self.mean_norm_LA = np.mean(self.norm_LA)
                
            elif action == "maintain":
                tcommand_pos = command_pos + dX * diff_view_dist

            elif action == "vertical":
                # Handle first iteration
                if len(self.norm_LA) == 0:
                    self.mean_norm_LA = norm_lookat
                
                vov = (2 * self.mean_norm_LA * np.tan(np.deg2rad(self.fov[1]) / 2) -
                    self.photogrammetric_params[1] * 2 * self.mean_norm_LA * np.tan(np.deg2rad(self.fov[1]) / 2))
                
                tcommand_pos = command_pos + dX * diff_view_dist + dZ * vov
            else:
                logger.warning(f"Unknown action: {action}, defaulting to maintain")
                tcommand_pos = command_pos + dX * diff_view_dist

        # Compute valid yaw angle
        valid_yaw = np.arctan2(dX[1], dX[0]) 
        
        return tcommand_pos, valid_yaw

    def getRotatedOdomYaw(self, odom_pose, yaw_offset):
        """Apply yaw offset to odometry pose."""
        predPose = odom_pose.copy()
        
        _, _, qyaw = PlannerUtils.quat2eul(odom_pose[3], odom_pose[4], odom_pose[5], odom_pose[6])
        rqyaw = qyaw + yaw_offset
        [rqx, rqy, rqz, rqw] = PlannerUtils.eul2quat(0, 0, rqyaw)
        predPose[3] = rqx
        predPose[4] = rqy
        predPose[5] = rqz
        predPose[6] = rqw
        
        return predPose
    
    def standardViewPolicy(self, pcl_pub_handle, lidar_points, odom_pose):
        """
        Generate predicted view poses for inspection.
        
        Args:
            pcl_pub_handle: Point cloud publisher
            lidar_points: Point cloud data
            odom_pose: Current odometry pose
            
        Returns:
            tuple: (pred_path, predPathArray, refPose, cpos, cyaw)
        """
        # Validate inputs
        if lidar_points is None or len(lidar_points) == 0:
            logger.error("standardViewPolicy called with no points")
            return None, None, None, None, None
        
        pred_path = Path()

        predPose = odom_pose.copy()
        
        if self.sensor_rot[2] != 0.0:
            predPose = self.getRotatedOdomYaw(odom_pose.copy(), self.sensor_rot[2])
        
        pred_pos = predPose[0:3]
        predPathArray = []
        
        for k in range(self.prediction_horizon):
            command_pos, command_yaw = self.generateViewPose(
                pred_pos, "inspect", lidar_points, predPose, pcl_pub_handle
            )
            
            # Handle None returns
            if command_pos is None:
                logger.error(f"generateViewPose returned None at iteration {k}")
                continue

            [cqx, cqy, cqz, cqw] = PlannerUtils.eul2quat(0, 0, command_yaw)

            # Populate PoseStamped message
            pred_pose = PoseStamped()
            pred_pose.header.stamp = self._now_msg()
            pred_pose.header.frame_id = self.world_frame
            
            pred_pose.pose.position.x = command_pos[0]
            pred_pose.pose.position.y = command_pos[1]
            if self.platform_modality == 0:  # ground robot
                pred_pose.pose.position.z = odom_pose[2]
            elif self.platform_modality == 1:  # aerial robot
                pred_pose.pose.position.z = self.insp_height
            
            pred_pose.pose.orientation.x = cqx
            pred_pose.pose.orientation.y = cqy
            pred_pose.pose.orientation.z = cqz
            pred_pose.pose.orientation.w = cqw
            
            pred_path.header.frame_id = self.world_frame
            pred_path.header.stamp = self._now_msg()
            pred_path.poses.append(pred_pose)

            # Save initial reference pose
            if k == 0:
                if self.platform_modality == 0:  # ground robot
                    command_pos[2] = odom_pose[2]
                elif self.platform_modality == 1:  # aerial robot
                    command_pos[2] = self.insp_height

                cpos = command_pos
                cyaw = command_yaw
                refPose = pred_pose
                
            predPose = np.array([command_pos[0], command_pos[1], odom_pose[2], cqx, cqy, cqz, cqw])
            
            predPathArray.append(predPose)

            # Update predicted position for the next iteration
            pred_pos = command_pos
            
        # Finalize predicted path
        pred_path.header.frame_id = self.world_frame
        pred_path.header.stamp = self._now_msg()
        
        return pred_path, predPathArray, refPose, cpos, cyaw

    def evaluateValidPoints(self, points, pred_pos):
        """Evaluate distance to valid points."""
        lidar_points = self.curr_pts
        
        # Handle empty point cloud
        if len(lidar_points) < Constants.MIN_POINTS_FOR_KDTREE:
            logger.warning("No valid points for evaluation")
            return float('inf')
        
        tree = KDTree(lidar_points)
        dist, interestPointIdx = tree.query(pred_pos, k=1, workers=-1)

        interestPoint = lidar_points[interestPointIdx, :]

        look_at = interestPoint - pred_pos

        norm_lookat = np.linalg.norm(look_at)
        
        return norm_lookat

    def normalizeAngle(self, angle):
        """Normalize an angle to the range [-180, 180]."""
        return (angle + np.pi) % (2 * np.pi) - np.pi

    def interpolateYaw(self, current_yaw, target_yaw, alpha=0.5):
        """Interpolate between two yaw angles."""
        # Normalize angles to be in the range [-180, 180]
        current_yaw = self.normalizeAngle(current_yaw)
        target_yaw = self.normalizeAngle(target_yaw)
        
        # Calculate the difference
        delta_yaw = self.normalizeAngle(target_yaw - current_yaw)

        # Interpolate
        interpolated_yaw = current_yaw + alpha * delta_yaw

        return self.normalizeAngle(interpolated_yaw)
    
    def resetAll(self):
        """Reset all accumulated state. Call at mission start/end."""
        with self._lock:
            self.norm_LA = []
            self.mean_norm_LA = 0.0
            self.path = Path()
            self.switch = False
            self.vertical_jump = False
            logger.info("PlannerCore state reset")
