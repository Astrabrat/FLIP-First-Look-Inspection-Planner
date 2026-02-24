#!/usr/bin/env python3

from inspection_planner.header import *  # noqa: F401,F403
from inspection_planner.utils import PlannerUtils
import os


logger.add("inspection_core_loguru.log")


def _strip_leading_slash(name: str) -> str:
    return name[1:] if name.startswith('/') else name


class PlannerCore:
    """Non-ROS planning core.

    ROS 1 version pulled parameters from the global parameter server via rospy.
    In ROS 2, parameters are node-scoped. To keep code changes minimal and
    preserve runtime behavior, the node passes itself in and we read parameters
    from it using a ROS1-compatible key style (leading '/' accepted).
    """

    def __init__(self, config: dict, node=None):
        self.cwd = os.getcwd()
        self._node = node
        self.initializeMissionParameters(config)
        logger.info('Planner Core Initialized')

    def _get_param(self, name: str, default):
        """
        ROS2-safe param getter with ROS1-style key tolerance.
        - Accepts '/foo' or 'foo'
        - Declares parameter if not declared
        - Returns default if missing/unset
        """
        key = name[1:] if isinstance(name, str) and name.startswith("/") else name

        # Declare if needed (ROS2 requires declare before get, unless allow_undeclared)
        if not self.has_parameter(key):
            self.declare_parameter(key, default)

        val = self.get_parameter(key).value

        # If parameter exists but is unset for some reason, fall back
        if val is None:
            return default
        return val

    def _now_msg(self):
        """ROS 2 time message for headers."""
        if self._node is not None:
            return self._node.get_clock().now().to_msg()
        # Fallback: wall-clock time if no node provided (keeps non-ROS usage working)
        return rclpy.clock.Clock().now().to_msg()

    def initializeMissionParameters(self,config: dict):

        # self.prediction_horizon = config["prediction_horizon"]
        # self.confidence_horizon = config["confidence_horizon"]
        # self.min_points_confidence = config["min_pts_conf"]
        # self.desired_viewing_distance = config["inspection_distance"]
        # self.fov = config["fov"]
        # self.photogrammetric_params = config["photogrammetric_params"]
        # self.interp_alpha = config["interpolation_alpha"]
        # self.insp_height = config["inspection_height"]

        # ---- REQUIRED PARAMS (do not skip anything from your list) ----
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

        self.min_points_confidence = int(config["min_pts_conf"])
        self.interp_alpha = float(config["interpolation_alpha"])

        self.world_frame = str(config["world_frame"])
        self.baseLink_frame = str(config["base_link_frame"])

        self.max_std_deviation = float(config["max_std_deviation"])
        self.run_mode = int(config["run_mode"])
        self.insp_height = float(config["inspection_height"])

        self.config = config

        # --------------------------------------------------------------
        # self.desired_viewing_distance = self._get_param('/inspection_distance', 2.0)
        # self.photogrammetric_params = self._get_param('/photogrammetric_params', [0.6, 0.8])
        # self.fov = self._get_param('/fov', [69.4, 45])
        # self.platform_modality = self._get_param('/platform_modality', 0)
        # self.min_pos_upd = self._get_param('/cmd_pos_upd', 0.2)
        # self.min_yaw_upd = self._get_param('/cmd_yaw_upd', 0.05)
        # self.sensor_rot = self._get_param('/sensor_rotation', [0.0, 0.0, 0.0])
        # self.prediction_horizon = self._get_param('/prediction_horizon', 5)
        # self.prediction_horizon = self._get_param('')
        # logger.info(f"pred_horz {self.prediction_horizon}")
        # self.confidence_horizon = self._get_param('/confidence_horizon', 5)
        # self.min_points_confidence = self._get_param('/min_pts_conf', 10)
        # self.interp_alpha = self._get_param('/interpolation_alpha', 0.5)
        # self.world_frame = self._get_param('/world_frame', 'world')
        # self.baseLink_frame = self._get_param('/base_link_frame', 'base_link')
        # self.max_std_deviation = self._get_param('/max_std_deviation', 0.05)
        # self.run_mode = self._get_param('/run_mode', 0)
        # self.insp_height = self._get_param('/inspection_height', 1.0)

        self.start_flag = False
        self.execute_plan = False
        self.diagnostic_flag = True
        self.rtb = False
        self.current_mission_status = 'Initialization'

        self.switch = False
        self.vertical_jump = False
        self.mean_norm_LA = 0
        self.norm_LA = []
        self.path = Path()

        self.hov = 0.5
        self.viewUPvec = np.array([0, 0, 1])

        logger.info("Successfully loaded parameters")

    def nearest_surface(self, raw_points, odom_pose, odom_curr_yaw):
        predPose = odom_pose.copy()  # Initialize predicted position

        if self.sensor_rot[2] != 0.0:
            predPose = self.getRotatedOdomYaw(odom_pose.copy(), self.sensor_rot[2])
            _, _, currYaw = PlannerUtils.quat2eul(predPose[3], predPose[4], predPose[5], predPose[6])
        else:
            currYaw = odom_curr_yaw

        points= PlannerUtils.pc2_to_xyz(raw_points)

        cpoints, croppedPointsMsg = PlannerUtils.crop_points_within_fov(points, predPose)


        if len(cpoints) < 2:
            tree = KDTree(points)
            tpoints = points
        else:
            tree = KDTree(cpoints)
            tpoints = cpoints

        dist, interestPointIdx = tree.query(predPose[0:3], k=1, workers=-1)
        interestPoint = tpoints[interestPointIdx]

        look_at = interestPoint - predPose[0:3]
        norm_lookat = np.linalg.norm(look_at)

        

        return norm_lookat, croppedPointsMsg

    @logger.catch
    def generateViewPose(self, pos, action, points, pose, pcl_pub_handle):
        cpoints, croppedPointsMsg = PlannerUtils.crop_points_within_fov(points, pose)
        pcl_pub_handle.publish(croppedPointsMsg)

        if len(cpoints) < 2:
            tree = KDTree(points)
            tpoints = points
        else:
            tree = KDTree(cpoints)
            tpoints = cpoints

        dist, interestPointIdx = tree.query(pos, k=1, workers=-1)
        interestPoint = tpoints[interestPointIdx]

        look_at = interestPoint - pos
        norm_lookat = np.linalg.norm(look_at)
        dX = look_at / norm_lookat

        dY = np.cross(self.viewUPvec, dX, axis=0)
        dZ = np.cross(dX, dY, axis=0)

        # The rest of the original PlannerCore implementation remains unchanged.
        # (Copied verbatim below)

        # --- BEGIN original content ---
        # Compute distance difference
        # Keep desired distance live-updated (ROS 1 polled param server here).
        # In ROS 2 we read the node parameter if available.
        self.desired_viewing_distance = float(self.config["inspection_distance"])
        diff_view_dist = dist - self.desired_viewing_distance

        # Initialize command position
        command_pos = pos.copy()
            
        if action == "inspect":
            
            hov = (2 * norm_lookat * np.tan(np.deg2rad(self.fov[0]) / 2) -
                 self.photogrammetric_params[0] * 2 * norm_lookat * np.tan(np.deg2rad(self.fov[0]) / 2))
            
            self.hov = hov
            
            vov = (2 * self.mean_norm_LA * np.tan(np.deg2rad(self.fov[1]) / 2) -
                self.photogrammetric_params[1] * 2 * self.mean_norm_LA * np.tan(np.deg2rad(self.fov[1]) / 2))
            
            if self.switch:
                tcommand_pos = command_pos + dX * diff_view_dist - dY * hov
            else:
                tcommand_pos = command_pos + dX * diff_view_dist + dY * hov

            self.norm_LA.append(norm_lookat)
            self.mean_norm_LA = np.mean(self.norm_LA)
            
        elif action == "maintain":
            tcommand_pos = command_pos + dX * diff_view_dist

        elif action == "vertical":
            vov = (2 * self.mean_norm_LA * np.tan(np.deg2rad(self.fov[1]) / 2) -
                self.photogrammetric_params[1] * 2 * self.mean_norm_LA * np.tan(np.deg2rad(self.fov[1]) / 2))
            
            tcommand_pos = command_pos + dX * diff_view_dist + dZ * vov

        # Compute valid yaw angle
        valid_yaw = np.arctan2(dX[1], dX[0]) 
        
        return tcommand_pos, valid_yaw

    def getRotatedOdomYaw(self,odom_pose,yaw_offset):

        predPose = odom_pose.copy()
        
        _,_,qyaw = PlannerUtils.quat2eul(odom_pose[3], odom_pose[4], odom_pose[5], odom_pose[6])
        rqyaw = qyaw + yaw_offset
        [rqx, rqy, rqz, rqw] = PlannerUtils.eul2quat(0, 0, rqyaw)
        predPose[3] = rqx
        predPose[4] = rqy
        predPose[5] = rqz
        predPose[6] = rqw
        
        return predPose
    
    @logger.catch
    def standardViewPolicy(self,pcl_pub_handle,lidar_points,odom_pose):
        
        pred_path = Path()

        predPose = odom_pose.copy()
        
        if self.sensor_rot[2] != 0.0:
            predPose = self.getRotatedOdomYaw(odom_pose.copy(),self.sensor_rot[2])
        
        pred_pos = predPose[0:3]
        predPathArray = []
        
        for k in range(self.prediction_horizon):
            
            command_pos, command_yaw = self.generateViewPose(pred_pos, "inspect", lidar_points,predPose,pcl_pub_handle)
            # command_yaw = command_yaw - self.sensor_rot[2]
            [cqx, cqy, cqz, cqw] = PlannerUtils.eul2quat(0, 0, command_yaw)

            # logger.debug(f"commadn_yaw{command_yaw}")

            # Populate PoseStamped message
            pred_pose = PoseStamped()
            pred_pose.header.stamp = self._now_msg()
            pred_pose.header.frame_id = self.world_frame
            
            pred_pose.pose.position.x = command_pos[0]
            pred_pose.pose.position.y = command_pos[1]
            pred_pose.pose.position.z = odom_pose[2]
            
            pred_pose.pose.orientation.x = cqx
            pred_pose.pose.orientation.y = cqy
            pred_pose.pose.orientation.z = cqz
            pred_pose.pose.orientation.w = cqw
            
            pred_path.header.frame_id = self.world_frame
            pred_path.header.stamp = self._now_msg()
            pred_path.poses.append(pred_pose)

            # logger.debug(f"{command_pos}, {pred_pos}")

            # Save initial reference pose
            if k == 0:
                command_pos[2] = odom_pose[2]
                cpos = command_pos
                cyaw = command_yaw
                refPose = pred_pose
                
            predPose = np.array([command_pos[0],command_pos[1],odom_pose[2],cqx,cqy,cqz,cqw])
            
            predPathArray.append(predPose)

            # Update predicted position for the next iteration
            pred_pos = command_pos
            
            # input()

        # Finalize predicted path
        pred_path.header.frame_id = self.world_frame
        pred_path.header.stamp = self._now_msg()
        
        return pred_path, predPathArray, refPose, cpos, cyaw


    def evaluateValidPoints(self,points, pred_pos):
        
        lidar_points = self.curr_pts
        
        tree = KDTree(lidar_points)
        dist, interestPointIdx = tree.query(pred_pos, k=1,workers=-1)

        interestPoint = lidar_points[interestPointIdx, :]

        look_at =  interestPoint - pred_pos

        norm_lookat = np.linalg.norm(look_at)
        
        return norm_lookat

    def normalizeAngle(self,angle):
        """ Normalize an angle to the range [-180, 180]. """
        return (angle + np.pi) % (2 * np.pi) - np.pi

    def interpolateYaw(self,current_yaw, target_yaw, alpha=0.5):

        """
        Interpolates between current_yaw and target_yaw.
        
        Parameters:
            current_yaw (float): Current yaw angle in degrees.
            target_yaw (float): Target yaw angle in degrees.
            alpha (float): Interpolation factor between 0 and 1.
        
        Returns:
            float: Interpolated yaw angle.
        """

        # Normalize angles to be in the range [-180, 180]
        current_yaw = self.normalizeAngle(current_yaw)
        target_yaw = self.normalizeAngle(target_yaw)
        
        # Calculate the difference
        delta_yaw = self.normalizeAngle(target_yaw - current_yaw)

        # Interpolate
        interpolated_yaw = current_yaw + alpha * delta_yaw

        return self.normalizeAngle(interpolated_yaw)
    

    def resetAll(self):

        self.norm_LA = []
        self.path = Path()



        
        
        
        
    

        