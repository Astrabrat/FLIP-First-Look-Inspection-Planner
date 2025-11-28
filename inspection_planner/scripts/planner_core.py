#!/usr/bin/env python3

from header import * 
from utils import PlannerUtils
import os

logger.add("inspection_core_loguru.log")

class PlannerCore():
    
    def __init__(self):
        self.cwd = os.getcwd()
        
        self.initializeMissionParameters()
        
        logger.info('Planner Core Initialized')


        
    def initializeMissionParameters(self):
    
        self.desired_viewing_distance = rospy.get_param('/inspection_distance', 2.0)
        self.photogrammetric_params = rospy.get_param('/photogrammetric_params', [0.6, 0.8])
        self.fov = rospy.get_param('/fov', [69.4,45])
        self.platform_modality = rospy.get_param('/platform_modality', 0)
        self.min_pos_upd = rospy.get_param('/cmd_pos_upd', 0.2)
        self.min_yaw_upd = rospy.get_param('/cmd_yaw_upd', 0.05)
        self.sensor_rot = rospy.get_param('/sensor_rotation', [0.0, 0.0, 0.0])
        self.prediction_horizon = rospy.get_param('/prediction_horizon', 5)
        self.confidence_horizon = rospy.get_param('/confidence_horizon', 5)
        self.min_points_confidence = rospy.get_param('/min_pts_conf', 10)
        self.interp_alpha = rospy.get_param('/interpolation_alpha', 0.5)
        self.world_frame = rospy.get_param('/world_frame', 'world')
        self.baseLink_frame = rospy.get_param('/base_link_frame', 'base_link')
        self.max_std_deviation = rospy.get_param('/max_std_deviation', 0.05)
        self.run_mode = rospy.get_param('/run_mode', 0)
        self.insp_height = rospy.get_param('/inspection_height', 1.0)
        
        self.start_flag = False
        self.execute_plan = False
        self.diagnostic_flag = True
        self.rtb = False
        self.current_mission_status = 'Initialization'
        
        self.switch = True
        self.vertical_jump = False
        self.mean_norm_LA = 0
        self.norm_LA = []
        self.path = Path()
    
        self.hov = 0.5
        self.viewUPvec = np.array([0,0,1])

        rospy.loginfo("Sucessfully loaded parameters")
        
    def nearest_surface(self,raw_points, odom_pose):
        
        pred_pose = odom_pose.copy()  # Initialize predicted position
        points = point_cloud2.pointcloud2_to_xyz_array(raw_points, remove_nans=True)
        
        cpoints,croppedPointsMsg = PlannerUtils.crop_points_within_fov(points,pred_pose)
        
        # self.pub_cropped_points.publish(croppedPointsMsg)
        
        if len(cpoints) < 2:
            # cpoints,croppedPointsMsg = PlannerUtils.crop_points_within_fov(points,pred_pose,increment_fov=True)
            
            # fallback to the orignal points list otherwise the rest of the code would fail
            tree = KDTree(points)
            tpoints = points
        else:
            tree = KDTree(cpoints)
            tpoints = cpoints
            
        # Find the nearest interest point
        dist, interestPointIdx = tree.query(pred_pose[0:3], k=1, workers=-1)
        interestPoint = tpoints[interestPointIdx]

        # Compute direction vectors
        look_at = interestPoint - pred_pose[0:3]
        norm_lookat = np.linalg.norm(look_at)
        
        return norm_lookat, croppedPointsMsg
    
    @logger.catch
    def generateViewPose(self, pos, action, points,pose,pcl_pub_handle):

        cpoints,croppedPointsMsg = PlannerUtils.crop_points_within_fov(points,pose)
        
        pcl_pub_handle.publish(croppedPointsMsg)
        
        if len(cpoints) < 2:
            # fallback to the orignal points list otherwise the rest of the code would fail
            tree = KDTree(points)
            tpoints = points
        else:
            tree = KDTree(cpoints)
            tpoints = cpoints
        
        # Find the nearest interest point
        dist, interestPointIdx = tree.query(pos, k=1, workers=-1)
        interestPoint = tpoints[interestPointIdx]

        # Compute direction vectors
        look_at = interestPoint - pos
        norm_lookat = np.linalg.norm(look_at)
        dX = look_at / norm_lookat
        
        dY = np.cross(self.viewUPvec, dX, axis=0)
        dZ = np.cross(dX, dY, axis=0)

        # Compute distance difference
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

            # Populate PoseStamped message
            pred_pose = PoseStamped()
            pred_pose.header.stamp = rospy.Time.now()
            pred_pose.header.frame_id = self.world_frame
            
            pred_pose.pose.position.x = command_pos[0]
            pred_pose.pose.position.y = command_pos[1]
            pred_pose.pose.position.z = odom_pose[2]
            
            pred_pose.pose.orientation.x = cqx
            pred_pose.pose.orientation.y = cqy
            pred_pose.pose.orientation.z = cqz
            pred_pose.pose.orientation.w = cqw
            
            pred_path.header.frame_id = self.world_frame
            pred_path.header.stamp = rospy.Time.now()
            pred_path.poses.append(pred_pose)

            logger.debug(f"{command_pos}, {pred_pos}")

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
        pred_path.header.stamp = rospy.Time.now()
        
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



        
        
        
        
    

        