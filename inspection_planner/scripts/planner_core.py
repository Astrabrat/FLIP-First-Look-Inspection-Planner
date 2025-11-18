#!/usr/bin/env python3

from header import * 
from utils import PlannerUtils
from inspection_quality import Snapshot

logger.add("inspection_loguru.log")

class PlannerCore():
    
    def __init__(self):
        self.cwd = os.getcwd()
        
        logger.info('Inspection Planner Initialized')

        self.initializeMissionParameters()
        
        self.initializeROSTopics()

        
    def initializeMissionParameters(self):
    
        self.desired_viewing_distance = rospy.get_param('~inspection_distance', 2.0)
        self.photogrammetric_params = rospy.get_param('~photogrammetric_params', [0.6, 0.8])
        self.fov = rospy.get_param('~fov', [69.4,45])
        self.platform_modality = rospy.get_param('~platform_modality', 0)
        self.min_pos_upd = rospy.get_param('~cmd_pos_upd', 0.2)
        self.min_yaw_upd = rospy.get_param('~cmd_yaw_upd', 0.05)
        self.sensor_rot = rospy.get_param('~sensor_rotation_S2B', [0.0, 0.0, 0.0])
        self.prediction_horizon = rospy.get_param('~prediction_horizon', 10)
        self.confidence_horizon = rospy.get_param('~confidence_horizon', 5)
        self.min_points_confidence = rospy.get_param('~min_pts_conf', 10)
        self.interp_alpha = rospy.get_param('~interpolation_alpha', 0.5)
        self.world_frame = rospy.get_param('~world_frame', 'world')
        self.baseLink_frame = rospy.get_param('~base_link_frame', 'base_link')
        self.max_std_deviation = rospy.get_param('~max_std_deviation', 0.05)
        self.run_mode = rospy.get_param('~run_mode', 0)

        self.ns = rospy.get_namespace()
        
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
        self.Upvec = [0 0 1]

        rospy.loginfo("Sucessfully loaded parameters")
        
    def initializeROSTopics(self):
        
        self.odom_topic = rospy.get_param("/odom_topic")
        self.pcl_topic = rospy.get_param("/pcl_topic")
        self.mission_plan_status_service_topic = rospy.get_param('/mission_plan_status_service_topic')
        self.initializeInspectionMissionService = rospy.get_param('/initialize_inspection_service')
        self.executeInspectionMissionService = rospy.get_param('/execute_inspection_service')
        
        self.inspDist_topic = rospy.get_param("/maintained_distance")
        self.reference_pose_topic = rospy.get_param("/reference_pose")
        predictedPath_topic_param = rospy.get_param("/predicted_path")
        
        self.croppedPoints_topic = rospy.get_param("/cropped_points")
        self.nearestNeighbour_topic = rospy.get_param("/nearest_neigbour")
        
        self.inspection_performance_topic = rospy.get_param("/inspection_performance")
        self.tracked_path_topic = rospy.get_param("/tracked_path")
        
        self.run_mode = rospy.get_param("/run_mode")
    
        rospy.Service(self.initializeInspectionMissionService,InitializeInspection,self.cb_initializeInspection)
        rospy.Service(self.executeInspectionMissionService,Trigger,self.cb_executeInspectionMission)

        self.pub_vieweingDistance = rospy.Publisher(self.inspDist_topic,Float64,queue_size=1)
        self.pub_cropped_points = rospy.Publisher(self.croppedPoints_topic,PointCloud2,queue_size=1)
        self.pub_nn_point = rospy.Publisher(self.nearestNeighbour_topic,PointCloud2,queue_size=1)
        self.pub_predPath = rospy.Publisher(predictedPath_topic_param,Path,queue_size=1)
        self.pub_refPose = rospy.Publisher(self.reference_pose_topic,PoseStamped,queue_size=1)
        

        rospy.Subscriber(self.odom_topic,Odometry,self.cb_odom,queue_size=1)
        rospy.wait_for_message(self.odom_topic,Odometry)
        
        rospy.Subscriber(self.pcl_topic,PointCloud2,self.cb_pointcloud,queue_size=1)
        rospy.wait_for_message(self.pcl_topic,PointCloud2)

        # Publish inspection quants
        self.pub_insp_performance = rospy.Publisher("inspection_planner/inspection_performance",InspectionPerformance,queue_size=1)
        self.path_pub = rospy.Publisher("inspection_planner/tracked_path",Path,queue_size=1)

        ## DEBUGGING 
        if self.run_mode == 0:
            rospy.Subscriber("/insp_start_flag",String,self.cb_start)
            self.path_pub = rospy.Publisher("inspection_planner/tracked_path",Path,queue_size=1)
            rospy.Subscriber("filtered_pointcloud_dbg",PointCloud2,self.cb_pointcloud,queue_size=1)
            self.pub_insp_performance = rospy.Publisher("inspection_planner/inspection_performance_dbg",InspectionPerformance,queue_size=1)
            self.res_start = True
            
            
        rospy.loginfo("Sucessfully loaded topics")

    def publish_path(self):
        self.path.header.stamp = self.now()
        self.path.header.frame_id = self.world_frame
        self.path_pub.publish(self.path)

    def cb_start(self, msg: String) -> None:
            self.res_start = True
            
    def cb_initializeInspection(self,srv):

        response = InitializeInspectionResponse()

        self.inspection_roi = srv.roi_polygons
        self.inspection_wps = srv.waypoints

        self.current_mission_status = 'Preliminary Parsing'

        response.status = True

        return response
    
    def cb_executeInspectionMission(self,srv):

        self.current_mission_status = f'Executing Mission'

        self.execute_plan = True

        return TriggerResponse(success=True)


    def cb_odom(self,data):

        px = data.pose.pose.position.x
        py = data.pose.pose.position.y
        pz = data.pose.pose.position.z
        
        qx = data.pose.pose.orientation.x
        qy = data.pose.pose.orientation.y
        qz = data.pose.pose.orientation.z
        qw = data.pose.pose.orientation.w
        
        self.odom_pose = np.array([px,py,pz,qx,qy,qz,qw])

        [r,p,yaw] = PlannerUtils.quat2eul(qx,qy,qz,qw)
        
        self.curr_yaw = yaw
        
    def cb_pointcloud(self,data):
        
        self.raw_pts = data

        self.pcl_stamp = data.header
        
        self.sensor_frame = data.header.frame_id

    @logger.catch
    def generateViewPose(self, pos, action, points,pose):

        cpoints,croppedPointsMsg = PlannerUtils.crop_points_within_fov(points,pose)
        
        self.pub_cropped_points.publish(croppedPointsMsg)
        
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
            
            # vov = (2 * self.mean_norm_LA * np.tan(np.deg2rad(self.fov[1]) / 2) -
            #     self.photogrammetric_params[1] * 2 * self.mean_norm_LA * np.tan(np.deg2rad(self.fov[1]) / 2))
            
            if self.switch:
                tcommand_pos = command_pos + dX * diff_view_dist - dY * hov + dZ * vov
            else:
                tcommand_pos = command_pos + dX * diff_view_dist + dY * hov + dZ * vov

            self.norm_LA.append(norm_lookat)
            self.mean_norm_LA = np.mean(self.norm_LA)
            
        elif action == "maintain":
            tcommand_pos = command_pos + dX * diff_view_dist

        elif action == "vertical":
            vov = (2 * self.mean_norm_LA * np.tan(np.deg2rad(self.fov[1]) / 2) -
                self.photogrammetric_params[1] * 2 * self.mean_norm_LA * np.tan(np.deg2rad(self.fov[1]) / 2))
            
            tcommand_pos = command_pos + dX * diff_view_dist + dZ * vov

        # Compute valid yaw angle
        valid_yaw = np.arctan2(dX[1], dX[0]) + self.sensor_rot[2]
        
        return tcommand_pos, valid_yaw
    

    def view_pred(self):
        """
        Predict the inspection view-pose by processing point cloud data and evaluating valid points.

        Returns:
                - pred_path: Path containing predicted poses.
                - refPose: Initial reference pose for the inspection.
                - cpos: Final command position.
                - cyaw: Final command yaw angle.
        """
        
        pred_path = Path()
        conf_flag = False
        nm_la = []

        # Evaluate valid points over observation horizon
        # while not conf_flag:
        #     odom_pos = self.odom_pose[0:3]
        #     for _ in range(self.confidence_horizon):
        #         self.curr_pts = point_cloud2.pointcloud2_to_xyz_array(self.raw_pts, remove_nans=True)
        #         nmla = self.evaluate_validPoints(self.curr_pts,odom_pos)
        #         nm_la.append(nmla)
                
        #         self.rate.sleep()

        #     std_deviation = np.std(nm_la, axis=0)
            
        #     std_dev_msg = Float64()
        #     std_dev_msg.data = std_deviation
        #     self.pub_confDev.publish(std_dev_msg)
            
        #     # print(f">>>> STDS {std_deviation}-{nmla}")

        #     if not (std_deviation >= self.max_std_deviation).all():
        #         conf_flag = True
                

        # # Wait for sufficient points
        # while len(self.curr_pts) < self.min_points_confidence:
        #     rospy.logwarn("[GECKO INSPECT] Not enough points to predict inspection view-pose")
            
        #     self.rate.sleep()
        
        pred_pos = self.odom_pose[0:3]  # Initialize predicted position
        lidar_points = point_cloud2.pointcloud2_to_xyz_array(self.raw_pts, remove_nans=True)
        
        predPose = self.odom_pose.copy()
        predPathArray = []
        for k in range(self.prediction_horizon):
            
            # Generate local view pose
            next_ref = list(self.bufferQ)[k][0:3]
            
            command_pos, command_yaw = self.generateViewPose(pred_pos, "inspect", lidar_points,predPose)

            [cqx, cqy, cqz, cqw] = PlannerUtils.eul2quat(0, 0, command_yaw)

            # Populate PoseStamped message
            pred_pose = PoseStamped()
            pred_pose.header.stamp = rospy.Time.now()
            pred_pose.header.frame_id = self.world_frame
            
            pred_pose.pose.position.x = command_pos[0]
            pred_pose.pose.position.y = command_pos[1]
            pred_pose.pose.position.z = next_ref[2]
            
            pred_pose.pose.orientation.x = cqx
            pred_pose.pose.orientation.y = cqy
            pred_pose.pose.orientation.z = cqz
            pred_pose.pose.orientation.w = cqw
            pred_path.header.frame_id = self.world_frame
            pred_path.header.stamp = rospy.Time.now()
            pred_path.poses.append(pred_pose)
            
            self.pub_predPath.publish(pred_path)

            # Save initial reference pose
            if k == 0:
                command_pos[2] = next_ref[2]
                cpos = command_pos
                cyaw = command_yaw
                refPose = pred_pose
                
            # logger.debug(f"{next_ref}, {pred_pos}")
            
            predPose = np.array([command_pos[0],command_pos[1],next_ref[2],cqx,cqy,cqz,cqw])
            
            predPathArray.append(predPose)

            # Update predicted position for the next iteration
            pred_pos = command_pos

        # Finalize predicted path
        pred_path.header.frame_id = self.world_frame
        pred_path.header.stamp = rospy.Time.now()

        return pred_path, predPathArray, refPose, cpos, cyaw


    def evaluate_validPoints(self,points, pred_pos):
        
        lidar_points = self.curr_pts
        
        tree = KDTree(lidar_points)
        dist, interestPointIdx = tree.query(pred_pos, k=1,workers=-1)

        interestPoint = lidar_points[interestPointIdx, :]

        look_at =  interestPoint - pred_pos

        norm_lookat = np.linalg.norm(look_at)
        
        return norm_lookat

    def normalize_angle(self,angle):
        """ Normalize an angle to the range [-180, 180]. """
        return (angle + np.pi) % (2 * np.pi) - np.pi

    def interpolate_yaw(self,current_yaw, target_yaw, alpha=0.5):

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
        current_yaw = self.normalize_angle(current_yaw)
        target_yaw = self.normalize_angle(target_yaw)
        
        # Calculate the difference
        delta_yaw = self.normalize_angle(target_yaw - current_yaw)

        # Interpolate
        interpolated_yaw = current_yaw + alpha * delta_yaw

        return self.normalize_angle(interpolated_yaw)
    
    def threshold_check(self,pose,yaw=None):
        
        flag = 0
        
        if self.platform_modality == 0:
            pose[2] = self.odom_pose[2]
            
        if yaw is not None:
            
            if np.linalg.norm(pose-self.odom_pose[0:3]) <= self.min_pos_upd and abs(yaw-self.curr_yaw) <= self.min_yaw_upd:

                flag = 1
        else:
            if np.linalg.norm(pose-self.odom_pose[0:3]) <= self.min_pos_upd:
                
                flag = 1
                
        
        return flag
        
    def transform2Global(self,pos,yaw):
        
        global_pos = np.array([0,0,0])

        global_pos[0] = self.odom_pose[0] + math.cos(self.curr_yaw)*pos[0] - math.sin(self.curr_yaw)*pos[1]
        global_pos[1] = self.odom_pose[1] + math.cos(self.curr_yaw)*pos[1] + math.sin(self.curr_yaw)*pos[0]
        global_pos[2] = self.odom_pose[2] + pos[2]

        global_yaw = self.curr_yaw + yaw

        return global_pos, global_yaw

    def update_yaw(self,path):
        
        egoPose = self.odom_pose.copy()
        ego = self.odom_pose[0:3]

        curr_points = point_cloud2.pointcloud2_to_xyz_array(self.raw_pts, remove_nans=True)
        
        next_ref = list(self.bufferQ)[0][0:3]

        _,command_yaw = self.generateViewPose(ego,"maintain",curr_points,egoPose)

        command_yaw = self.interpolate_yaw(self.curr_yaw, command_yaw, self.interp_alpha)

        command_yaw = command_yaw + self.sensor_rot[2]

        [cqx, cqy, cqz, cqw] = PlannerUtils.eul2quat(0, 0, command_yaw)

        path.poses[0].pose.orientation.x = cqx
        path.poses[0].pose.orientation.y = cqy
        path.poses[0].pose.orientation.z = cqz
        path.poses[0].pose.orientation.w = cqw

        path.header.stamp = rospy.Time.now()

        refPose = PoseStamped()
        refPose.header.stamp = rospy.Time.now()
        refPose.header.frame_id = self.world_frame

        refPose.pose = path.poses[0].pose
        
        return path,refPose,command_yaw

    def resetAll(self):

        self.norm_LA = []
        self.path = Path()
         
    def PublishInspectionPerformance(self):
        
        insp_perf = InspectionPerformance()
        
        insp_perf.header.stamp.secs = rospy.Time.now().secs
        insp_perf.view_planning_time.data = float(self.vp_time)
        insp_perf.maintained_distance.data = float(self.nearest_surface())
        insp_perf.desired_distance.data = float(self.desired_viewing_distance)
        insp_perf.view_quality = float(self.viewpose_quality)

        self.pub_insp_performance.publish(insp_perf)
        
        
    def main(self):
        
        while not rospy.is_shutdown():
            
            if self.start_flag:
                
                try:
                    t0 = time.perf_counter()
                    tpred_path, tpred_path_array, predRefPose, commandPos, tcommand_yaw = self.view_pred()
                    self.vp_time = time.perf_counter() - t0
                    logger.info(f"[View planning] Took: {self.vp_time:.3f} s")
                except Exception as e:
                    self.get_logger().error(f"view_pred failed: {e}")
                    
                self.PublishInspectionPerformance()
                
                self.path.poses.append(PlannerUtils.PoseArraytoPoseMsg(self.odom_pose.copy()))
                self.publish_path()
                self.pub_predPath.publish(tpred_path)
                self.pub_refPose.publish(predRefPose)

                while not self.threshold_check(np.array(commandPos), tcommand_yaw):
                    self.PublishInspectionPerformance()
                    tpred_path, predRefPose, tcommand_yaw = self.update_yaw(tpred_path)
                    self.pub_predPath.publish(tpred_path)
                    self.pub_refPose.publish(predRefPose)
                    self.rate.sleep()

                self.PublishInspectionPerformance()
                
            self.rate.sleep()

        
if __name__ == '__main__':
    
    rospy.init_node('inspection_planner') #inspection node
    
    try:
        PlannerCore()
        rospy.spin()
    except rospy.ROSInterruptException:
        pass


        
        
        
        
    

        