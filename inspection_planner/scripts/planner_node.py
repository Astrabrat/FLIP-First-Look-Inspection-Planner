#!/usr/bin/env python3
from header import *

from planner_core import PlannerCore
from utils import PlannerUtils

logger.add("inspection_node_loguru.log")

class PlannerNode():
    def __init__(self):
        self.cwd = os.getcwd()
        
        self.initializeMissionParameters()
        
        self.initializeROSTopics()

        # --- Instantiate non-ROS planner core ---
        self.planner = PlannerCore()
        
        logger.debug("Initializing Planning ...")
        
        self.main()

    def initializeMissionParameters(self):
    
        self.desired_viewing_distance = rospy.get_param('/inspection_distance', 2.0)
        self.platform_modality = rospy.get_param('/platform_modality', 0)
        self.min_pos_upd = rospy.get_param('/cmd_pos_upd', 0.2)
        self.min_yaw_upd = rospy.get_param('/cmd_yaw_upd', 0.05)
        self.world_frame = rospy.get_param('/world_frame', 'world')
        self.run_mode = rospy.get_param('/run_mode', 1)
        self.sensor_rot = rospy.get_param('/sensor_rotation', [0.0, 0.0, 0.0])
        self.rate_controller = rospy.get_param('/rate_controller', 10)
        
        self.rate = rospy.Rate(self.rate_controller)

        self.res_start = False
        self.execute_plan = False
        self.vp_time = 0.0
        self.path = Path()
        
        self.current_mission_status = 'Initialization'
    
        rospy.loginfo("Sucessfully loaded parameters")

    def initializeROSTopics(self):
        
        self.odom_topic = rospy.get_param("/odom_topic")
        self.pcl_topic = rospy.get_param("/pcl_topic")
        
        self.inspDist_topic = rospy.get_param("/maintained_distance")
        self.reference_pose_topic = rospy.get_param("/reference_pose")
        predictedPath_topic_param = rospy.get_param("/predicted_path")
        
        self.croppedPoints_topic = rospy.get_param("/cropped_points")
        # self.nearestNeighbour_topic = rospy.get_param("/nearest_neigbour")
        
        self.inspection_performance_topic = rospy.get_param("/inspection_performance")
        self.tracked_path_topic = rospy.get_param("/tracked_path")
        
        self.run_mode = rospy.get_param("/run_mode")
    
        self.pub_vieweingDistance = rospy.Publisher(self.inspDist_topic,Float64,queue_size=1)
        self.pub_cropped_points = rospy.Publisher(self.croppedPoints_topic,PointCloud2,queue_size=1)
        # self.pub_nn_point = rospy.Publisher(self.nearestNeighbour_topic,PointCloud2,queue_size=1)
        self.pub_predPath = rospy.Publisher(predictedPath_topic_param,Path,queue_size=1)
        self.pub_refPose = rospy.Publisher(self.reference_pose_topic,PoseStamped,queue_size=1)
        
        rospy.Subscriber(self.odom_topic,Odometry,self.cb_odom,queue_size=1)
        rospy.wait_for_message(self.odom_topic,Odometry)
        logger.info("Odometry received")
        
        rospy.Subscriber(self.pcl_topic,PointCloud2,self.cb_pointcloud,queue_size=1)
        rospy.wait_for_message(self.pcl_topic,PointCloud2)
        
        print(self.pcl_topic)
        logger.info("Pointcloud received")
        
        rospy.Service("initialize_inspection",Trigger,self.cb_start)
        
        ## Viswa Control Law 
        self.cbfPolicy  = rospy.ServiceProxy('cbf_input', Trigger)

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

    def cb_start(self, srv):
        
        self.res_start = True
        return TriggerResponse(success=True)
        
            
    def cb_pointcloud(self,data):
        
        self.raw_pts = data

        self.pcl_stamp = data.header
        
        self.sensor_frame = data.header.frame_id
        
    def cb_odom(self,data):

        px = data.pose.pose.position.x
        py = data.pose.pose.position.y
        pz = data.pose.pose.position.z
        
        qx = data.pose.pose.orientation.x
        qy = data.pose.pose.orientation.y
        qz = data.pose.pose.orientation.z
        qw = data.pose.pose.orientation.w

        [r,p,yaw] = PlannerUtils.quat2eul(qx,qy,qz,qw)

        # self.curr_yaw = yaw + self.sensor_rot[2]
        self.curr_yaw = yaw 
        
        # rqx,rqy,rqz,rqw = PlannerUtils.eul2quat(0,0,self.curr_yaw)
        
        self.odom_pose = np.array([px,py,pz,qx,qy,qz,qw])
        

    def viewPredPolicy(self):
        
        """
        Predict the inspection view-pose by processing point cloud data and evaluating valid points.

        Returns:
                - pred_path: Path containing predicted poses.
                - refPose: Initial reference pose for the inspection.
                - cpos: Final command position.
                - cyaw: Final command yaw angle.
        """
    
        lidar_points = point_cloud2.pointcloud2_to_xyz_array(self.raw_pts, remove_nans=True)
        
        currPose = self.odom_pose.copy()

        pred_path, predPathArray, refPose, cpos, cyaw = self.planner.standardViewPolicy(self.pub_cropped_points,lidar_points=lidar_points, odom_pose=currPose)
        
        self.pub_predPath.publish(pred_path)
        
        return pred_path, predPathArray, refPose, cpos, cyaw
     

    def thresholdCheck(self,pose,yaw=None):
        
        if self.platform_modality == 0:
            pose[2] = self.odom_pose[2]
            
        if yaw is not None:
            
            # try:
            #     resp =  self.cbfPolicy()
            # except rospy.ServiceException as e:
            #     logger.warning(f"CBF Service call failed: {e}")
            #     resp = None
                
            # ## Fallback incase cbf service fails
            # if resp is not None and resp.success:
                
            logger.debug("[Case 1] CBF Policy Triggered")
            
            if self.sensor_rot[2] != 0.0:
                curr_yaw = self.curr_yaw + self.sensor_rot[2]
                curr_yaw = self.planner.normalizeAngle(curr_yaw)
            else:
                curr_yaw = self.curr_yaw
                
            if np.linalg.norm(pose-self.odom_pose[0:3]) <= self.min_pos_upd and abs(yaw-curr_yaw) <= self.min_yaw_upd: ## Check the condition whre the CBF return True but the Yaw is not satisfied (in nominal as well as obstacle case)

                return True
            # else:
            #     logger.warning(f"CBF responded {resp} but thresholds not met")
            
        else:
            # try:
            #     resp =  self.cbfPolicy()
            # except rospy.ServiceException as e:
            #     logger.warning(f"CBF Service call failed: {e}")
            #     resp = None
                
            ## Fallback incase cbf service fails
            # if resp is not None and resp.success:
                
            logger.debug("[Case 2] CBF Policy Triggered")

            if np.linalg.norm(pose-self.odom_pose[0:3]) <= self.min_pos_upd:
                
                return True
            # else:
            #     logger.warning(f"CBF responded {resp} but thresholds not met")
                                            
        return False
                
        
    def updateYaw(self,path,pcl_pub_handle):
        
        currPose = self.odom_pose.copy()
        currPos = currPose[0:3]

        if self.sensor_rot[2] != 0.0:
            
            currPose = self.planner.getRotatedOdomYaw(self.odom_pose.copy(),self.sensor_rot[2])
            _,_,currYaw = PlannerUtils.quat2eul(currPose[3],currPose[4],currPose[5],currPose[6])
            currPos = currPose[0:3]
        else:
            currYaw = self.curr_yaw

        curr_points = point_cloud2.pointcloud2_to_xyz_array(self.raw_pts, remove_nans=True)

        _,command_yaw = self.planner.generateViewPose(currPos,"maintain",curr_points,currPose,pcl_pub_handle)

        command_yaw = self.planner.interpolateYaw(currYaw, command_yaw)

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
    
    def publishInspectionPerformance(self):
        
        
        insp_perf = InspectionPerformance()
        
        insp_perf.header.stamp.secs = rospy.Time.now().secs
        insp_perf.view_planning_time.data = float(self.vp_time)
        nmla_info, croppedPointsMsg = self.planner.nearest_surface(self.raw_pts,self.odom_pose.copy())
        insp_perf.maintained_distance.data = float(nmla_info)
        insp_perf.desired_distance.data = float(self.desired_viewing_distance)
        # insp_perf.view_quality.data = float(self.planner.viewpose_quality)

        self.pub_insp_performance.publish(insp_perf)
        self.pub_cropped_points.publish(croppedPointsMsg)

    def publish_path(self):
        self.path.header.stamp = rospy.Time.now()
        self.path.header.frame_id = self.world_frame
        self.path_pub.publish(self.path)

    def modifyReferenceYaw(self, refPose, yaw_offset):
        [_,_,current_yaw] = PlannerUtils.quat2eul(refPose.pose.orientation.x,
                                                  refPose.pose.orientation.y,
                                                  refPose.pose.orientation.z,
                                                  refPose.pose.orientation.w)
        
        modified_yaw = current_yaw - yaw_offset
        
        modified_yaw = self.planner.normalizeAngle(modified_yaw)
        
        logger.debug(f"Modifying reference yaw from {current_yaw:.3f} to {modified_yaw:.3f} with an offet of {yaw_offset} radians")
        
        [mqx, mqy, mqz, mqw] = PlannerUtils.eul2quat(0, 0, modified_yaw)
        
        refPose.pose.orientation.x = mqx
        refPose.pose.orientation.y = mqy
        refPose.pose.orientation.z = mqz
        refPose.pose.orientation.w = mqw
        
        self.pub_refPose.publish(refPose)
        
    def main(self):
        
        while not rospy.is_shutdown():
            
            nmla_info, croppedPointsMsg = self.planner.nearest_surface(self.raw_pts,self.odom_pose.copy())
            self.pub_cropped_points.publish(croppedPointsMsg)

            if self.res_start:
                
                try:
                    t0 = tic()
                    tpred_path, tpred_path_array, predRefPose, commandPos, tcommand_yaw = self.viewPredPolicy()
                    self.vp_time = toc()
                    logger.debug(f"[View planning] Took: {self.vp_time:.3f} s")
                except Exception as e:
                    logger.warning(f"View planning failed: {e}")
                    
                self.publishInspectionPerformance()
                
                self.path.poses.append(PlannerUtils.PoseArraytoPoseMsg(self.odom_pose.copy()))
                self.publish_path()
                self.pub_predPath.publish(tpred_path)
                if self.sensor_rot[2] != 0: # rotated case, go for modified publishing
                    self.modifyReferenceYaw(predRefPose, self.sensor_rot[2])
                else: # nominal case, go for direct publishing
                    self.pub_refPose.publish(predRefPose)

                while not self.thresholdCheck(np.array(commandPos), tcommand_yaw):
                    self.publishInspectionPerformance()
                    tpred_path, predRefPose, tcommand_yaw = self.updateYaw(tpred_path,self.pub_cropped_points)
                    self.pub_predPath.publish(tpred_path)
                    
                    if self.sensor_rot[2] != 0: # rotated case, go for modified publishing
                        self.modifyReferenceYaw(predRefPose, self.sensor_rot[2])
                    else: # nominal case, go for direct publishing
                        self.pub_refPose.publish(predRefPose)
                        
                    # self.pub_refPose.publish(predRefPose)
                    self.rate.sleep()
                
            self.rate.sleep()


if __name__ == '__main__':
    
    rospy.init_node('inspection_planner_node') #inspection node
    
    try:
        PlannerNode()
        rospy.spin()
    except rospy.ROSInterruptException:
        pass