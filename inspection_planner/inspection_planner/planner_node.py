#!/usr/bin/env python3
"""
ROS2 node for FLIP inspection planner.

"""

from inspection_planner.header import *  # TODO: Replace with explicit imports

from inspection_planner.planner_core import PlannerCore
from inspection_planner.utils import PlannerUtils
import threading

logger.add("inspection_node_loguru.log")


# ============================================================================
# CONSTANTS
# ============================================================================
class Constants:
    """Configuration constants for inspection planner node."""
    STARTUP_TIMEOUT_SEC = 5.0        # Maximum wait time for sensor data
    MIN_TIMER_RATE_HZ = 1.0         # Minimum timer frequency
    MAX_PATH_HISTORY = 1000         # Maximum tracked path poses
    POINT_CLOUD_CACHE_STAMP = None  # Cache stamp for point cloud


def _strip_leading_slash(name: str) -> str:
    """Strip leading slash from parameter name."""
    return name[1:] if name.startswith('/') else name


class InspectionPlannerNode(Node):
    """ROS2 node for inspection view pose planning."""

    def __init__(self):
        super().__init__(
            'inspection_planner_node',
        )
        self._tick_group = MutuallyExclusiveCallbackGroup()
        # Thread safety lock for shared state
        self._state_lock = threading.Lock()
        
        PlannerUtils.set_node(self)

        self.sensor = PlannerUtils.LoadSensorParams()

        self.declare_mission_params()
        self.initializeMissionParameters()
        self.initializeROSTopics()

        # Instantiate core planner (with param/time access via node)
        self.planner = PlannerCore(config=self.params, node=self)

        logger.debug("Initializing Planning ...")

        # State - protected by _state_lock
        self._have_pred = False
        self._last_pred_path = None
        self._last_pred_refpose = None
        self._last_command_pos = None
        self._last_command_yaw = None
        
        # Point cloud cache
        self._cached_lidar_points = None
        self._last_pcl_stamp = None

        # Main loop timer
        # Dedicated callback group so the timer cannot overlap with itself
        

        # Create timer using declared param
        self._make_or_update_timer()
        self.add_on_set_parameters_callback(self._on_param_change)

    # -----------------
    # Param helpers
    # -----------------
    def _make_or_update_timer(self):
        """Create (or recreate) the tick timer based on current rate_controller."""
        rate = float(self.get_param('/rate_controller', 5.0))

        # Clamp to a safe range
        if rate <= 0.0:
            logger.warning(f"rate_controller <= 0 ({rate}); clamping to {Constants.MIN_TIMER_RATE_HZ} Hz")
            rate = Constants.MIN_TIMER_RATE_HZ

        period = 1.0 / rate

        # If timer exists, destroy and recreate
        if hasattr(self, "timer") and self.timer is not None:
            try:
                self.timer.cancel()
                self.destroy_timer(self.timer)
            except Exception as e:
                logger.warning(f"Failed to destroy old timer: {e}")

        self.timer = self.create_timer(period, self._tick, callback_group=self._tick_group)
        self.rate_controller = rate
        logger.info(f"Tick rate set to {rate:.2f} Hz (period {period:.3f} s)")

    def _on_param_change(self, params):
        """Allow runtime change of rate_controller."""
        for p in params:
            if p.name in ("rate_controller", "/rate_controller"):
                # accept anything numeric; clamp later
                try:
                    _ = float(p.value)
                except Exception:
                    return SetParametersResult(successful=False, reason="rate_controller must be numeric")

        # Apply changes after validation
        result = SetParametersResult(successful=True)

        # Rebuild timer if rate_controller changed
        for p in params:
            if p.name == "rate_controller":
                self._make_or_update_timer()

        return result
    
    def get_param(self, name: str, default):
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

    # -----------------
    # Initialization
    # -----------------

    def declare_mission_params(self):
        """Declare all mission parameters with defaults."""
        # Scalars
        self.declare_parameter('inspection_distance', 2.0)
        self.declare_parameter('platform_modality', 0)
        self.declare_parameter('cmd_pos_upd', 0.2)
        self.declare_parameter('cmd_yaw_upd', 0.05)
        self.declare_parameter('prediction_horizon', 5)
        self.declare_parameter('confidence_horizon', 5)
        self.declare_parameter('world_frame', 'world')
        self.declare_parameter('base_link_frame', 'base_link')
        self.declare_parameter('run_mode', 0)
        self.declare_parameter('inspection_height', 1.0)
        self.declare_parameter('rate_controller', 1.0)

        # Lists
        self.declare_parameter('photogrammetric_params', [0.8, 0.8])
        self.declare_parameter('fov', [69.4, 45.0])
        self.declare_parameter('sensor_rotation', [0.0, 0.0, 0.0])
        self.declare_parameter('sensor_translation', [0.0, 0.0, 0.0])

        self.declare_parameter('odom_topic', 'odometry/imu')
        self.declare_parameter('pcl_topic', 'filtered_pointcloud')
        self.declare_parameter('sbl_topic', 'sbl/data')

        # Outputs
        self.declare_parameter('maintained_distance', 'inspection_planner/results/maintained_distance')
        self.declare_parameter('reference_pose', 'command/pose')
        self.declare_parameter('predicted_path', 'inspection_planner/results/predicted_path')
        self.declare_parameter('cropped_points', 'inspection_planner/results/cropped_points')
        self.declare_parameter('inspection_performance', 'inspection_planner/results/inspection_performance')
        self.declare_parameter('tracked_path', 'inspection_planner/results/tracked_path')
        self.declare_parameter('cam_frustum', 'inspection_planner/general/camera_frustum')

        logger.info('All params declared')

    def initializeMissionParameters(self):
        """Initialize and cache mission parameters."""
        
        ## Store params to feed PlannerCore
        self.params = {
            "inspection_distance": float(self.get_parameter('inspection_distance').value),

            "photogrammetric_params": [
                float(x) for x in self.get_parameter('photogrammetric_params').value
            ],

            "fov": [
                float(x) for x in self.get_parameter('fov').value
            ],

            "platform_modality": int(self.get_parameter('platform_modality').value),

            "cmd_pos_upd": float(self.get_parameter('cmd_pos_upd').value),
            "cmd_yaw_upd": float(self.get_parameter('cmd_yaw_upd').value),

            "sensor_rotation": [
                float(x) for x in self.get_parameter('sensor_rotation').value
            ],
            "sensor_translation": [
                float(x) for x in self.get_parameter('sensor_translation').value
            ],

            "prediction_horizon": max(1, int(self.get_parameter('prediction_horizon').value)),
            "confidence_horizon": max(1, int(self.get_parameter('confidence_horizon').value)),

            "world_frame": str(self.get_parameter('world_frame').value),
            "base_link_frame": str(self.get_parameter('base_link_frame').value),

            "run_mode": int(self.get_parameter('run_mode').value),
            "inspection_height": float(self.get_parameter('inspection_height').value),
        }

        # convenience mirrors (optional)
        self.desired_viewing_distance = self.params["inspection_distance"]
        self.platform_modality = self.params["platform_modality"]
        self.min_pos_upd = self.params["cmd_pos_upd"]
        self.min_yaw_upd = self.params["cmd_yaw_upd"]
        self.sensor_rot = self.params["sensor_rotation"]
        self.sensor_trans = self.params["sensor_translation"]
        self.world_frame = self.params["world_frame"]
        self.run_mode = self.params["run_mode"]
        self.inspection_height = self.params["inspection_height"]
        
        # Cast to float, not int (preserves precision)
        self.rate_controller = float(self.get_parameter('rate_controller').value) if self.has_parameter('rate_controller') else 5.0

        logger.info("Successfully loaded parameters")

        # Initialize pz before first use
        self.pz = 0.0
        
        self.res_start = False
        self.execute_plan = False
        self.vp_time = 0.0
        
        # Bound path history
        self.path = Path()
        self._path_count = 0  # Track poses count

        self.current_mission_status = 'Initialization'

        self._have_odom = False
        self._have_pcl = False
        self.odom_pose = None
        self.curr_yaw = 0.0
        self.raw_pts = None
        self.res_start = True

    def initializeROSTopics(self):
        """Initialize ROS2 topics and services."""
        self.odom_topic = str(self.get_parameter('odom_topic').value)
        self.pcl_topic = str(self.get_parameter('pcl_topic').value)
        self.sbl_topic = str(self.get_parameter('sbl_topic').value)

        logger.info(f"odom_topic: {self.odom_topic}")
        logger.info(f"pcl_topic:  {self.pcl_topic}")

        # Outputs
        self.inspDist_topic = str(self.get_parameter('maintained_distance').value)
        self.reference_pose_topic = str(self.get_parameter('reference_pose').value)
        predictedPath_topic_param = str(self.get_parameter('predicted_path').value)
        self.croppedPoints_topic = str(self.get_parameter('cropped_points').value)
        self.inspection_performance_topic = str(self.get_parameter('inspection_performance').value)
        self.tracked_path_topic = str(self.get_parameter('tracked_path').value)
        self.cam_frustum_topic = str(self.get_parameter('cam_frustum').value)

        # Keep run_mode consistent
        self.run_mode = int(self.get_parameter('run_mode').value)

        qos1 = QoSProfile(depth=1)

        self.pub_vieweingDistance = self.create_publisher(Float64, self.inspDist_topic, qos1)
        self.pub_cropped_points = self.create_publisher(PointCloud2, self.croppedPoints_topic, qos1)
        self.pub_predPath = self.create_publisher(Path, predictedPath_topic_param, qos1)
        self.pub_refPose = self.create_publisher(PoseStamped, self.reference_pose_topic, qos1)

        self.pub_insp_performance = self.create_publisher(InspectionPerformance, self.inspection_performance_topic, qos1)
        self.path_pub = self.create_publisher(Path, self.tracked_path_topic, qos1)
        self.pub_frustum = self.create_publisher(MarkerArray, self.cam_frustum_topic, qos1)

        # Use same callback group for ALL subscriptions to prevent race conditions
        self.create_subscription(
            Odometry, self.odom_topic, self.cb_odom, QOS_SENSOR, 
            callback_group=self._tick_group
        )
        self.create_subscription(
            PointCloud2, self.pcl_topic, self.cb_pointcloud, QOS_SENSOR,
            callback_group=self._tick_group
        )
        self.create_subscription(
            Float64, self.sbl_topic, self.cb_sbl, QOS_SENSOR,
            callback_group=self._tick_group
        )

        # Start service
        self.create_service(
            Trigger, 'initialize_inspection', self.cb_start,
            callback_group=self._tick_group
        )

        self._wait_for_initial_messages(timeout_sec=Constants.STARTUP_TIMEOUT_SEC)

        logger.info("Successfully loaded topics")

    def _wait_for_initial_messages(self, timeout_sec: float = Constants.STARTUP_TIMEOUT_SEC):
        """Wait for initial sensor messages with timeout."""
        t0 = time.time()
        while rclpy.ok() and (not self._have_odom or not self._have_pcl):
            rclpy.spin_once(self, timeout_sec=0.1)
            if time.time() - t0 > timeout_sec:
                # Log warning but continue - node remains alive
                if not self._have_odom:
                    logger.warning(f"No odometry received on '{self.odom_topic}' after {timeout_sec}s")
                if not self._have_pcl:
                    logger.warning(f"No pointcloud received on '{self.pcl_topic}' after {timeout_sec}s")
                break

    # -----------------
    # ROS callbacks
    # -----------------

    def cb_sbl(self, msg: Float64):
        """Handle SBL altitude data."""
        with self._state_lock:
            self.pz = msg.data

    def cb_start(self, request, response):
        """Handle inspection start service."""
        with self._state_lock:
            self.res_start = True
        response.success = True
        response.message = "inspection initialized"
        return response

    def cb_pointcloud(self, data: PointCloud2):
        """Handle incoming point cloud data."""
        with self._state_lock:
            self.raw_pts = data
            self.pcl_stamp = data.header
            self.sensor_frame = data.header.frame_id
            self._have_pcl = True
            
            # Cache converted point cloud to avoid redundant conversions
            if self._last_pcl_stamp != data.header.stamp:
                self._cached_lidar_points = None  # Reset cache
                self._last_pcl_stamp = data.header.stamp

    def cb_odom(self, data: Odometry):
        """Handle incoming odometry data."""
        px = data.pose.pose.position.x
        py = data.pose.pose.position.y
        pz = data.pose.pose.position.z

        qx = data.pose.pose.orientation.x
        qy = data.pose.pose.orientation.y
        qz = data.pose.pose.orientation.z
        qw = data.pose.pose.orientation.w

        [r, p, yaw] = PlannerUtils.quat2eul(qx, qy, qz, qw)

        with self._state_lock:
            self.curr_yaw = yaw
            if self.platform_modality == 1:
                self.odom_pose = np.array([px, py, self.pz, qx, qy, qz, qw])
            else:
                self.odom_pose = np.array([px, py, pz, qx, qy, qz, qw])
            self._have_odom = True

    # -----------------
    # Core logic
    # -----------------
    def viewPredPolicy(self):
        """Generate view prediction policy."""
        # Use cached point cloud
        with self._state_lock:
            if self._cached_lidar_points is None:
                self._cached_lidar_points = PlannerUtils.pc2_to_xyz(self.raw_pts)
            lidar_points = self._cached_lidar_points.copy()
            currPose = self.odom_pose.copy()

        try:
            pred_path, predPathArray, refPose, cpos, cyaw = self.planner.standardViewPolicy(
                self.pub_cropped_points,
                lidar_points=lidar_points,
                odom_pose=currPose,
            )
            
            # Handle None returns
            if pred_path is None:
                logger.error("standardViewPolicy returned None")
                return None, None, None, None, None
            
            self.pub_predPath.publish(pred_path)
            return pred_path, predPathArray, refPose, cpos, cyaw
        except Exception as e:
            logger.error(f"viewPredPolicy failed: {e}")
            raise  # Re-raise to let caller handle

    def thresholdCheck(self, pose, yaw=None):
        """Check if pose and yaw meet update thresholds."""
        with self._state_lock:
            if self.platform_modality == 0:  # ground robot
                pose[2] = self.odom_pose[2]
            elif self.platform_modality == 1:  # aerial robot
                pose[2] = self.inspection_height

            if yaw is not None:
                if self.sensor_rot[2] != 0.0:
                    curr_yaw = self.curr_yaw + self.sensor_rot[2]
                    curr_yaw = self.planner.normalizeAngle(curr_yaw)
                else:
                    curr_yaw = self.curr_yaw

                if np.linalg.norm(pose - self.odom_pose[0:3]) <= self.min_pos_upd and abs(yaw - curr_yaw) <= self.min_yaw_upd:
                    return True
            else:
                logger.debug("[Case 2] CBF Policy Triggered")
                if np.linalg.norm(pose - self.odom_pose[0:3]) <= self.min_pos_upd:
                    return True

        return False

    def updateYaw(self, path, pcl_pub_handle):
        """Update yaw in predicted path."""
        # Validate path has poses
        if path is None or len(path.poses) == 0:
            logger.error("updateYaw called with empty path")
            return None, None, None
        
        with self._state_lock:
            currPose = self.odom_pose.copy()
            currPos = currPose[0:3]

            if self.sensor_rot[2] != 0.0:
                currPose = self.planner.getRotatedOdomYaw(self.odom_pose.copy(), self.sensor_rot[2])
                _, _, currYaw = PlannerUtils.quat2eul(currPose[3], currPose[4], currPose[5], currPose[6])
                currPos = currPose[0:3]
            else:
                currYaw = self.curr_yaw

            # Use cached point cloud
            if self._cached_lidar_points is None:
                self._cached_lidar_points = PlannerUtils.pc2_to_xyz(self.raw_pts)
            curr_points = self._cached_lidar_points.copy()

        try:
            _, command_yaw = self.planner.generateViewPose(currPos, "maintain", curr_points, currPose, pcl_pub_handle)

            if command_yaw is None:
                logger.error("generateViewPose returned None yaw")
                return path, None, None

            command_yaw = self.planner.interpolateYaw(currYaw, command_yaw)

            [cqx, cqy, cqz, cqw] = PlannerUtils.eul2quat(0, 0, command_yaw)

            path.poses[0].pose.orientation.x = cqx
            path.poses[0].pose.orientation.y = cqy
            path.poses[0].pose.orientation.z = cqz
            path.poses[0].pose.orientation.w = cqw

            now_msg = self.get_clock().now().to_msg()
            path.header.stamp = now_msg

            refPose = PoseStamped()
            refPose.header.stamp = now_msg
            refPose.header.frame_id = self.world_frame
            refPose.pose = path.poses[0].pose

            return path, refPose, command_yaw
        except Exception as e:
            logger.error(f"updateYaw failed: {e}")
            return path, None, None

    def publishInspectionPerformance(self):
        """Publish inspection performance metrics."""
        with self._state_lock:
            if self.raw_pts is None or self.odom_pose is None:
                return

            odom_pose_copy = self.odom_pose.copy()
            raw_pts_copy = self.raw_pts
            curr_yaw_copy = self.curr_yaw

        insp_perf = InspectionPerformance()
        insp_perf.header.stamp = self.get_clock().now().to_msg()

        insp_perf.view_planning_time.data = float(self.vp_time)
        
        try:
            nmla_info, croppedPointsMsg = self.planner.nearest_surface(
                raw_pts_copy, odom_pose_copy, curr_yaw_copy
            )
            insp_perf.maintained_distance.data = float(nmla_info)

            # Keep distance live-updated
            self.desired_viewing_distance = float(self.get_parameter('inspection_distance').value)
            
            insp_perf.desired_distance.data = float(self.desired_viewing_distance)

            self.pub_insp_performance.publish(insp_perf)
            self.pub_cropped_points.publish(croppedPointsMsg)
        except Exception as e:
            logger.warning(f"publishInspectionPerformance failed: {e}")

    def publish_path(self):
        """Publish tracked path."""
        self.path.header.stamp = self.get_clock().now().to_msg()
        self.path.header.frame_id = self.world_frame
        self.path_pub.publish(self.path)

    def modifyReferenceYaw(self, refPose, yaw_offset):
        """Modify reference pose yaw by offset."""
        [_, _, current_yaw] = PlannerUtils.quat2eul(
            refPose.pose.orientation.x,
            refPose.pose.orientation.y,
            refPose.pose.orientation.z,
            refPose.pose.orientation.w,
        )

        modified_yaw = current_yaw + yaw_offset
        modified_yaw = self.planner.normalizeAngle(modified_yaw)

        [mqx, mqy, mqz, mqw] = PlannerUtils.eul2quat(0, 0, modified_yaw)

        refPose.pose.orientation.x = mqx
        refPose.pose.orientation.y = mqy
        refPose.pose.orientation.z = mqz
        refPose.pose.orientation.w = mqw

        return refPose

    def visualizeFootprint(self, path: Path):
        """Visualize camera frustum footprint."""
        frustumMsg = MarkerArray()
        counter = 0
        for pose in path.poses:
            state = np.array([
                pose.pose.position.x,
                pose.pose.position.y,
                pose.pose.position.z,
                pose.pose.orientation.x,
                pose.pose.orientation.y,
                pose.pose.orientation.z,
                pose.pose.orientation.w
            ])
            markerMsg = self.sensor.get_frustum(state, self.sensor_rot, self.sensor_trans, counter)
            markerMsg.header.frame_id = "world"
            markerMsg.header.stamp = self.get_clock().now().to_msg()

            frustumMsg.markers.append(markerMsg)
            counter += 1

        self.pub_frustum.publish(frustumMsg)

    # -----------------
    # Timer tick
    # -----------------
    def _tick(self):
        """Main timer callback - process sensor data and generate commands."""
        # Copy shared state atomically
        with self._state_lock:
            if self.raw_pts is None or self.odom_pose is None:
                return
            raw_pts_copy = self.raw_pts
            odom_pose_copy = self.odom_pose.copy()
            curr_yaw_copy = self.curr_yaw
            res_start_copy = self.res_start

        # Keep publishing cropped points for visibility
        try:
            _, croppedPointsMsg = self.planner.nearest_surface(raw_pts_copy, odom_pose_copy, curr_yaw_copy)
            self.pub_cropped_points.publish(croppedPointsMsg)
        except Exception as e:
            logger.warning(f"nearest_surface failed: {e}")

        if not res_start_copy:
            return

        # Compute a new view plan once, then update yaw until thresholds met.
        if not self._have_pred:
            try:
                t0 = tic()
                tpred_path, tpred_path_array, predRefPose, commandPos, tcommand_yaw = self.viewPredPolicy()
                
                # Handle None returns
                if tpred_path is None:
                    logger.error("viewPredPolicy returned None, skipping this tick")
                    return
                
                self.vp_time = toc()
                logger.debug(f"[View planning] Took: {self.vp_time:.3f} s")
                
                odomMsg = PlannerUtils.PoseArraytoPoseMsg(odom_pose_copy)
                refPose = self.modifyReferenceYaw(odomMsg, self.sensor_rot[2])
                tpred_path.poses.insert(0, refPose)
                self.visualizeFootprint(tpred_path)

                with self._state_lock:
                    self._last_pred_path = tpred_path
                    self._last_pred_refpose = predRefPose
                    self._last_command_pos = np.array(commandPos)
                    self._last_command_yaw = tcommand_yaw
                    self._have_pred = True
            except Exception as e:
                logger.error(f"View planning failed: {e}")
                return

        # Publish performance + tracked path
        self.publishInspectionPerformance()

        try:
            # Bound path history
            with self._state_lock:
                pose_msg = PlannerUtils.PoseArraytoPoseMsg(self.odom_pose.copy())
                self.path.poses.append(pose_msg)
                self._path_count += 1
                
                # Limit history to prevent memory leak
                if self._path_count > Constants.MAX_PATH_HISTORY:
                    self.path.poses = self.path.poses[-Constants.MAX_PATH_HISTORY:]
                    self._path_count = len(self.path.poses)
                
                self.publish_path()
        except Exception as e:
            logger.warning(f"publish_path failed: {e}")

        # Publish current command outputs
        with self._state_lock:
            last_pred_path = self._last_pred_path
            last_pred_refpose = self._last_pred_refpose
            last_command_pos = self._last_command_pos
            last_command_yaw = self._last_command_yaw

        if last_pred_path is not None:
            self.pub_predPath.publish(last_pred_path)

        if last_pred_refpose is not None:
            if self.sensor_rot[2] != 0:
                refPose = self.modifyReferenceYaw(last_pred_refpose, self.sensor_rot[2])
                self.pub_refPose.publish(refPose)
            else:
                self.pub_refPose.publish(last_pred_refpose)

        # If thresholds not met, update yaw and keep going next tick.
        if last_command_pos is None:
            return

        if self.run_mode == 1:
            if not self.thresholdCheck(last_command_pos.copy(), last_command_yaw):
                try:
                    updated_path, updated_refpose, updated_yaw = self.updateYaw(
                        last_pred_path,
                        self.pub_cropped_points,
                    )
                    
                    if updated_path is not None:
                        with self._state_lock:
                            self._last_pred_path = updated_path
                            self._last_pred_refpose = updated_refpose
                            self._last_command_yaw = updated_yaw
                        
                        self.pub_predPath.publish(updated_path)

                        if self.sensor_rot[2] != 0:
                            refPose = self.modifyReferenceYaw(updated_refpose, self.sensor_rot[2])
                            self.pub_refPose.publish(refPose)
                        else:
                            self.pub_refPose.publish(updated_refpose)
                except Exception as e:
                    logger.error(f"updateYaw failed: {e}")
            else:
                with self._state_lock:
                    self._have_pred = False
        else:
            with self._state_lock:
                self._have_pred = False


def main(args=None):
    rclpy.init(args=args)
    node = InspectionPlannerNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
