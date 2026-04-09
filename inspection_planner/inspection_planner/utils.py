"""
Utility functions for FLIP inspection planner.

"""

from inspection_planner.header import *  # TODO: Replace with explicit imports
import math


# ============================================================================
# CONSTANTS
# ============================================================================
class Constants:
    """Numerical constants for utility functions."""
    NORMALIZATION_EPSILON = 1e-8  # Minimum norm to avoid division by zero


@dataclass
class Plane:
    """Represents a geometric plane."""
    point: np.ndarray
    normal: np.ndarray


@dataclass
class Camera:
    """Represents camera intrinsic properties."""
    Position: np.ndarray
    Front: np.ndarray
    Right: np.ndarray
    Up: np.ndarray


# NOTE: Frustum dataclass removed - was never used


class SequenceIterator:
    """Iterator for sequence access with index tracking."""
    
    def __init__(self, sequence):
        self._sequence = sequence
        self._index = 0
        self.current_index = 0

    def __iter__(self):
        return self

    def __next__(self):
        if self._index < len(self._sequence):
            self.current_index = self._index
            item = self._sequence[self._index]
            self._index += 1
            return item
        else:
            raise StopIteration
    
    def __index__(self):
        return self.current_index
        

class SensorModel:
    """Sensor model for computing camera frustum visualizations."""

    def __init__(self, horz_fov, aspect_ratio, sensor_range):
        """Initialize sensor model with camera parameters."""
        cam = Camera(
            Position=np.array([0.0, 0.0, 0.0]),
            Front=np.array([1.0, 0.0, 0.0]),
            Right=np.array([0.0, -1.0, 0.0]),
            Up=np.array([0.0, 0.0, 1.0])
        )
        # Don't return from __init__, store frustum as instance variable
        self.frustum_vertices = self.create_frustum_from_camera(
            cam, 
            aspect=aspect_ratio, 
            fov_y=np.radians(horz_fov), 
            z_near=0.0, 
            z_far=sensor_range
        )
        
        # Camera parameters for frustum visualization
        self.rot_sensor_to_body = None  # Set by LoadSensorParams

    @staticmethod
    def create_frustum_from_camera(cam: Camera, aspect: float, fov_y: float, 
                                      z_near: float, z_far: float):
        """
        Create frustum vertices from camera parameters.
        
        Args:
            cam: Camera with Position, Front, Right, Up vectors
            aspect: Aspect ratio (width/height)
            fov_y: Vertical field of view in radians
            z_near: Near plane distance
            z_far: Far plane distance
            
        Returns:
            List of face vertex lists for frustum visualization
        """
        body_origin = np.array([0.0, 0.0, 0.0])
        
        half_v_side_far = z_far * np.tan(fov_y * 0.5)
        half_h_side_far = half_v_side_far * aspect
        
        front_mult_far = z_far * cam.Front
        far_center = cam.Position + front_mult_far
        
        far_top_left = far_center + cam.Up * half_v_side_far - cam.Right * half_h_side_far
        far_top_right = far_center + cam.Up * half_v_side_far + cam.Right * half_h_side_far
        far_bottom_left = far_center - cam.Up * half_v_side_far - cam.Right * half_h_side_far
        far_bottom_right = far_center - cam.Up * half_v_side_far + cam.Right * half_h_side_far

        right_face_vertex_list = [body_origin, far_bottom_left, far_top_left]
        top_face_vertex_list = [body_origin, far_top_left, far_top_right]
        left_face_vertex_list = [body_origin, far_top_right, far_bottom_right]
        bottom_face_vertex_list = [body_origin, far_bottom_right, far_bottom_left]

        return [right_face_vertex_list, left_face_vertex_list, 
                top_face_vertex_list, bottom_face_vertex_list]


class PlannerUtils(SensorModel):
    """Utility functions for planning calculations."""

    _node = None

    @classmethod
    def set_node(cls, node):
        """Set ROS node for parameter access."""
        cls._node = node

    @classmethod
    def _get_param(cls, name, default=None):
        """Get parameter value with fallback to default."""
        if cls._node is None:
            return default
        if isinstance(name, str):
            if name.startswith('/'):
                key = name[1:]
            elif name.startswith('~'):
                key = name[1:]
            else:
                key = name
        else:
            key = name
        try:
            if cls._node.has_parameter(key):
                return cls._node.get_parameter(key).value
            return cls._node.get_parameter(key).value
        except Exception:
            return default

    @classmethod
    def _now_msg(cls):
        """Get current time as ROS message."""
        if cls._node is None:
            return rclpy.clock.Clock().now().to_msg()
        return cls._node.get_clock().now().to_msg()
    
    class LoadSensorParams:
        """Load sensor parameters and create sensor model."""

        def __init__(self):
            ns_ = PlannerUtils._get_param("/robot_namespace", "")
            sensor_type = PlannerUtils._get_param("/sensor_modality", 0)
            fov = PlannerUtils._get_param("/fov", [69.4, 45])           # degrees
            ar = PlannerUtils._get_param("/aspect_ratio", 1.33)         # width / height
            sensor_range = PlannerUtils._get_param("/inspection_distance", 2.0)

            if sensor_type == 0:  # camera
                rot_params = PlannerUtils._get_param("/rotation", [0.0, 0.0, 0.0])
                rot_euler = np.array([rot_params[0], rot_params[1], rot_params[2]])

                self.rot_sensor_to_body = R.from_euler("XYZ", rot_euler, degrees=True)

                # Define unit vectors in the sensor frame (sensor-forward = +X)
                up = np.array([0.0, 0.0, 1.0])
                forward = np.array([1.0, 0.0, 0.0])
                right = np.array([0.0, 1.0, 0.0])

                # Frustum endpoints (sensor frame) and body frame
                self.frustum_endpoints_sensor = []
                self.frustum_endpoints_body = []

                center = np.array([0.0, 0.0, 0.0])

                # Horizontal FOV is fov[0] degrees => compute width at far plane
                wfar = 2.0 * np.tan(np.deg2rad(fov[0]) / 2.0) * sensor_range
                # Height from aspect ratio width/height
                hfar = wfar / ar

                # Far plane center (in sensor frame, forward axis)
                farCenter = center + forward * sensor_range

                # Compute corners on far plane (sensor coordinates)
                epTL = farCenter + right * (wfar / 2.0) + up * (hfar / 2.0)
                epTR = farCenter - right * (wfar / 2.0) + up * (hfar / 2.0)
                epBR = farCenter - right * (wfar / 2.0) - up * (hfar / 2.0)
                epBL = farCenter + right * (wfar / 2.0) - up * (hfar / 2.0)

                self.frustum_endpoints_sensor = np.array([epTL, epTR, epBR, epBL])

                # Transform sensor-frame corners into body frame
                R_s2b = self.rot_sensor_to_body.as_matrix()
                for vertex in self.frustum_endpoints_sensor:
                    self.frustum_endpoints_body.append(R_s2b @ vertex)

                # Ensure stored as numpy arrays
                self.frustum_endpoints_body = [np.asarray(v) for v in self.frustum_endpoints_body]

        def getFrustumEndpoints_W2B(self, world_state):
            """Transform frustum endpoints from world to body frame."""
            world_pos = np.asarray(world_state[0:3])
            r, p, yaw = PlannerUtils.quat2eul(
                world_state[3], world_state[4], world_state[5], world_state[6]
            )

            # Rotation from world -> body (constructed from yaw)
            rot_W2B = R.from_euler("XYZ", np.array([0.0, 0.0, yaw]), degrees=False)
            R_w2b = rot_W2B.as_matrix()

            frustum_endpoints_world = []
            for vertex_body in self.frustum_endpoints_body:
                # Transform body-frame vertex to world: world_pos + R_w2b @ vertex_body
                frustum_endpoints_world.append(world_pos + (R_w2b @ vertex_body))

            return frustum_endpoints_world

        def get_frustum(self, odom_state, rot, trans, counter):
            """Get frustum visualization marker."""
            # Get corners (list of 4 points: TL,TR,BR,BL) in world coords
            frustum_corners = self.getFrustumEndpoints_W2B(odom_state)

            # Camera origin (world coords)
            origin = odom_state[0:3]

            m = Marker()
            m.header.frame_id = PlannerUtils._get_param("/sensor_frame", "husky/base_link")
            m.ns = "camera_frustum"
            m.id = counter
            m.type = Marker.LINE_LIST
            m.action = Marker.ADD
            m.scale.x = 0.02  # Line width in meters
            m.color = ColorRGBA()

            if counter == 0:
                m.scale.x = 0.05
                m.color.r = 0.91
                m.color.g = 0.12
                m.color.b = 0.31
                m.color.a = 1.0
            else:
                m.color.r = 0.0
                m.color.g = 1.0
                m.color.b = 0.0
                m.color.a = 1.0
            
            # Ensure corners are numpy arrays
            corners = [np.asarray(c) for c in frustum_corners]  # TL, TR, BR, BL

            # Lines from origin to each corner
            for c in corners:
                m.points.append(PlannerUtils.getPointMsg(origin[0], origin[1], origin[2]))
                m.points.append(PlannerUtils.getPointMsg(float(c[0]), float(c[1]), float(c[2])))

            # Lines around the far plane (TL->TR, TR->BR, BR->BL, BL->TL)
            for i in range(4):
                a = corners[i]
                b = corners[(i + 1) % 4]
                m.points.append(PlannerUtils.getPointMsg(float(a[0]), float(a[1]), float(a[2])))
                m.points.append(PlannerUtils.getPointMsg(float(b[0]), float(b[1]), float(b[2])))

            return m
    
    @staticmethod
    def GetRotmat_B2W(world_state):
        """Get rotation matrix from body to world frame."""
        roll, pitch, yaw = PlannerUtils.quat2eul(
            world_state[3], world_state[4], world_state[5], world_state[6]
        )
        rot_B2W = R.from_euler("XYZ", [roll, pitch, yaw], degrees=False).inv()
        
        return rot_B2W

    @staticmethod
    def getPathMsgfromViewPlan(viewplan):
        """Convert view plan to Path message."""
        pathMsg = Path()
        
        for pose in viewplan:
            poseMsg = PoseStamped()
        
            poseMsg.pose.position.x = pose[0]
            poseMsg.pose.position.y = pose[1]
            poseMsg.pose.position.z = pose[2]

            poseMsg.pose.orientation.x = 0
            poseMsg.pose.orientation.y = 0
            poseMsg.pose.orientation.z = 0
            poseMsg.pose.orientation.w = 1
            
            pathMsg.poses.append(poseMsg)
        
        return pathMsg

    @staticmethod
    def pc2_to_xyz(raw_points) -> np.ndarray:
        """
        Convert PointCloud2 to numpy XYZ array.
        
        Args:
            raw_points: PointCloud2 message
            
        Returns:
            np.ndarray: Nx3 array of (x, y, z) points, filtered for finite values
        """
        t = np.array(
            list(point_cloud2.read_points(
                raw_points,
                field_names=("x", "y", "z"),
                skip_nans=False,
            ))
        )

        if t.size == 0:
            return np.empty((0, 3), dtype=np.float32)

        # If it's structured (has named fields), stack them
        if t.dtype.names is not None:
            pts = np.column_stack((t["x"], t["y"], t["z"])).astype(np.float32, copy=False)
        else:
            # Otherwise it's already Nx3-like
            pts = np.asarray(t, dtype=np.float32)
            if pts.ndim == 1:
                pts = pts.reshape(-1, 3)
            else:
                pts = pts[:, :3]

        # Drop NaN/Inf rows
        pts = pts[np.isfinite(pts).all(axis=1)]
        return pts

    @staticmethod
    def crop_points_within_fov(points, odom_pose, fov=60, increment_iter=1, increment_fov=False):
        """
        Crop points within field of view cone.
        
        Args:
            points: Nx3 numpy array of points
            odom_pose: [x, y, z, qx, qy, qz, qw] pose array
            fov: Field of view in degrees (default 60, should use configured value)
            increment_iter: Iteration counter for incremental FOV
            increment_fov: Whether to use incremental FOV mode
            
        Returns:
            tuple: (cropped_points_array, PointCloud2_message)
        """
        # Validate quaternion before transformation
        t = np.asarray(odom_pose[:3])
        q = np.asarray(odom_pose[3:7])  # (x, y, z, w) for SciPy
        
        # Normalize quaternion
        q_norm = np.linalg.norm(q)
        if q_norm < Constants.NORMALIZATION_EPSILON:
            logger.warning("Quaternion is zero vector, using identity")
            q = np.array([0.0, 0.0, 0.0, 1.0])
        else:
            q = q / q_norm

        # Check for NaN/Inf
        if not np.isfinite(q).all():
            logger.warning("Quaternion contains NaN/Inf, using identity")
            q = np.array([0.0, 0.0, 0.0, 1.0])

        # World-from-body rotation
        R_wb = R.from_quat(q)
        # Extract roll, pitch, yaw (intrinsic x,y,z). rz is yaw.
        rx, ry, rz = R_wb.as_euler('xyz', degrees=False)

        # Build yaw-only rotation (world-from-body with only yaw)
        R_wb_yaw = R.from_euler('z', rz)

        # Transform world points into yaw-only ego frame: p_yaw = R_yaw^T (p - t)
        p_rel = points - t
        p_yaw = R_wb_yaw.inv().apply(p_rel)

        # FOV about +X axis (leveled, roll/pitch removed)
        # Angle between vector and +X equals acos( dot( v̂, x̂ ) ) -> compare dot to cos(th)
        norms = np.linalg.norm(p_yaw, axis=1)
        valid = norms > Constants.NORMALIZATION_EPSILON
        vhat = np.zeros_like(p_yaw)
        vhat[valid] = p_yaw[valid] / norms[valid, None]

        # Clip for safety if you later use arccos
        if increment_fov:
            cos_thresh = np.cos(np.deg2rad(fov * increment_iter))
        else:
            cos_thresh = np.cos(np.deg2rad(fov))
        
        dots = np.clip(vhat[:, 0], -1.0, 1.0)

        fov_mask = dots >= cos_thresh
        cropped_points = points[fov_mask]

        # Create PointCloud2 message
        pcl = PointCloud2()
        pcl.header.frame_id = PlannerUtils._get_param("/world_frame", "world")
        pcl.header.stamp = PlannerUtils._now_msg()

        fields = [
            PointField(name='x', offset=0, datatype=PointField.FLOAT32, count=1),
            PointField(name='y', offset=4, datatype=PointField.FLOAT32, count=1),
            PointField(name='z', offset=8, datatype=PointField.FLOAT32, count=1),
        ]

        pointcloudMsg = point_cloud2.create_cloud(pcl.header, fields, cropped_points.tolist())
        return cropped_points, pointcloudMsg

    @staticmethod
    def PoseArraytoPathMsg(path, ref_alt=None):
        """Convert pose array to Path message."""
        pathMsg = Path()
        pathMsg.header.frame_id = PlannerUtils._get_param("/world_frame", "world")
        pathMsg.header.stamp = PlannerUtils._now_msg()
        
        for pose in path:
            pose_msg = PoseStamped()
            pose_msg.header.frame_id = PlannerUtils._get_param("/world_frame", "world")
            pose_msg.header.stamp = PlannerUtils._now_msg()
            
            pose_msg.pose.position.x = pose[0]
            pose_msg.pose.position.y = pose[1]
            if ref_alt is not None:
                pose_msg.pose.position.z = ref_alt
            else:
                pose_msg.pose.position.z = pose[2]
            
            pose_msg.pose.orientation.x = 0
            pose_msg.pose.orientation.y = 0
            pose_msg.pose.orientation.z = 0
            pose_msg.pose.orientation.w = 1
            
            pathMsg.poses.append(pose_msg)
        
        return pathMsg
    
    @staticmethod
    def PoseArraytoPoseMsg(pose):
        """Convert pose array to PoseStamped message."""
        pose_msg = PoseStamped()
        pose_msg.header.frame_id = PlannerUtils._get_param("/world_frame", "world")
        pose_msg.header.stamp = PlannerUtils._now_msg()
        
        pose_msg.pose.position.x = pose[0]
        pose_msg.pose.position.y = pose[1]
        pose_msg.pose.position.z = pose[2]
        
        pose_msg.pose.orientation.x = pose[3]
        pose_msg.pose.orientation.y = pose[4]
        pose_msg.pose.orientation.z = pose[5]
        pose_msg.pose.orientation.w = pose[6]
        
        return pose_msg

    @staticmethod
    def PoseMsgtoPoseArray(pose_msg):
        """
        Convert a Pose message to a numpy array [x, y, z, qx, qy, qz, qw].
        
        Args:
            pose_msg: Pose or PoseStamped message
            
        Returns:
            np.ndarray: 7-element array [x, y, z, qx, qy, qz, qw]
        """
        pose_array = np.zeros(7)
        
        # Handle both Pose and PoseStamped
        if hasattr(pose_msg, 'pose'):
            pose = pose_msg.pose
        else:
            pose = pose_msg
            
        pose_array[0] = pose.position.x
        pose_array[1] = pose.position.y
        pose_array[2] = pose.position.z
        pose_array[3] = pose.orientation.x
        pose_array[4] = pose.orientation.y
        pose_array[5] = pose.orientation.z
        pose_array[6] = pose.orientation.w

        return pose_array

    @staticmethod
    def GetLookAtOrientation(start, end):
        """
        Compute look-at orientation from start to end point.
        
        Args:
            start: Start position [x, y, z]
            end: End position [x, y, z]
            
        Returns:
            tuple: (qx, qy, qz, qw) quaternion orientation
        """
        la = end - start
        
        # Prevent division by zero
        norm_la = np.linalg.norm(la)
        if norm_la < Constants.NORMALIZATION_EPSILON:
            logger.warning("GetLookAtOrientation: start and end are coincident, using default orientation")
            return 0.0, 0.0, 0.0, 1.0  # Identity quaternion
            
        nm_la = la / norm_la
        cd_yaw = np.arctan2(nm_la[1], nm_la[0])
        qx, qy, qz, qw = PlannerUtils.eul2quat(0, 0, cd_yaw)

        return qx, qy, qz, qw

    @staticmethod
    def transform2Global(odom_pose, curr_yaw, pos, yaw):
        """Transform local position to global coordinates."""
        global_pos = np.array([0.0, 0.0, 0.0])

        global_pos[0] = odom_pose[0] + math.cos(curr_yaw) * pos[0] - math.sin(curr_yaw) * pos[1]
        global_pos[1] = odom_pose[1] + math.cos(curr_yaw) * pos[1] + math.sin(curr_yaw) * pos[0]
        global_pos[2] = odom_pose[2] + pos[2]

        global_yaw = curr_yaw + yaw

        return global_pos, global_yaw
    
    @staticmethod
    def eul2quat(roll, pitch, yaw):
        """
        Convert Euler angles to quaternion.
        
        Args:
            roll: Roll angle in radians
            pitch: Pitch angle in radians
            yaw: Yaw angle in radians
            
        Returns:
            tuple: (qx, qy, qz, qw) normalized quaternion
        """
        qx = np.sin(roll/2) * np.cos(pitch/2) * np.cos(yaw/2) - np.cos(roll/2) * np.sin(pitch/2) * np.sin(yaw/2)
        qy = np.cos(roll/2) * np.sin(pitch/2) * np.cos(yaw/2) + np.sin(roll/2) * np.cos(pitch/2) * np.sin(yaw/2)
        qz = np.cos(roll/2) * np.cos(pitch/2) * np.sin(yaw/2) - np.sin(roll/2) * np.sin(pitch/2) * np.cos(yaw/2)
        qw = np.cos(roll/2) * np.cos(pitch/2) * np.cos(yaw/2) + np.sin(roll/2) * np.sin(pitch/2) * np.sin(yaw/2)
        
        qnorm = np.linalg.norm([qx, qy, qz, qw])
        
        # Prevent division by zero
        if qnorm < Constants.NORMALIZATION_EPSILON:
            logger.warning("eul2quat: zero quaternion, returning identity")
            return 0.0, 0.0, 0.0, 1.0
        
        qx = qx / qnorm
        qy = qy / qnorm
        qz = qz / qnorm
        qw = qw / qnorm
        
        return qx, qy, qz, qw
    
    @staticmethod
    def quat2eul(x, y, z, w):
        """
        Convert quaternion to Euler angles.
        
        Args:
            x, y, z, w: Quaternion components
            
        Returns:
            tuple: (roll, pitch, yaw) in radians
        """
        # Validate quaternion
        q_norm = np.sqrt(x*x + y*y + z*z + w*w)
        if q_norm < Constants.NORMALIZATION_EPSILON:
            logger.warning("quat2eul: zero quaternion, returning zero angles")
            return 0.0, 0.0, 0.0
        
        # Normalize
        x = x / q_norm
        y = y / q_norm
        z = z / q_norm
        w = w / q_norm

        t0 = +2.0 * (w * x + y * z)
        t1 = +1.0 - 2.0 * (x * x + y * y)
        roll = math.atan2(t0, t1)
        
        t2 = +2.0 * (w * y - z * x)
        t2 = +1.0 if t2 > +1.0 else t2
        t2 = -1.0 if t2 < -1.0 else t2
        pitch = math.asin(t2)
        
        t3 = +2.0 * (w * z + x * y)
        t4 = +1.0 - 2.0 * (y * y + z * z)
        yaw = math.atan2(t3, t4)

        return roll, pitch, yaw

    @staticmethod
    def getPointMsg(x, y, z):
        """Create Point message from coordinates."""
        p = Point()
        p.x = float(x)
        p.y = float(y)
        p.z = float(z)
        return p
