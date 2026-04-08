
from inspection_planner.header import *


@dataclass
class Plane:
    point: np.ndarray
    normal: np.ndarray

@dataclass
class Frustum:
    nearFace: Plane
    farFace: Plane
    rightFace: Plane
    leftFace: Plane
    topFace: Plane
    bottomFace: Plane

@dataclass
class Camera:
    Position: np.ndarray
    Front: np.ndarray
    Right: np.ndarray
    Up: np.ndarray

class SequenceIterator:
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
        
            
class SensorModel():

    def __init__(self,horz_fov,aspect_ratio,sensor_range):

        cam = Camera(Position=np.array([0, 0, 0]), Front=np.array([1, 0, 0]), Right=np.array([0, -1, 0]), Up=np.array([0, 0, 1]))
        return self.create_frustum_from_camera(cam, aspect=aspect_ratio, fov_y=np.radians(horz_fov), z_near=0.0, z_far=sensor_range)

    def create_frustum_from_camera(cam: Camera, aspect: float, fov_y: float, z_near: float, z_far: float):

        body_origin = np.array([0,0,0])
        
        half_v_side_far = z_far * np.tan(fov_y * 0.5)
        half_h_side_far = half_v_side_far * aspect
        # half_v_side_near = z_near * np.tan(fov_y * 0.5)
        # half_h_side_near = half_v_side_near * aspect
        
        front_mult_far = z_far * cam.Front
        # front_mult_near = z_near * cam.Front
        
        # near_center = cam.Position + front_mult_near
        far_center = cam.Position + front_mult_far
        
        # near_top_left = near_center + cam.Up * half_v_side_near - cam.Right * half_h_side_near
        # near_top_right = near_center + cam.Up * half_v_side_near + cam.Right * half_h_side_near
        # near_bottom_left = near_center - cam.Up * half_v_side_near - cam.Right * half_h_side_near
        # near_bottom_right = near_center - cam.Up * half_v_side_near + cam.Right * half_h_side_near
        
        far_top_left = far_center + cam.Up * half_v_side_far - cam.Right * half_h_side_far
        far_top_right = far_center + cam.Up * half_v_side_far + cam.Right * half_h_side_far
        far_bottom_left = far_center - cam.Up * half_v_side_far - cam.Right * half_h_side_far
        far_bottom_right = far_center - cam.Up * half_v_side_far + cam.Right * half_h_side_far
        
        # near_face = Plane(near_center, cam.Front)
        # far_face = Plane(far_center, -cam.Front)
        # right_face = Plane(cam.Position, np.cross(far_top_right - cam.Position, far_bottom_right - cam.Position))
        # left_face = Plane(cam.Position, np.cross(far_bottom_left - cam.Position, far_top_left - cam.Position))
        # top_face = Plane(cam.Position, np.cross(far_top_left - cam.Position, far_top_right - cam.Position))
        # bottom_face = Plane(cam.Position, np.cross(far_bottom_right - cam.Position, far_bottom_left - cam.Position))

        right_face_vertex_list = [body_origin,far_bottom_left,far_top_left]
        top_face_vertex_list = [body_origin,far_top_left,far_top_right]
        left_face_vertex_list = [body_origin,far_top_right,far_bottom_right]
        bottom_face_vertex_list = [body_origin,far_bottom_right,far_bottom_left]

        return [right_face_vertex_list,left_face_vertex_list,top_face_vertex_list,bottom_face_vertex_list]

class PlannerUtils(SensorModel):

    _node = None

    @classmethod
    def set_node(cls, node):
        cls._node = node

    @classmethod
    def _get_param(cls, name, default=None):
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
        if cls._node is None:
            return rclpy.clock.Clock().now().to_msg()
        return cls._node.get_clock().now().to_msg()
    class LoadSensorParams:

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

                # define unit vectors in the sensor frame (sensor-forward = +X)
                up = np.array([0.0, 0.0, 1.0])
                forward = np.array([1.0, 0.0, 0.0])
                right = np.array([0.0, 1.0, 0.0])

            # frustum endpoints (sensor frame) and body frame
            self.frustum_endpoints_sensor = []
            self.frustum_endpoints_body = []

            center = np.array([0.0, 0.0, 0.0])

            # horizontal FOV is fov[0] degrees => compute width at far plane
            wfar = 2.0 * np.tan(np.deg2rad(fov[0]) / 2.0) * sensor_range
            # height from aspect ratio width/height
            hfar = wfar / ar

            # far plane center (in sensor frame, forward axis)
            farCenter = center + forward * sensor_range

            # compute corners on far plane (sensor coordinates)
            epTL = farCenter + right * (wfar / 2.0) + up * (hfar / 2.0)
            epTR = farCenter - right * (wfar / 2.0) + up * (hfar / 2.0)
            epBR = farCenter - right * (wfar / 2.0) - up * (hfar / 2.0)
            epBL = farCenter + right * (wfar / 2.0) - up * (hfar / 2.0)

            self.frustum_endpoints_sensor = np.array([epTL, epTR, epBR, epBL])

            # transform sensor-frame corners into body frame
            R_s2b = self.rot_sensor_to_body.as_matrix()
            for vertex in self.frustum_endpoints_sensor:
                self.frustum_endpoints_body.append(R_s2b @ vertex)

            # make sure stored as numpy arrays
            self.frustum_endpoints_body = [np.asarray(v) for v in self.frustum_endpoints_body]

        def getFrustumEndpoints_W2B(self, world_state):

            world_pos = np.asarray(world_state[0:3])
            r,p,yaw = PlannerUtils.quat2eul(world_state[3],world_state[4],world_state[5],world_state[6])

            # rotation from world -> body (constructed from yaw)
            rot_W2B = R.from_euler("XYZ", np.array([0.0, 0.0, yaw]), degrees=False)
            R_w2b = rot_W2B.as_matrix()

            frustum_endpoints_world = []
            for vertex_body in self.frustum_endpoints_body:
                # transform body-frame vertex to world: world_pos + R_w2b @ vertex_body
                frustum_endpoints_world.append(world_pos + (R_w2b @ vertex_body))

            return frustum_endpoints_world

        def get_frustum(self, odom_state, rot, trans,counter):

            # get corners (list of 4 points: TL,TR,BR,BL) in world coords
            frustum_corners = self.getFrustumEndpoints_W2B(odom_state)

            # camera origin (world coords). 
            origin = odom_state[0:3]

            m = Marker()
            m.header.frame_id = PlannerUtils._get_param("/sensor_frame", "husky/base_link")
            m.ns = "camera_frustum"
            m.id = counter
            m.type = Marker.LINE_LIST
            m.action = Marker.ADD
            m.scale.x = 0.02  # line width in meters
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
            

            # ensure corners are numpy arrays
            corners = [np.asarray(c) for c in frustum_corners]  # TL, TR, BR, BL

            # lines from origin to each corner
            for c in corners:
                m.points.append(PlannerUtils.getPointMsg(origin[0], origin[1], origin[2]))
                m.points.append(PlannerUtils.getPointMsg(float(c[0]), float(c[1]), float(c[2])))

            # lines around the far plane (TL->TR, TR->BR, BR->BL, BL->TL)
            for i in range(4):
                a = corners[i]
                b = corners[(i + 1) % 4]
                m.points.append(PlannerUtils.getPointMsg(float(a[0]), float(a[1]), float(a[2])))
                m.points.append(PlannerUtils.getPointMsg(float(b[0]), float(b[1]), float(b[2])))

            return m
    
    def GetRotmat_B2W(world_state):

        roll, pitch, yaw = PlannerUtils.quat2eul(world_state[3], world_state[4], world_state[5], world_state[6])
        rot_B2W = R.from_euler("XYZ", [roll, pitch, yaw], degrees=False).inv()
        
        return rot_B2W

    
    def getPathMsgfromViewPlan(viewplan):
        
        pathMsg  = Path()
        
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

    
    def pc2_to_xyz(raw_points) -> np.ndarray:
        # Force a structured numpy array with fields x,y,z
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

    def crop_points_within_fov(points,odom_pose,fov=60,increment_iter = 1,increment_fov=False):
        
        # odom_pose = [x, y, z, qx, qy, qz, qw]
        t = np.asarray(odom_pose[:3])
        q = np.asarray(odom_pose[3:7])  # (x, y, z, w) for SciPy

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
        valid = norms > 1e-8
        vhat = np.zeros_like(p_yaw)
        vhat[valid] = p_yaw[valid] / norms[valid, None]

        # dot with x-axis == vhat[:,0]
        if increment_fov:
            cos_thresh = np.cos(np.deg2rad(fov*increment_iter))
        else:
            cos_thresh = np.cos(np.deg2rad(fov))
        # clip for safety if you later use arccos (here we don't need arccos)
        dots = np.clip(vhat[:, 0], -1.0, 1.0)

        fov_mask = dots >= cos_thresh
        cropped_points = points[fov_mask]

        # (Optional) publish as PointCloud2
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

    def PoseArraytoPathMsg(path,ref_alt=None):
        
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
    
    
    def PoseArraytoPoseMsg(pose):

        
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

    def PoseMsgtoPoseArray(pose_msg):
        """
        Convert a Pose message to a numpy array [x, y, z, qx, qy, qz, qw].
        """
        pose_array = np.zeros(7)
        pose_array[0] = pose_msg.pose.position.x
        pose_array[1] = pose_msg.pose.position.y
        pose_array[2] = pose_msg.pose.position.z
        pose_array[3] = pose_msg.pose.orientation.x
        pose_array[4] = pose_msg.pose.orientation.y
        pose_array[5] = pose_msg.pose.orientation.z
        pose_array[6] = pose_msg.pose.orientation.w

        return pose_array

    def GetLookAtOrientation(start,end):

        la = end - start

        nm_la = la/np.linalg.norm(la)
        cd_yaw = np.arctan2(nm_la[1],nm_la[0])
        qx,qy,qz,qw = PlannerUtils.eul2quat(0,0,cd_yaw)

        return qx,qy,qz,qw

    def transform2Global(odom_pose,curr_yaw,pos,yaw):
        
        global_pos = np.array([0,0,0])

        global_pos[0] = odom_pose[0] + math.cos(curr_yaw)*pos[0] - math.sin(curr_yaw)*pos[1]
        global_pos[1] = odom_pose[1] + math.cos(curr_yaw)*pos[1] + math.sin(curr_yaw)*pos[0]
        global_pos[2] = odom_pose[2] + pos[2]

        global_yaw = curr_yaw + yaw

        return global_pos, global_yaw
    
    def eul2quat(roll,pitch,yaw):
        qx = np.sin(roll/2) * np.cos(pitch/2) * np.cos(yaw/2) - np.cos(roll/2) * np.sin(pitch/2) * np.sin(yaw/2)
        qy = np.cos(roll/2) * np.sin(pitch/2) * np.cos(yaw/2) + np.sin(roll/2) * np.cos(pitch/2) * np.sin(yaw/2)
        qz = np.cos(roll/2) * np.cos(pitch/2) * np.sin(yaw/2) - np.sin(roll/2) * np.sin(pitch/2) * np.cos(yaw/2)
        qw = np.cos(roll/2) * np.cos(pitch/2) * np.cos(yaw/2) + np.sin(roll/2) * np.sin(pitch/2) * np.sin(yaw/2)
        
        qnorm = np.linalg.norm([qx,qy,qz,qw])
        qx = qx/qnorm
        qy = qy/qnorm
        qz = qz/qnorm
        qw = qw/qnorm
        
        return qx, qy, qz, qw
    
    def quat2eul(x,y,z,w):

        t0 = +2.0 * (w * x + y * z)
        t1 = +1.0 - 2.0 * (x * x + y * y)
        roll = math.atan2(t0, t1)
        
        t2 = +2.0 * (w * y - z * x)
        t2 = +1.0 if t2 > +1.0 else t2
        t2 = -1.0 if t2 < -1.0 else t2
        pitch = math.asin(t2)
        
        t3 = +2.0 * (w * z + x * y)
        t4 = +1.0 - 2.0 * (y * y + z * z)
        yaw = (math.atan2(t3, t4))

        return roll,pitch,yaw

    def getPointMsg(x, y, z):
        p = Point()
        p.x, p.y, p.z = float(x), float(y), float(z)
        return p
