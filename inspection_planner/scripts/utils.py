#! usr/bin/env python3 

from header import *
from dataclasses import dataclass
import heapq
import ttictoc
from itertools import count
from scipy.spatial.transform import Rotation as R
from scipy.spatial.transform import Slerp
from scipy.spatial import geometric_slerp

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
        
            
class GradientColorGenerator:
    def __init__(self, saturation=1.0, value=1.0, step=0.1):
        """
        :param saturation: Saturation level (0-1). 1 means full color.
        :param value: Value/Brightness level (0-1). 1 means maximum brightness.
        :param step: The amount by which the hue will increment on each call.
        """
        self.hue = 0.0  # starting hue (0 corresponds to red)
        self.saturation = saturation
        self.value = value
        self.step = step
        self.cmap = plt.get_cmap("plasma")

    def get_next_color(self,value,max_val,min_val):
        """
        Returns:
            A tuple (r, g, b) where each component is in the range [0, 1].
        """

        # Convert the current HSV value to RGB (all in 0-1 range)
        # rgb = mcolors.hsv_to_rgb((self.hue, self.saturation, self.value))
        # Increment hue for the next call
        self.hue = (self.hue + self.step) % 1.0  # wrap around after reaching 1.0
        normalized_value = (value - min_val) / (max_val - min_val)
        rgba = self.cmap(normalized_value)

        rgb = rgba[:3]

        return rgb

class DTWGradientColorGenerator:
    def __init__(self, saturation=1.0, value=1.0, step=0.1,ucmap="plasma" ):
        """
        :param saturation: Saturation level (0-1). 1 means full color.
        :param value: Value/Brightness level (0-1). 1 means maximum brightness.
        :param step: The amount by which the hue will increment on each call.
        """
        self.hue = 0.0  # starting hue (0 corresponds to red)
        self.saturation = saturation
        self.value = value
        self.step = step
        self.cmap = plt.get_cmap(ucmap)

    def get_next_color(self,value,max_val,min_val):
        """
        Returns:
            A tuple (r, g, b) where each component is in the range [0, 1].
        """

        # Convert the current HSV value to RGB (all in 0-1 range)
        # rgb = mcolors.hsv_to_rgb((self.hue, self.saturation, self.value))
        # Increment hue for the next call
        self.hue = (self.hue + self.step) % 1.0  # wrap around after reaching 1.0
        normalized_value = (value - min_val) / (max_val - min_val)
        rgba = self.cmap(normalized_value)

        rgb = rgba[:3]

        return rgb
    
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


class PlannerUtils(GradientColorGenerator,DTWGradientColorGenerator):
    
    class LoadSensorParams:
        
        def __init__(self):
            
            ns_ = rospy.get_param("/robot_namespace")
            sensor_type = rospy.get_param("/sensor_modality")
            fov = rospy.get_param("/fov")
            ar = rospy.get_param("/aspect_ratio")
            sensor_range = rospy.get_param("/sensor_range")
            
            if sensor_type == 0 : # For camera
                
                rot_params = rospy.get_param("/rotation")
                rot_euler = np.array([rot_params[0],rot_params[1],rot_params[2]])
                rot_B2S = R.from_euler("XYZ",rot_euler,degrees=False)
            
                ## define unit vectors<
                
                up = np.array([0,0,1])
                forward = np.array([1,0,0])
                right = np.array([0,1,0])
                
            ## create3DViewFrustumGeometry
            
            self.frustum_endpoints_sensor = []
            self.frustum_endpoints_body = []
            
            center = np.array([0.0,0.0,0.0])
            wfar = 2 * np.tan(np.deg2rad(fov[0])/2) * sensor_range[1]
    
            hfar = wfar/ar
        
            farCenter = center + forward*sensor_range[1]  
    
            epTL = farCenter + (np.dot(right,(wfar/2))) + (np.dot(up,(hfar/2)))
            epTR = farCenter - (np.dot(right,(wfar/2))) + (np.dot(up,(hfar/2)))
            epBR = farCenter - (np.dot(right,(wfar/2))) - (np.dot(up,(hfar/2)))
            epBL = farCenter + (np.dot(right,(wfar/2))) - (np.dot(up,(hfar/2)))
            
            self.frustum_endpoints_sensor = np.array([epTL,epTR,epBR,epBL])
            
            # rospy.loginfo("sensor EP: {}".format(self.frustum_endpoints_sensor))
            
            for vertex in self.frustum_endpoints_sensor:
                
                self.frustum_endpoints_body.append(rot_B2S.as_matrix()@vertex)
                
            # rospy.loginfo("body EP: {}".format(self.frustum_endpoints_body))
            
        def getFrustomEndpoints_W2B(self,world_state):
            
            world_pos = world_state[0:3]
            
            rot_W2B = R.from_euler("XYZ",np.array([0.0,0.0,world_state[3]]),degrees=False)
            
            frustum_endpoints_world = []
            
            for vertex in self.frustum_endpoints_body:
                
                frustum_endpoints_world.append(world_pos + rot_W2B.as_matrix()*vertex)
            
            return frustum_endpoints_world
        
    def GetRotmat_B2W(world_state):

        roll, pitch, yaw = PlannerUtils.quat2eul(world_state[3], world_state[4], world_state[5], world_state[6])
        rot_B2W = R.from_euler("XYZ", [roll, pitch, yaw], degrees=False).inv()
        
        return rot_B2W

    def InspectionROItoPoseStampedList(inspection_plan):
        """
        Convert an InspectionPlan to a list of PoseStamped messages.
        Each ROI centroid is converted to a PoseStamped message.
        """
        pose_list = []
            
        for j, validPlan in enumerate(inspection_plan.status):
            # print(">>>", validPlan in ('ACTIVE', 'Initialized'))
            # if validPlan in ('ACTIVE', 'Initialized'):
                    
            centroid = inspection_plan.roi_centroids[j]

            # print("centroid",centroid)
            pose_msg = PoseStamped()
            pose_msg.header.frame_id = rospy.get_param("/world_frame")
            pose_msg.header.stamp = rospy.Time.now()
            
            pose_msg.pose.position.x = centroid[0]
            pose_msg.pose.position.y = centroid[1]
            pose_msg.pose.position.z = centroid[2]
            
            # Default orientation (no rotation)
            pose_msg.pose.orientation.x = centroid[3]
            pose_msg.pose.orientation.y = centroid[4]
            pose_msg.pose.orientation.z = centroid[5]
            pose_msg.pose.orientation.w = centroid[6]
            
            pose_list.append(pose_msg)
                
        # print("lenof ROI RRT query",len(pose_list))
        
        if len(pose_list) < 1:
            logger.warning("Invalid range of pose list. Most likely its empty")
        
        return pose_list
    
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

    def euclidean_distance_3d(p1, p2):
        return np.linalg.norm(np.array(p1) - np.array(p2))

    def compute_frechet_distance(P, Q):
        len_p = len(P)
        len_q = len(Q)
        ca = np.full((len_p, len_q), -1.0)

        def c(i, j):
            if ca[i, j] > -1:
                return ca[i, j]
            elif i == 0 and j == 0:
                ca[i, j] = PlannerUtils.euclidean_distance_3d(P[0], Q[0])
            elif i > 0 and j == 0:
                ca[i, j] = max(c(i - 1, 0), PlannerUtils.euclidean_distance_3d(P[i], Q[0]))
            elif i == 0 and j > 0:
                ca[i, j] = max(c(0, j - 1), PlannerUtils.euclidean_distance_3d(P[0], Q[j]))
            elif i > 0 and j > 0:
                ca[i, j] = max(
                    min(c(i - 1, j), c(i - 1, j - 1), c(i, j - 1)),
                    PlannerUtils.euclidean_distance_3d(P[i], Q[j])
                )
            else:
                ca[i, j] = float('inf')
            return ca[i, j]

        return c(len_p - 1, len_q - 1)

    def computeDTW(path1,path2):
        
        # path1 = global view plan
        # path2 = predicted path   
        
        tmp_path1 = np.asarray(path1)
        
        tmp_path2 = np.asarray(path2)
        
        
        score = dtw_ndim.warping_paths(tmp_path1,tmp_path2)
        
        return score
    
    def computeViewPlanBuffer(viewpoints_iterator,bufferQ,prediction_horizon=5):
        
        logger.info("Slicing and Dicing the view plan")

        to_add = islice(viewpoints_iterator, prediction_horizon)
        
        bufferQ.extend(to_add)

        if len(list(bufferQ)) < prediction_horizon:
            
            logger.warning("Assigned BufferQ is short of prediction horizon. Appending the last pose to fill buffer")
            logger.debug(f"BufferQ: {list(bufferQ)}")
            
            for kiter in range(prediction_horizon-len(list(bufferQ))):
                
                bufferQ.append(list(bufferQ)[-1])
        
        tee(viewpoints_iterator,prediction_horizon)
        
        return bufferQ

    def crop_points_within_fov(points,odom_pose,increment_fov=False):
        
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
            cos_thresh = np.cos(np.deg2rad(30))
        else:
            cos_thresh = np.cos(np.deg2rad(60))
        # clip for safety if you later use arccos (here we don't need arccos)
        dots = np.clip(vhat[:, 0], -1.0, 1.0)

        fov_mask = dots >= cos_thresh
        cropped_points = points[fov_mask]

        # (Optional) publish as PointCloud2
        pcl = PointCloud2()
        pcl.header.frame_id = rospy.get_param("/world_frame")
        pcl.header.stamp = rospy.Time.now()

        fields = [
            PointField('x', 0, PointField.FLOAT32, 1),
            PointField('y', 4, PointField.FLOAT32, 1),
            PointField('z', 8, PointField.FLOAT32, 1),
        ]
        pointcloudMsg = pcl2_msg.create_cloud(pcl.header, fields, cropped_points.tolist())
        return cropped_points, pointcloudMsg

    def PoseArraytoPathMsg(path,ref_alt=None):
        """
        Convert a PoseArray to a Pose message.
        Assumes the first pose in the array is the desired one.
        """
        # if not len(pose)== :
        #     logger.warning("PoseArray is empty.")
        #     return None
        
        pathMsg = Path()
        pathMsg.header.frame_id = rospy.get_param("/world_frame")
        pathMsg.header.stamp = rospy.Time.now()
        
        for pose in path:
        
            pose_msg = PoseStamped()
            pose_msg.header.frame_id = rospy.get_param("/world_frame")
            pose_msg.header.stamp = rospy.Time.now()
            
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
        """
        Convert a PoseArray to a Pose message.
        Assumes the first pose in the array is the desired one.
        """
        # if not len(pose)== :
        #     logger.warning("PoseArray is empty.")
        #     return None
        
        
        pose_msg = PoseStamped()
        pose_msg.header.frame_id = rospy.get_param("/world_frame")
        pose_msg.header.stamp = rospy.Time.now()
        
        pose_msg.pose.position.x = pose[0]
        pose_msg.pose.position.y = pose[1]
        pose_msg.pose.position.z = pose[2]
        
        pose_msg.pose.orientation.x = pose[3]
        pose_msg.pose.orientation.y = pose[4]
        pose_msg.pose.orientation.z = pose[5]
        pose_msg.pose.orientation.w = pose[6]
        
        return pose_msg

    def dtw_se3(A, B, w_pos=1.0, w_rot=1.0, pos_norm="l2", band=None,return_locals=False):
        return  PlannerUtils.dtw_path(A, B, lambda a,b: PlannerUtils.se3_step_cost(a,b,w_pos,w_rot,pos_norm), band=band,return_locals=return_locals)

    def _quat_normalize(q):
        n = np.linalg.norm(q)
        return q / n if n > 0 else np.array([0, 0, 0, 1], dtype=float)

    def _quat_geodesic_angle(q1, q2):
        """
        Geodesic angle between unit quaternions q1, q2 (x,y,z,w).
        Handles the double cover via abs(dot).
        Result in [0, pi].
        """
        q1 = PlannerUtils._quat_normalize(q1)
        q2 = PlannerUtils._quat_normalize(q2)
        d = float(np.abs(np.dot(q1, q2)))
        d = np.clip(d, -1.0, 1.0)
        return 2.0 * np.arccos(d)

    def se3_step_cost(a, b, w_pos=1.0, w_rot=1.0, pos_norm="l2"):
        """
        a,b: shape (7,) -> [x,y,z,qx,qy,qz,qw]
        """
        dp = a[:3] - b[:3]
        pos_err = np.linalg.norm(dp) if pos_norm == "l2" else np.abs(dp).sum()
        ang =  PlannerUtils._quat_geodesic_angle(a[3:], b[3:])
        return w_pos * pos_err + w_rot * ang
    
    def dtw_path(A, B, step_cost_fn, band=None,return_locals=False):
        """
        A: (N,D), B: (M,D)
        step_cost_fn: function(A[i], B[j]) -> scalar
        band: None or int (Sakoe–Chiba band; constrain |i-j| <= band)
        Returns: total_cost, path (list of (i_idx, j_idx))
        """
        A = np.asarray(A)
        B = np.asarray(B)
        N, M = len(A), len(B)

        INF = 1e18
        D = np.full((N+1, M+1), INF)
        D[0, 0] = 0.0
        back = np.full((N+1, M+1, 2), -1, dtype=int)

        for i in range(1, N+1):
            j_lo, j_hi = 1, M
            if band is not None:
                j_lo = max(1, i - band)
                j_hi = min(M, i + band)
            for j in range(j_lo, j_hi+1):
                c = step_cost_fn(A[i-1], B[j-1])
                choices = (D[i-1, j], D[i, j-1], D[i-1, j-1])  # up, left, diag
                k = int(np.argmin(choices))
                D[i, j] = c + choices[k]
                back[i, j] = (i-1, j) if k == 0 else ((i, j-1) if k == 1 else (i-1, j-1))

        # backtrack
        i, j = N, M
        path = []
        locals_list = []
        while i > 0 or j > 0:
            path.append((i-1, j-1))
            if return_locals and i > 0 and j > 0:
                locals_list.append(step_cost_fn(A[i-1], B[j-1]))
            pi, pj = back[i, j]
            if pi < 0 or pj < 0:
                break
            i, j = pi, pj
        path.reverse()
        if return_locals:
            locals_list.reverse()
            return float(D[N, M]), path, locals_list
        return float(D[N, M]), path

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
    def interpolatefromCurrentToReference(current_pose,reference_pose,max_vel=0.5,default_vel=0.1,plat_mode=1):

        # define the interpolation factor
         
        # perform linear interpolation between current and reference positions
        
        # performs slerp betwenn the orientation 
        
        start_pos = current_pose[0:3]
        if not plat_mode == 1:
            end_pos_Z = start_pos[2]
        end_pos = reference_pose[0:3]
        interim_pose = []
        
        if not np.linalg.norm(start_pos- end_pos) <= max_vel:
            
            inter_pos = []

            look_at = end_pos-start_pos

            dX = look_at/np.linalg.norm(look_at)

            start_rot = R.from_quat(current_pose[3:7])
            end_rot =  R.from_quat(reference_pose[3:7])
            
            rotations = R.concatenate([start_rot,end_rot])
            key_times = [0, 1]
            slerp = Slerp(key_times,rotations)

            interp_times = np.linspace(0, 1, num=np.int16(np.linalg.norm(start_pos-end_pos)/max_vel))

            interp_rots = slerp(interp_times)

            inter_rots = interp_rots.as_quat().tolist()

            for delT in range(0,np.int16(np.linalg.norm(start_pos-end_pos)/max_vel)):
                
                inter_pos.append(start_pos + dX*(delT*max_vel))
                
            inter_pos.pop(0)

            for pos,rot in zip(inter_pos,inter_rots):

                if not plat_mode == 1:
                    interim_pose.append(np.array([pos[0],pos[1],end_pos_Z,rot[0],rot[1],rot[2],rot[3]]))
                else:
                    interim_pose.append(np.array([pos[0],pos[1],pos[2],rot[0],rot[1],rot[2],rot[3]]))

            interim_pose.append(reference_pose)
            
            if not len(interim_pose) >= 1:
                
                logger.warning(f"Interpolated poses is non existent ! Defaulting to {max_vel-default_vel} ToDO")
                
                interim_pose = []
            
            interim_pose.append(reference_pose)
                # print("1",interim_pose)
            
            return interim_pose
        
        else:
            
            interim_pose = []
            
            interim_pose.append(reference_pose)
            
            # print("2",interim_pose)
            
            return interim_pose
            
            
    def computeUmeyamaAlignment(x: np.ndarray, y: np.ndarray,
                        with_scale: bool = False):
        """
        Computes the least squares solution parameters of an Sim(m) matrix
        that minimizes the distance between a set of registered points.
        Umeyama, Shinji: Least-squares estimation of transformation parameters
                        between two point patterns. IEEE PAMI, 1991
        :param x: mxn matrix of points, m = dimension, n = nr. of data points
        :param y: mxn matrix of points, m = dimension, n = nr. of data points
        :param with_scale: set to True to align also the scale (default: 1.0 scale)
        :return: r, t, c - rotation matrix, translation vector and scale factor
        """
        if x.shape != y.shape:
            raise logger.warning("data matrices must have the same shape")
            

        # m = dimension, n = nr. of data points
        m, n = x.shape

        # means, eq. 34 and 35
        mean_x = x.mean(axis=1)
        mean_y = y.mean(axis=1)

        # variance, eq. 36
        # "transpose" for column subtraction
        sigma_x = 1.0 / n * (np.linalg.norm(x - mean_x[:, np.newaxis])**2)

        # covariance matrix, eq. 38
        outer_sum = np.zeros((m, m))
        for i in range(n):
            outer_sum += np.outer((y[:, i] - mean_y), (x[:, i] - mean_x))
        cov_xy = np.multiply(1.0 / n, outer_sum)
        
        logger.debug (f"Covariance matrix: {cov_xy}")

        # SVD (text betw. eq. 38 and 39)
        u, d, v = np.linalg.svd(cov_xy)
        if np.count_nonzero(d > np.finfo(d.dtype).eps) < m - 1:
            raise logger.warning("Degenerate covariance rank, "
                                    "Umeyama alignment is not possible")

        # S matrix, eq. 43
        s = np.eye(m)
        if np.linalg.det(u) * np.linalg.det(v) < 0.0:
            # Ensure a RHS coordinate system (Kabsch algorithm).
            s[m - 1, m - 1] = -1

        # rotation, eq. 40
        r = u.dot(s).dot(v)

        # scale & translation, eq. 42 and 41
        c = 1 / sigma_x * np.trace(np.diag(d).dot(s)) if with_scale else 1.0
        t = mean_y - np.multiply(c, r.dot(mean_x))

        return  PlannerUtils.applyUmeyamaAlignment(r,t,c,y)
    
    def se3_inverse(p: np.ndarray) -> np.ndarray:
        """
        :param p: absolute SE(3) pose
        :return: the inverted pose
        """
        r_inv = p[:3, :3].transpose()
        t_inv = -r_inv.dot(p[:3, 3])
        return PlannerUtils.se3(r_inv, t_inv)

    def relative_se3(p1: np.ndarray, p2: np.ndarray) -> np.ndarray:
        """
        :param p1, p2: SE(3) matrices
        :return: the relative transformation p1^{⁻1} * p2
        """
        return np.dot(PlannerUtils.se3_inverse(p1), p2)

    def transform(poses,t: np.ndarray, right_mul: bool = False,
                  propagate: bool = False):
        """
        apply a left or right multiplicative transformation to the whole path
        :param t: a 4x4 transformation matrix (e.g. SE(3) or Sim(3))
        :param right_mul: whether to apply it right-multiplicative or not
        :param propagate: whether to propagate drift with RHS transformations
        """
        
        num_poses = len(poses)
        # print(poses)
        if right_mul and not propagate:
            # Transform each pose individually.
            poses_se3 = [np.dot(p, t) for p in poses]
        elif right_mul and propagate:
            # Transform each pose and propagate resulting drift to the next.
            ids = np.arange(0, num_poses, 1, dtype=int)
            rel_poses = [
                PlannerUtils.relative_se3(poses[i], poses[j]).dot(t)
                for i, j in zip(ids, ids[1:])
            ]
            poses_se3 = [poses[0]]
            for i, j in zip(ids[:-1], ids):
                poses_se3.append(poses_se3[j].dot(rel_poses[i]))
        else:
            poses_se3 = [np.dot(t, p) for p in poses]
        
        return poses_se3

    def se3(r: np.ndarray = np.eye(3),
            t: np.ndarray = np.array([0, 0, 0])) -> np.ndarray:
        """
        :param r: SO(3) rotation matrix
        :param t: 3x1 translation vector
        :return: SE(3) transformation matrix
        """
        se3 = np.eye(4)
        se3[:3, :3] = r
        se3[:3, 3] = t
        return se3

    def applyUmeyamaAlignment(rot_mat,trans_vec,c,reference_path):
        
        TMat = PlannerUtils.se3(rot_mat,trans_vec)
        
        new_poses = np.array([[pos[0], pos[1], pos[2], 1] for pos in reference_path])  # shape (4, N)
        
        aligned_path = PlannerUtils.transform(new_poses,TMat)
        
        print(">>>>",aligned_path)

        return aligned_path
        
    def estimate_transform(reference, predicted, *, with_scale=False, allow_reflection=False, weights=None, eps=1e-9):
        """
        Compute the transformation that maps 'reference' -> 'predicted'.
        reference, predicted: (N, d) arrays with d=2 or 3 and point-to-point correspondences.
        with_scale: if True, also estimate a uniform scale (Umeyama); else rigid Kabsch.
        allow_reflection: if False, enforce a proper rotation (det(R)=+1).
        weights: optional (N,) nonnegative weights for correspondences.
        Returns:
            T (d+1, d+1) homogeneous matrix,
            R (d, d) rotation,
            t (d,) translation,
            s (float) scale (1.0 if with_scale=False)
        """
        ref = np.asarray(reference, dtype=np.float64)
        pred = np.asarray(predicted, dtype=np.float64)
        if ref.shape != pred.shape or ref.ndim != 2:
            raise ValueError("reference and predicted must both be (N, d) arrays of the same shape")
        N, d = ref.shape

        if weights is not None:
            w = np.asarray(weights, dtype=np.float64).reshape(-1, 1)
            if w.shape[0] != N:
                raise ValueError("weights must have length N")
            if np.any(w < 0):
                raise ValueError("weights must be nonnegative")
            w = w / (w.sum() + eps)
            mu_ref = (w * ref).sum(axis=0)
            mu_pred = (w * pred).sum(axis=0)
            X = ref - mu_ref
            Y = pred - mu_pred
            H = (w * X).T @ Y                              # weighted covariance-like matrix
            var_ref = (w * (X**2)).sum()                   # for scale
        else:
            mu_ref = ref.mean(axis=0)
            mu_pred = pred.mean(axis=0)
            X = ref - mu_ref
            Y = pred - mu_pred
            H = X.T @ Y
            var_ref = (X**2).sum()

        # SVD-based rotation (Kabsch)
        U, S, Vt = np.linalg.svd(H)
        R = Vt.T @ U.T

        # Enforce a proper rotation unless reflections are allowed
        if np.linalg.det(R) < 0:
            if not allow_reflection:
                Vt[-1, :] *= -1
                R = Vt.T @ U.T
            # else: keep reflection

        # Optional uniform scale (Umeyama)
        if with_scale:
            # This matches Umeyama: s = trace(S) / sum(||X||^2)
            s = S.sum() / max(var_ref, eps)
        else:
            s = 1.0

        t = mu_pred - s * (R @ mu_ref)

        # Build homogeneous transform that maps reference -> predicted
        T = np.eye(d + 1)
        T[:d, :d] = s * R
        T[:d, d]  = t
        return T, R, t, s

    def apply_transform(points, T):
        """
        Apply a homogeneous transform T to an (N, d) point array (row vectors).
        """
        pts = np.asarray(points, dtype=np.float64)
        N, d = pts.shape
        pts_h = np.hstack([pts, np.ones((N, 1))])         # (N, d+1)
        aligned = (pts_h @ T.T)[:, :d]                    # row-vector convention
        return aligned

    def invert_transform(T):
        """
        Invert a homogeneous transform (works with or without uniform scale).
        """
        d = T.shape[0] - 1
        A = T[:d, :d]
        t = T[:d, d]
        A_inv = np.linalg.inv(A)
        T_inv = np.eye(d + 1)
        T_inv[:d, :d] = A_inv
        T_inv[:d, d] = -A_inv @ t
        return T_inv
    def GetPathfromTSPTour(plan,tour):
        
        # Get the fixed frame id from server
        fixed_frame = rospy.get_param("/world_frame")
    
        # Initialize the path container
        TSPPath = Path()
        TSPPath.header.frame_id = fixed_frame
        
        if not len(tour) > 1:
            logger.warning("TSP Tour is empty or has only one point.")
            
            # return None
        
        for index in tour:
            
            # Initialize pose container for the tour
            TSPPose = PoseStamped()
            TSPPose.header.frame_id = fixed_frame
            TSPPose.header.stamp = rospy.Time.now()
            
            TSPPose.pose.position.x = plan[index][0]
            TSPPose.pose.position.y = plan[index][1]
            TSPPose.pose.position.z = plan[index][2]
            
            TSPPose.pose.orientation.x = 0.0
            TSPPose.pose.orientation.y = 0.0
            TSPPose.pose.orientation.z = 0.0
            TSPPose.pose.orientation.w = 1.0
            
            TSPPath.poses.append(TSPPose)
        
        TSPPath.header.stamp = rospy.Time.now()
        # logger.debug("TSP Path: {}".format(TSPPath))
        
        return TSPPath

    def distance_matrix(self,data, metric='euclidean',weighted_height=False):
        """
        Calculates the distance matrix for a given data matrix.

        Args:
            data: A NumPy array of shape (n, p) where n is the number of data points and p is the number of features.
            metric: The distance metric to use ('euclidean', 'manhattan', etc.)

        Returns:
            A NumPy array of shape (n, n) representing the distance matrix.
        """
        n = data.shape[0]
        dist_matrix = np.zeros((n, n))

        for i in range(n):
            for j in range(i, n):
                if metric == 'euclidean':
                    if not weighted_height:
                        
                        distance = np.linalg.norm(data[i] - data[j]) 
                    else:
                        distance = np.linalg.norm(data[i] - data[j]) + 10*(data[i,2] - data[j,2])
                elif metric == 'manhattan':
                    distance = np.sum(np.abs(data[i] - data[j]))
                else:
                    raise ValueError("Unsupported metric")

                dist_matrix[i, j] = distance
                dist_matrix[j, i] = distance  # Distance matrix is symmetric
                
        # print(dist_matrix)

        return dist_matrix
    

    def relocate(solution, cost, cuslist, data):
        bestcostinc = 10000
        for i in cuslist:
            for j in range(0,len(solution)):
                if i in solution[j]:
                   pos = solution[j].index(i)
                   costinc = data[solution[j][pos-1],i] + data[i,solution[j][pos+1]] \
                              - data[solution[j][pos-1],solution[j][pos+1]]
            for j in range(0, len(solution)):
                for k in range(1, len(solution[j])):
                    costinc += data[solution[j][k - 1], i] + data[i, solution[j][k]] \
                              - data[solution[j][k - 1], solution[j][k]]
                    if costinc < bestcostinc:
                        bestcostinc = costinc
                        bestveh = j
                        bestpos = k
                        bestcus = i
        for j in range(0, len(solution)):
            if bestcus in solution[j]:
                pos = solution[j].index(bestcus)
                cost[j] -= data[solution[j][pos - 1], bestcus] + data[bestcus, solution[j][pos + 1]] \
                           - data[solution[j][pos - 1], solution[j][pos + 1]]
                solution[j].remove(bestcus)
        solution[bestveh].insert(bestpos, bestcus)
        cost[bestveh] += bestcostinc

    def simulated_annealing(num_agents, num_tasks, cost_matrix):

        start = time.time()

        # load file
        data = cost_matrix

        # k is the number of vehicles, c is the number of customers
        K = num_agents
        C = num_tasks
        N = K + C
        data[:, :K] = 0

        # initialize customerlist, solution and cost vectors
        solution = []
        cost = []
        for i in range(0,K):
            solution.append([i,i])
            cost.append(0)

        cuslist = []
        for i in range(K,N):
            cuslist.append(i)

        # main loop for greedy insertion, in each loop we insert the customer with lowest cost increment to
        # the whole solution
        while len(cuslist) != 0:
            bestcostinc = 10000
            for i in cuslist:
                for j in range(0, K):
                    for k in range(1, len(solution[j])):
                        costinc = data[solution[j][k - 1], i] + data[i, solution[j][k]] \
                                  - data[solution[j][k - 1], solution[j][k]]
                        if costinc < bestcostinc:
                            bestcostinc = costinc
                            bestcus = i
                            bestveh = j
                            bestpos = k
            cuslist.remove(bestcus)
            solution[bestveh].insert(bestpos,bestcus)
            cost[bestveh] += bestcostinc

        # main loop for simulated annealing
        temperature = 0.1
        bestsol = solution
        bestcost = max(cost)
        tempsol = solution

        cuslist = []
        for i in range(K, N):
            cuslist.append(i)

        for i in range(0,5000):
            solution = tempsol
            PlannerUtils.relocate(solution,cost,cuslist,data)
            if (max(cost)-bestcost)/bestcost < temperature:
                tempsol = solution
                if max(cost) < bestcost:
                   bestsol = solution
                   bestcost = max(cost)
            temperature -= temperature/5000

        end = time.time()
        total_cost = sum(cost)
        total_time =  end - start
        print("maxcost:", max(cost))
        print("totalcost",total_cost )
        print(solution)
        print("the longest route is:", solution[cost.index(max(cost))])
        print("computational time:",total_time)
        
        return solution[0], total_cost,total_time

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

    def calcTotalDistance(routine,distMat):
        '''The objective function. input routine, return total distance.
        cal_total_distance(np.arange(num_points))
        '''
        num_points = len(routine)
        return sum([distMat[routine[i % num_points], routine[(i + 1) % num_points]] for i in range(num_points)])
    
    def getDistanceMatrix(data, metric='euclidean'):
        """
        Calculates the distance matrix for a given data matrix.

        Args:
            data: A NumPy array of shape (n, p) where n is the number of data points and p is the number of features.
            metric: The distance metric to use ('euclidean', 'manhattan', etc.)

        Returns:
            A NumPy array of shape (n, n) representing the distance matrix.
        """
        n = len(data)
        dist_matrix = np.zeros((n, n))

        for i in range(n):
            for j in range(i, n):
                if metric == 'euclidean':
                    distance = np.linalg.norm(data[i] - data[j])
                elif metric == 'manhattan':
                    distance = np.sum(np.abs(data[i] - data[j]))
                else:
                    raise ValueError("Unsupported metric")

                dist_matrix[i, j] = distance
                dist_matrix[j, i] = distance  # Distance matrix is symmetric

        return dist_matrix

    def rms(a, b):
        return np.sqrt(np.mean(np.sum((a - b)**2, axis=1)))

    def getPointMsg(x, y, z):
        p = Point()
        p.x, p.y, p.z = float(x), float(y), float(z)
        return p

    def VisualizeDTWPath(path,cost,threshold):
        
        DTWmarkerArray = MarkerArray()
        
        pathMarker = Marker()
        pathMarker.header.stamp = rospy.Time.now()
        pathMarker.header.frame_id = rospy.get_param("/world_frame")
        
        pathMarker.ns = "fretd_path"
        pathMarker.id = 0
        pathMarker.type = Marker.LINE_STRIP
        pathMarker.action = Marker.ADD
        pathMarker.scale.x = 0.2  # line width
        
        cmap = DTWGradientColorGenerator(ucmap="spring")
        cmin, cmax = float(0.02), float(1.0)
        
        for index,pose in enumerate(path):
            
            pathPoint = PlannerUtils.getPointMsg(pose[0],pose[1],pose[2])
            
            pathMarker.points.append(pathPoint)

            color_map = cmap.get_next_color(cost,threshold,0.02)

            pathColor = ColorRGBA()
            pathColor.r = color_map[0]
            pathColor.g = color_map[1]
            pathColor.b = color_map[2]
            pathColor.a = 1.0
            pathMarker.colors.append(pathColor)
        
        DTWmarkerArray.markers.append(pathMarker)

        return DTWmarkerArray
        
    def VisualizePlanCost(plan_agenda,plan_cost=None):
        
        #Check and warn if plan_cost is invalid
        if not len(plan_cost.rrt_path) > 0:
            logger.warning("Input Plan cost is empty ! ")
            
            return MarkerArray() 
        
        path_length_list = []
        
        cmap = GradientColorGenerator()
        
        for index,path_dist in enumerate(plan_cost.rrt_path_length):
            
            if not plan_agenda.status[index] in ('COMPLETED','FAILED'):
            
                path_length_list.append(path_dist.data)
            
        path_id = 0
        pose_id = 0
        
        cost_array_ = MarkerArray()
        
        for index,path in enumerate(plan_cost.rrt_path):
            

            
            # Visualize the path as a line strip
            path_marker = Marker()
            path_marker.header.frame_id = rospy.get_param('/world_frame')
            path_marker.header.stamp = rospy.Time.now()            

            path_marker.type = Marker.LINE_STRIP
            
            
            path_marker.ns = "rrt_path"
        
            #Create a markerarray 

            path_marker.id = path_id
            

                
            
            path_marker.action = Marker.ADD
                
            path_length = plan_cost.rrt_path_length[index].data
            
            if len(path_length_list) > 1:

                color_map = cmap.get_next_color(path_length,np.max(np.asarray(path_length_list)),np.min(np.asarray(path_length_list)))
            else:
                color_map = cmap.get_next_color(path_length,1000,0.0)

            path_color = ColorRGBA()
            path_color.r = color_map[0]
            path_color.g = color_map[1]
            path_color.b = color_map[2]
            path_color.a = 1.0
            
            path_marker.scale.x = 0.2  # line width

            pose_marker = Marker()
                
            pose_marker.header.frame_id = rospy.get_param('/world_frame')
            pose_marker.header.stamp = rospy.Time.now()
            
            pose_marker.type = Marker.SPHERE_LIST
            pose_marker.action = Marker.ADD
            
            pose_marker.ns = "waypoints"
            pose_marker.id = pose_id
            
            pose_marker.scale.x = 0.5  # sphere radius
            pose_marker.scale.y = 0.5
            pose_marker.scale.z = 0.5
                
            for pose in path.poses:
                
                pose_point = Point()
                pose_point.x = pose.position.x
                pose_point.y = pose.position.y
                pose_point.z = pose.position.z
                
                pose_marker.points.append(pose_point)
                pose_marker.colors.append(path_color)
                
                path_marker.points.append(pose_point)
                path_marker.colors.append(path_color)
                
                # cost_array_.markers.append(pose_marker)
                
                #increase the counter

            if plan_agenda.status[index] in ('COMPLETED','FAILED'):
                path_marker.action = Marker.DELETE
                pose_marker.action = Marker.DELETE
                
            cost_array_.markers.append(pose_marker)
            cost_array_.markers.append(path_marker)
            
            # Increase the counter
            path_id += 1
            pose_id += 1 
            
        return cost_array_   
    
    def fitSigmoid(value):
        
        sigmoid_value = 1 / (1 + np.exp(-value))
        
        return sigmoid_value

    def EvaluateInspectionPlan(InspectionPlan,PlanCost,InspectionAgenda=None):

        if InspectionAgenda is None:

            InspectionAgenda = []

            for k,centroid in enumerate(InspectionPlan.roi_centroids):

                dist = PlanCost.rrt_path_length[k].data
                if InspectionPlan.status[k] == 'Initialized':
                    InspectionPlan.status[k] = 'ACTIVE'
                heapq.heappush(InspectionAgenda, [dist,InspectionPlan.roi_ids[k], InspectionPlan.status[k]])

            active_plan = heapq.heappop(InspectionAgenda)

            current_inspection_plan = active_plan[1]
            
            return InspectionAgenda, current_inspection_plan

        else:

            try:

                agenda = []
                tie = count()  # monotonic tiebreaker
              
                for j, status in enumerate(InspectionPlan.status):
                    if status in ('ACTIVE', 'Initialized'):
                        dist = PlanCost.rrt_path_length[j].data
                        # Store index j in the heap; use counter to avoid tie comparisons
                    else:
                        dist = math.inf
                        
                    heapq.heappush(agenda, (dist, next(tie), j))


                while agenda:
                    _, _, j = heapq.heappop(agenda)
                    if InspectionPlan.status[j] in ('ACTIVE', 'Initialized'):
                        return agenda, j

                return None, None
            
            
                # # print(enumerate(InspectionPlan.status))
                # for j,validPlan in enumerate(InspectionPlan.status):
                    
                #     logger.debug(f"Inspection Plan status:{j,validPlan}")
                    
                #     # if validPlan == 'ACTIVE' or validPlan == 'Initialized':
                #     if not validPlan == 'COMPLETED':
                #         dist = PlanCost.rrt_path_length[j].data
                #         # InspectionPlan.status[k] = 'ACTIVE'
                #         heapq.heappush(InspectionAgenda, [dist,InspectionPlan.roi_ids[j], InspectionPlan.status[j]])
                #     else: #validPlan == 'COMPLETED'or validPlan == 'FAILED':
                #         logger.warning("INSP PLAN is updated with dist 100000")
                #         dist = 100000
                #         heapq.heappush(InspectionAgenda, [dist,InspectionPlan.roi_ids[j], InspectionPlan.status[j]])
                        
                
                # active_plan = heapq.heappop(InspectionAgenda)
                
                # if InspectionPlan.status[active_plan[1]] == 'ACTIVE':
                #     current_inspection_plan = active_plan[1]
                #     logger.debug("Returning the next active plan")
                #     return InspectionAgenda, current_inspection_plan
                # else:
                #     logger.warning(f"PLAN IS {InspectionPlan.status[active_plan[1]]}")
                #     current_inspection_plan = active_plan[1]
                #     # logger.warning(f"ActivePlan status is: {active_plan.status} and the ROI is {current_inspection_plan} ")
                        
                #     return None, None

            except IndexError:
                
                return None,None
            
    class InspectionPerformance():
        def __init__(self):
            
            self.header = Header()
            self.roi_id = String()
            self.total_waypoints = Float64()
            self.remaining_waypoints = Float64()
            self.fret_dtw_cost = Float64()
            self.view_planning_time = Float64()
            self.fret_compute_time = Float64()
            self.fret_dtw_confidence = Float64()
            self.kabsch_compute_time = Float64()
            self.kabsch_rmse_before = Float64()
            self.kabsch_rmse_after = Float64()
            self.current_mission_status = String()
            self.maintained_distance = Float64()
        

    class InspectionPlan():
        def __init__(self):
            
            self.number_of_rois = None
            self.roi_ids = []
            self.rois = []
            self.roi_areas = []
            self.roi_centroids = []
            self.ideal_plans = []
            self.ideal_attitudes = []
            self.planned_targets_pos = []
            self.viz_plans = []
            self.status = []

    class ViewPosePlanner(InspectionPlan):

        def __init__(self):

            # Parameters (can be set via ROS params or defaulted here)
            self.viewing_distance = rospy.get_param("/inspection_distance", 3.0)  # distance along normal
            cameraFOV =      rospy.get_param("/fov", [69.4,45])          
            overlapParam = rospy.get_param("/photogrammetric_params", [0.6,0.5])                  # vertical overlap fraction
            
            logger.debug(f"overlap: {overlapParam}")
            
            self.k = rospy.get_param("~k", 4)
            self.adapt_to_roi_angle = rospy.get_param("~roi_angle",False)
            self.glocal_adaptive = rospy.get_param("~adaptive_to_surface", 0.5)
            
            self.fov_h_deg = cameraFOV[0]
            self.fov_v_deg = cameraFOV[1]
            self.overlap_h = overlapParam[0]
            self.overlap_v = overlapParam[1]
            
    
        # --- Helper functions (adapted from the original code) ---
        def compute_camera_footprint(self, distance, fov_h_deg, fov_v_deg):
            fov_h = np.deg2rad(fov_h_deg)
            fov_v = np.deg2rad(fov_v_deg)
            width = 2 * distance * np.tan(fov_h / 2)
            height = 2 * distance * np.tan(fov_v / 2)
            return width, height


        def look_at(self,eye, center, up):
            f = center - eye
            f = f / np.linalg.norm(f)
            s = np.cross(f, up)
            if np.linalg.norm(s) < 1e-6:
                s = np.array([0, 1, 0])
            else:
                s = s / np.linalg.norm(s)
            u = np.cross(s, f)
            T = np.eye(4)
            T[0, :3] = s
            T[1, :3] = u
            T[2, :3] = -f
            T[:3, 3] = eye

            return T

        def compute_camera_pose_local(self,target_point, viewing_distance, roi_points, k=20, preferred_up=np.array([0,0,1])):
            """
            Compute a camera pose for a target point by estimating the local surface normal.
            The camera is placed at target_point + viewing_distance * local_normal.
            The camera is then oriented to look at the target, with the 'up' direction computed 
            from the local normal and a preferred up (which can help disambiguate the roll).
            """
            local_normal = self.compute_local_normal(target_point, roi_points, k)
            camera_position = target_point + viewing_distance * local_normal
            
            # For the up direction: remove any component of the preferred_up along the camera's view direction.
            view_dir = target_point - camera_position
            view_dir = view_dir / np.linalg.norm(view_dir)
            proj = np.dot(preferred_up, view_dir) * view_dir
            camera_up = preferred_up - proj
            if np.linalg.norm(camera_up) < 1e-6:
                camera_up = np.array([0, 0, 1])
            else:
                camera_up = camera_up / np.linalg.norm(camera_up)
            
            T = self.look_at(camera_position, target_point, camera_up)
            return T, local_normal


        def compute_local_normal(self, query_point, roi_points, k=20):
            """
            Estimate the local normal at 'query_point' using PCA on the k nearest neighbors.
            """
            roi_points_np = np.array(roi_points)
            nbrs = NearestNeighbors(n_neighbors=k, algorithm='auto').fit(roi_points_np)
            distances, indices = nbrs.kneighbors([query_point])
            neighbors = roi_points_np[indices[0]]
            cov = np.cov(neighbors.T)
            # eigvals, eigvecs = np.linalg.eig(cov)
            eigvals, eigvecs = np.linalg.eigh(cov)
            normal = eigvecs[:, np.argmin(eigvals)]

            if normal[2] < 0:
                normal = -normal
                
            return normal

        def is_behind_plane(self, point, plane_centroid, plane_normal):
            """Check if point is behind the plane (opposite side of normal)"""
            vec_to_point = point - plane_centroid
            return np.dot(vec_to_point, plane_normal) < 0

        def compute_best_fit_plane(self, roi_points):
            """Compute plane normal with consistent outward orientation"""
            roi_array = np.array(roi_points)
            centroid = np.mean(roi_array, axis=0)
            
            # 1. Compute PCA normal
            cov = np.cov(roi_array.T)
            _, eigvecs = np.linalg.eigh(cov)
            normal = eigvecs[:, 0]

            if not self.adapt_to_roi_angle:
                normal[2] = 0
            
            # 2. Compute polygon normal from vertex winding order
            if len(roi_points) >= 3:
                p0, p1, p2 = roi_array[0], roi_array[1], roi_array[2]
                edge1 = p1 - p0
                edge2 = p2 - p0
                poly_normal = np.cross(edge1, edge2)
                poly_normal /= np.linalg.norm(poly_normal)
                
                # Align PCA normal with polygon winding normal
                if np.dot(normal, poly_normal) < 0:
                    normal *= -1

            # Choose a reference vector that is not parallel to the normal.
            ref = np.array([1, 0, 0]) if abs(normal[0]) < 0.9 else np.array([0, 1, 0])
            local_x = np.cross(normal, ref)
            local_x = local_x / np.linalg.norm(local_x)
            local_y = np.cross(normal, local_x)

            return centroid, -normal,local_x, local_y

        def transform_to_local(self,roi_points, centroid, local_x, local_y):
            """
            Transform ROI points into the local 2D coordinate system defined by local_x and local_y.
            """
            local_points = []
            for pt in roi_points:
                pt_rel = np.array(pt) - centroid
                x = np.dot(pt_rel, local_x)
                y = np.dot(pt_rel, local_y)
                local_points.append([x, y])
            return np.array(local_points)

        def compute_camera_pose_fixed_normal(self,target_point, viewing_distance, roi_plane_normal, centroid, preferred_up=np.array([0, 0, 1])):

            # Position the camera so that the view direction is along the ROI's positive normal.
            camera_position = target_point - viewing_distance * roi_plane_normal

            # Compute the viewing direction (should be aligned with roi_plane_normal).
            view_dir = target_point - camera_position
            view_dir = view_dir / np.linalg.norm(view_dir)
            
            # Compute an up vector that’s orthogonal to the view direction.
            proj = np.dot(preferred_up, view_dir) * view_dir
            camera_up = preferred_up - proj
            if np.linalg.norm(camera_up) < 1e-6:
                camera_up = np.array([0, 0, 1])
            else:
                camera_up = camera_up / np.linalg.norm(camera_up)
                    # Verify camera position is in front of ROI

            # Create the transformation matrix using the look_at helper.
            T = self.look_at(camera_position, target_point, camera_up)

            return T

        def generate_grid_points_on_plane(self,
                                        local_points,
                                        centroid,
                                        local_x,
                                        local_y,
                                        footprint_width,
                                        footprint_height,
                                        overlap_h,
                                        overlap_v):
            """
            Generate grid points on the best-fit plane in global coordinates,
            starting from the center of the local bounding box and expanding outward.
            """

            # Create a 2D polygon from local ROI points
            poly = Polygon(local_points)
            min_x, min_y, max_x, max_y = poly.bounds

            # Compute grid steps
            step_y = footprint_width - footprint_width * (overlap_h)
            step_x = footprint_height - footprint_height * (overlap_v)


            # ---------------------------------------------------------
            # 1. Identify the bounding box center in local 2D
            center_x = 0.5 * (min_x + max_x)
            center_y = 0.5 * (min_y + max_y)

            # 2. Compute half-ranges
            half_range_x = 0.5 * (max_x - min_x)
            half_range_y = 0.5 * (max_y - min_y)

            # 3. Determine how many steps we can take in each direction
            #    (Add +1 to ensure we include endpoints)
            num_steps_x = int(half_range_x / step_x) + 1
            num_steps_y = int(half_range_y / step_y) + 1

            # 4. Build arrays that start from the center and expand outward.
            #    For x, we go from -num_steps_x to +num_steps_x. Same for y.
            #    Then we shift by center_x / center_y.
            x_candidates = [
                center_x + (i * step_x)
                for i in range(-num_steps_x, num_steps_x + 1)
            ]
            y_candidates = [
                center_y + (j * step_y)
                for j in range(-num_steps_y, num_steps_y + 1)
            ]
            # ---------------------------------------------------------

            grid_points_local = []

            for x in x_candidates:
                # If x is outside the bounding box, skip it for minor performance improvement
                if x <= min_x - 2*step_x or x >= max_x + 2*step_x:
                    continue

                for y in y_candidates:
                    # Same bounding‐box check for y (optional optimization)
                    if y <= min_y - 2*step_y or y >= max_y + 2*step_y:
                        continue

                    # Check if the point is inside or on the boundary
                    pt_local = (x, y)
                    if poly.contains(ShapelyPoint(pt_local)) or poly.touches(ShapelyPoint(pt_local)):
                        grid_points_local.append(pt_local)

            # Transform local 2D grid points back to global (3D) coordinates
            grid_points_global = []
            for (lx, ly) in grid_points_local:
                pt_global = centroid + lx * local_x + ly * local_y
                grid_points_global.append(pt_global)

            return grid_points_global

        def generate_view_poses_nonplanar(self, roi_points, viewing_distance, fov_h_deg, fov_v_deg, overlap_h, overlap_v, k=20):
            """
            Generate camera poses for each grid target on a non-planar ROI.
            """
            footprint_width, footprint_height = self.compute_camera_footprint(viewing_distance, fov_h_deg, fov_v_deg)
            # Step 1: Compute best-fit plane.
            centroid, normal, local_x, local_y = self.compute_best_fit_plane(roi_points)

            self.k = len(roi_points) 

            # Step 2: Transform ROI points into the plane's local 2D coordinates.
            local_roi = self.transform_to_local(roi_points, centroid, local_x, local_y)

        
            grid_points_global = self.generate_grid_points_on_plane(local_roi, centroid, local_x, local_y,
                                                                footprint_width, footprint_height,
                                                                overlap_h, overlap_v)

            camera_poses = []
            local_normals = []

            for pt in grid_points_global:
                # T, l_normal = self.compute_camera_pose_local(pt, viewing_distance, roi_points, self.k)
                T = self.compute_camera_pose_fixed_normal(pt, viewing_distance, normal, centroid)

                camera_poses.append(T)
                local_normals.append(normal)
                
            proj_centroid = centroid - viewing_distance * local_normals[0]
            
            cqx,cqy,cqz,cqw = PlannerUtils.GetLookAtOrientation(proj_centroid,centroid)
            
            centroidPose = np.array([proj_centroid[0],proj_centroid[1],proj_centroid[2],cqx,cqy,cqz,cqw])

            return camera_poses, grid_points_global, footprint_width, footprint_height, local_normals,centroidPose
        

        def GenerateIdealInspectionPlans(self, roi_polygons):

            # Use the header frame (global fixed frame) for all computations/visualization.
            
            # Create markers for visualization.
            all_marker_array = MarkerArray()
            marker_id = 0
            
            InspPlan = PlannerUtils.InspectionPlan()

            InspPlan.number_of_rois = len(roi_polygons.polygons)

            id_counter = 0
            
            for msg in roi_polygons.polygons:

                marker_array = MarkerArray()

                InspPlan.rois.append(msg.polygon)
                InspPlan.roi_ids.append(id_counter)
                id_counter +=1

                frame_id = msg.header.frame_id

                # Convert the polygon points (geometry_msgs/Point32) into a list of [x, y, z] points.
                roi_points = []
                for pt in msg.polygon.points:
                    roi_points.append([pt.x, pt.y, pt.z])
                
                ## Verify if the polygon is in the proper orienttation CCW
                
                # polygon = shapely.geometry.Polygon(roi_points)
                # polygon  = shapely.geometry.polygon.orient(polygon, sign=1)
                # roi_points = list(polygon.exterior.coords)

                # Generate view poses (camera positions, target points, and local normals)
                camera_poses, grid_targets, footprint_width, footprint_height, local_normals,roi_centroid = \
                    self.generate_view_poses_nonplanar(roi_points, self.viewing_distance,
                                                    self.fov_h_deg, self.fov_v_deg,
                                                    self.overlap_h, self.overlap_v, self.k)
                # print(local_normals)
                # print(roi_centroid)
                
                InspPlan.roi_centroids.append(roi_centroid)
                InspPlan.status.append('Initialized')

                # ROI Polygon Marker (red LINE_STRIP)
                roi_marker = Marker()
                roi_marker.header.frame_id = frame_id
                roi_marker.header.stamp = rospy.Time.now()
                roi_marker.ns = "roi_polygon"
                roi_marker.id = marker_id
                marker_id += 1
                roi_marker.type = Marker.LINE_STRIP
                roi_marker.action = Marker.ADD
                roi_marker.scale.x = 0.1  # line width
                roi_marker.color.r = 1.0
                roi_marker.color.g = 0.0
                roi_marker.color.b = 0.0
                roi_marker.color.a = 1.0
                for pt in roi_points:
                    p = Point()
                    p.x, p.y, p.z = pt
                    roi_marker.points.append(p)
                # Ensure the polygon is closed.
                if roi_points[0] != roi_points[-1]:
                    p = Point()
                    p.x, p.y, p.z = roi_points[0]
                    roi_marker.points.append(p)
                all_marker_array.markers.append(roi_marker)
                marker_array.markers.append(roi_marker)

                cam_pos_list = []
                cam_att_list = []
                target_pos_list = []

                # For each computed camera pose, add visualization markers.
                for i, (T, target) in enumerate(zip(camera_poses, grid_targets)):
                    # Extract camera position from T.
                    cam_pos = T[:3, 3]
                    quat = tf.transformations.quaternion_from_matrix(T)
                    
                    cam_pos_list.append(cam_pos)
                    target_pos_list.append(target)

                    # Marker for camera position (blue sphere).
                    cam_marker = Marker()
                    cam_marker.header.frame_id = frame_id
                    cam_marker.header.stamp = rospy.Time.now()
                    cam_marker.ns = "camera_pose"
                    cam_marker.id = marker_id
                    marker_id += 1
                    cam_marker.type = Marker.SPHERE
                    cam_marker.action = Marker.ADD
                    cam_marker.pose.position.x = cam_pos[0]
                    cam_marker.pose.position.y = cam_pos[1]
                    cam_marker.pose.position.z = cam_pos[2]
                    cam_marker.pose.orientation.x = 0
                    cam_marker.pose.orientation.y = 0
                    cam_marker.pose.orientation.z = 0
                    cam_marker.pose.orientation.w = 1
                    cam_marker.scale.x = 0.5
                    cam_marker.scale.y = 0.5
                    cam_marker.scale.z = 0.5
                    cam_marker.color.r = 1.0
                    cam_marker.color.g = 0.0
                    cam_marker.color.b = 0.0
                    cam_marker.color.a = 1.0
                    all_marker_array.markers.append(cam_marker)
                    marker_array.markers.append(cam_marker)

                    # Marker for camera view direction (green arrow from camera to target).
                    arrow_marker = Marker()
                    arrow_marker.header.frame_id = frame_id
                    arrow_marker.header.stamp = rospy.Time.now()
                    arrow_marker.ns = "camera_direction"
                    arrow_marker.id = marker_id
                    marker_id += 1
                    arrow_marker.type = Marker.ARROW
                    arrow_marker.action = Marker.ADD
                    arrow_marker.points = []
                    start_pt = Point()
                    start_pt.x, start_pt.y, start_pt.z = cam_pos
                    end_pt = Point()
                    end_pt.x, end_pt.y, end_pt.z = target
                    
                    qx,qy,qz,qw = PlannerUtils.GetLookAtOrientation(cam_pos, target)
                    cam_att_list.append([qx,qy,qz,qw])
                    arrow_marker.points.append(start_pt)
                    arrow_marker.points.append(end_pt)
                    arrow_marker.scale.x = 0.2  # shaft diameter
                    arrow_marker.scale.y = 0.4  # head diameter
                    arrow_marker.color.r = 0.0
                    arrow_marker.color.g = 1.0
                    arrow_marker.color.b = 0.0
                    arrow_marker.color.a = 1.0
                    all_marker_array.markers.append(arrow_marker)
                    marker_array.markers.append(arrow_marker)

                InspPlan.ideal_plans.append(cam_pos_list)
                InspPlan.ideal_attitudes.append(cam_att_list)
                InspPlan.planned_targets_pos.append(target_pos_list)
                InspPlan.viz_plans.append(marker_array)


            return all_marker_array,InspPlan
        
    

            
    
        
        
        
        
        
    
    