import open3d as o3d
import collections 
import numpy as np
import cv2
import rospy
import rospkg
import struct
import copy 
import threading
import tf
import gc
import tf2_ros
import tf2_geometry_msgs
from cv_bridge import CvBridge
from nav_msgs.msg import Odometry
from sensor_msgs.msg import Image  
from std_msgs.msg import Bool
from sensor_msgs.msg import PointCloud2
import sensor_msgs.point_cloud2 as pc2
from std_msgs.msg import Header
from message_filters import ApproximateTimeSynchronizer, Subscriber
from geometry_msgs.msg import TransformStamped
from geometry_msgs.msg import PoseWithCovarianceStamped
from geometry_msgs.msg import PointStamped
from sensor_msgs.msg import PointField

#dynamic_reconfigure 
from dynamic_reconfigure.server import Server
from localization_correction.cfg import ICPConfigConfig
from transform_utils import rotate_to_ros, enforce_yaw_only, pose_to_matrix
from debug_utils import publish_amcl_pose_point,publish_center,publish_position_estimation
class ICPLocalizer:
    def __init__(self):
        self.max_dist = 1.5
        # init
        rospy.init_node('icp_localization', anonymous=True)
        rospy.loginfo(f"[ICPLocalizer] Initializing...")

        # Intrinsics
        self.intrinsic = o3d.camera.PinholeCameraIntrinsic()
        self.intrinsic.set_intrinsics(
                                        width=640, height=480,
                                        fx=615.03, fy=615.19,
                                        cx=326.34, cy=237.27                                     )
        
        # parameter tuning 
        self.voxel_size = 0.02 #세밀 = 작은 값       
        self.icp_max_iteration = 200 #최대 반복 횟수 
        self.fitness_threshold = 0.8 #fitness 값 기준 (너무 낮게 하면 적은 수의 포인트 정합 성립 -> 오차 커짐)
        self.icp_max_correspondence_distance = 0.2 #0.1~0.5 #최대 대응 거리 (좀 넉넉히 하면 더 많은 포인트 매칭 -> 정확도를 높이려면 더 작은 값)
        self.depth_trunc = 6.0 #depth 거리 기준 
        self.rmse_weight = 0.6  
        self.yaw_penalty_weight = 0.2 #yaw오차가 중요한 경우 높게 설정해서 yaw보정 
        self.score_threshold = 0.4 #높게 설정하면 정확도가 높아야 정합을 인정 
        self.location_penalty_weight =0.1 #위치 오차 중요하면 높게 설정 
        
        # Internal variables
        self.bridge = CvBridge() 
        self.listener = tf.TransformListener()
        #self.latest_linear_vel = 0.0  # 최근 속도 저장용
        self.amcl_pose_msg = None
        self.amcl_position = None
        self.icp_initialized = False 
        self.last_icp_pose = None
        self.debug_mode = True
        self.amcl_warn_logged = False
        
        self.icp_fail_count = 0      
        self.icp_fail_threshold = 3

        self.offsets = [
                        [0.0, 0.0],
                        [0.5, 0.0], [-0.5, 0.0],
                        [0.0, 0.5], [0.0, -0.5]
                    ]              
        #publishers 
        self.icp_cloud_pub = rospy.Publisher("/icp_cloud", PointCloud2, queue_size=10) #실시간 포인트클라우드 
        self.map_cloud_pub = rospy.Publisher("/map_cloud", PointCloud2, queue_size=1, latch = True) #기존 3d map 포인트클라우드
        self.icp_pose_pub = rospy.Publisher('/icp_pose', PoseWithCovarianceStamped, queue_size=10) #위치 추정 결과 
        self.icp_done_pub = rospy.Publisher('/icp_done', Bool, queue_size=10)  #초기 상태 반환 
        self.icp_initial_pose_pub = rospy.Publisher('/icp_done', Bool, queue_size=1,latch = True) 

        # Debug publishers
        self.debug_amcl_pub = rospy.Publisher("/amcl_debug", PointStamped, queue_size=1)
        self.debug_estimation_point_pub = rospy.Publisher("/estimation_debug", PointStamped, queue_size=1)
        self.debug_crop_point_pub = rospy.Publisher("/crop_center_debug", PointStamped, queue_size=1)
        self.debug_curr_point_pub = rospy.Publisher("/curr_center_debug", PointStamped, queue_size=1)
           
        # Load npy 
        self.T_3d_to_2d_path = rospy.get_param('~T_3d_to_2d')
        self.T_3d_to_2d = np.load(self.T_3d_to_2d_path)
        
        self.T_2d_to_3d = np.linalg.inv(self.T_3d_to_2d) # 역행렬 
        print("T_3d_to_2d:\n",self.T_3d_to_2d)
        print("T_2d_to_3d:\n",self.T_2d_to_3d)

        # Load 3D Map 
        self.reference_map = self.load_map("map/renaissance_0416_3_good_denoised.ply")        

        #self.reference_map = self.load_map("map/myungsin_aligned.ply")        
        self.reference_map = self.reference_map.voxel_down_sample(voxel_size=self.voxel_size) # downsampling
        
        self.reference_map.estimate_normals(search_param=o3d.geometry.KDTreeSearchParamHybrid(radius=0.2, max_nn=50))
        self.reference_map = self.filter_pcd(self.reference_map, use_normal = True, use_height = True)
           
        # Visualize            
        self.initial_guess = None
        self.reference_map_vis = copy.deepcopy(self.reference_map)
        self.reference_map_vis.transform(self.T_3d_to_2d) # align to 2d  
        
        self.publish_reference_map_once()
        
        # Subscribers
        #self.amcl_pose_sub = rospy.Subscriber('/initialpose', PoseWithCovarianceStamped, self.amcl_pose_callback) #rviz에서 2D pose estimate로 위치 지정 후에 진행하는 경우 
        self.amcl_pose_sub = rospy.Subscriber('/amcl_pose', PoseWithCovarianceStamped, self.amcl_pose_callback) 
        self.odom_sub = rospy.Subscriber('/odom', Odometry, self.odom_callback)
        
        # RGBD Subscribers (Synchronized)
        self.rgb_sub = Subscriber('/camera/color/image_raw', Image)
        self.depth_sub = Subscriber('/camera/aligned_depth_to_color/image_raw', Image) 
        ats = ApproximateTimeSynchronizer(
            [self.rgb_sub, self.depth_sub],
            queue_size=30,
            slop=0.3          )        
        ats.registerCallback(self.rgbd_callback) 
       


    # 3D 맵 불러오기 (utils로 이동?)
    def load_map(self, relative_path):
        pkg_path = rospkg.RosPack().get_path('localization_correction')
        map_path = f"{pkg_path}/{relative_path}"
        return o3d.io.read_point_cloud(map_path)

    # 레퍼런스 3D 맵 발행 (rgb 미포함)
    def publish_reference_map_once(self):
        header = Header(stamp = rospy.Time.now(),frame_id = "map")  
        pc_data = pc2.create_cloud_xyz32(header, np.asarray(self.reference_map_vis.points).tolist())
        self.map_cloud_pub.publish(pc_data)
        rospy.loginfo("[Map] Reference map published (latched).")

    # rgb, depth data 동기화 
    def preprocess_rgbd(self,rgb_msg, depth_msg):
        rgb_img = self.bridge.imgmsg_to_cv2(rgb_msg, desired_encoding="rgb8")
        rgb_img = np.ascontiguousarray(rgb_img) # 메모리 연속 배열
        depth_img = self.bridge.imgmsg_to_cv2(depth_msg, desired_encoding="passthrough").astype(np.uint16)
        depth_img = np.ascontiguousarray(depth_img) # 메모리 연속 배열

        rgbd_image = o3d.geometry.RGBDImage.create_from_color_and_depth(
            o3d.geometry.Image(rgb_img), o3d.geometry.Image(depth_img.astype(np.uint16)), 
            depth_scale=1000.0, depth_trunc=self.depth_trunc, convert_rgb_to_intensity=False        )

        curr_pcd = o3d.geometry.PointCloud.create_from_rgbd_image(
            rgbd_image, self.intrinsic)
        
        if not curr_pcd.has_points():
            rospy.logwarn(f"curr_pcd is empty!")

        curr_pcd = curr_pcd.voxel_down_sample(voxel_size=self.voxel_size) # downsampling
        if self.debug_mode:
            print(f"curr_pcd points:{len(curr_pcd.points)}")
        
        return curr_pcd

    def compute_icp_score(self, fitness, rmse):
        if fitness is None or rmse is None:
            return -float('inf')  
        score = self.rmse_weight * fitness - (1 - self.rmse_weight) * rmse
        return score

    # amcl_pose를 3d기준으로 변환해서 중심점으로 잡아 3dmap을 crop
    def crop_map_based_on_amcl_position(self, curr_pcd):
        if self.amcl_pose_msg is None:
            rospy.logwarn("/amcl_pose not arrived!")
            return None  
        
        # 기준 pose
        pose_matrix = self.amcl_pose_matrix
        center = pose_matrix[:3, 3]
        rotation = pose_matrix[:3, :3]

        # 회전행렬에서 yaw 추출
        yaw = np.arctan2(rotation[1, 0], rotation[0, 0])
        cos_yaw, sin_yaw = np.cos(yaw), np.sin(yaw)

        # FoV offset 적용 (후방 2m, 전방 6m → 총 depth=8m, 좌우는 6.5m씩 → 총 width=13m)
        # 맵 기준 front가 x방향, side가 y방향이라고 가정 (Open3D 좌표계)
        fov_center_offset = np.array([
            (6.0 - 2.0) / 2 * cos_yaw,  # x방향 중심 이동 (전방-후방 길이의 절반만큼 전방으로)
            (6.0 - 2.0) / 2 * sin_yaw,
            0.0
        ])

        center = center + fov_center_offset
        center[2] = self.amcl_pose_matrix[2, 3]  # 높이는 그대로

        # FoV 영역 크기 (x: depth=8m, y: width=13m, z: height=4m)
        extent = np.array([8.0, 13.0, 4.0])  # 세로(x), 가로(y), 높이(z)

        # 크롭 박스 생성
        obb = o3d.geometry.OrientedBoundingBox(center, rotation, extent)
        cropped_map = self.reference_map.crop(obb)

        if self.debug_mode:
            print(f"[FoV Crop] Cropped_map Size: {np.asarray(cropped_map.points).shape}")
            center_h = np.append(center, 1.0)
            center_in_2dmap = self.T_3d_to_2d @ center_h
            publish_center(center_in_2dmap[:3], self.debug_crop_point_pub)

        return cropped_map

    def crop_map_based_on_last_icp_pose(self):
        if self.last_icp_pose is None:
            rospy.logwarn("No last_icp_pose available!")
            return None

        center = self.last_icp_pose[:3, 3]  # 위치 벡터 추출
        rotation = self.last_icp_pose[:3, :3]

        center[2] = self.last_icp_pose[2, 3]
        if self.debug_mode:
            print(f"Crop center z set to last_icp_pose center z = {center[2]:.2f}")        
        

        extent = np.array([15.0, 15.0, 7.0])
        obb = o3d.geometry.OrientedBoundingBox(center, rotation, extent)
        cropped_map = self.reference_map.crop(obb)
        if self.debug_mode:
            print(f"[based on last_icp_pose] Cropped_map Size: {np.asarray(cropped_map.points).shape}")

    
        if self.debug_mode:
            center_h = np.append(center, 1.0) #T_3d_to_2d와 연산을 위해 4D homogeneous 좌표로 변환
            center_in_2dmap = self.T_3d_to_2d @ center_h
  
            publish_center(center_in_2dmap[:3], self.debug_crop_point_pub) 

        return cropped_map

    def crop_map(self, curr_pcd):
        if not self.icp_initialized:
            if self.amcl_position is not None:
                return self.crop_map_based_on_amcl_position(curr_pcd)
            else:
                return self.crop_map_fallback(curr_pcd)
        else:
            return self.crop_map_based_on_last_icp_pose()
   
    def filter_cropped_map(self, cropped_map):
        if len(cropped_map.points) < 3:  # 최소값 ransac_n보다 작으면
            rospy.logwarn("Too few points in cropped_map for plane segmentation. Skipping filtering.")
            return cropped_map  # 그냥 원본 반환하거나 None으로 처리

        plane_model, inliers = cropped_map.segment_plane(distance_threshold=0.02, ransac_n=3, num_iterations=1000)
        cropped_map = cropped_map.select_by_index(inliers, invert=True)  # 평면 아닌 것만 남김
        points = np.asarray(cropped_map.points)
        normals = np.asarray(cropped_map.normals)
        
        #여기서 튜닝 
        mask_z = np.logical_and(points[:, 2] > -1.5, points[:, 2] < 3.0)  # 높이 필터링
        mask_normal = np.abs(normals[:, 2]) < 0.98                        # 수직 벽 필터링

        mask = np.logical_and(mask_z, mask_normal)
        filtered = cropped_map.select_by_index(np.where(mask)[0])
        print(f"[filter_cropped_map] filtered points: {len(filtered.points)}")

        return filtered

    # 포인트 클라우드를 맵 좌표계에 정합된 위치로 변환하고 발행. (Open3D 시각화 포함)
    def visualize_curr_icp_result(self, curr_pcd, cropped_map=None, T_icp=None, frame_id="map"):
        if not curr_pcd.has_points():
            rospy.logwarn(f"curr_pcd point clouds is empty!")
            return

        if T_icp is None:
            rospy.logwarn("No T_icp provided for visualization.")
            return
        
        curr_pcd_transformed = copy.deepcopy(curr_pcd)
        curr_pcd_transformed.transform(T_icp)  # 정합된 위치로 이동

        pc_data = self.pointcloud_to_colored_rosmsg(curr_pcd_transformed, frame_id=frame_id)
        if pc_data :
            self.icp_cloud_pub.publish(pc_data)

    def pointcloud_to_colored_rosmsg(self, pcd, frame_id="map"):
        header = Header()
        header.stamp = rospy.Time.now()
        header.frame_id = frame_id

        points = np.asarray(pcd.points)
        colors = np.asarray(pcd.colors)

        if points.shape != colors.shape:
            rospy.logwarn("Points and colors shape mismatch in PointCloud conversion.")
            return None

        cloud_data = []
        for pt, color in zip(points, colors):
            r, g, b = (color * 255).astype(np.uint8)
            rgb_uint32 = struct.unpack('I', struct.pack('BBBB', b, g, r, 0))[0]
            cloud_data.append([pt[0], pt[1], pt[2], rgb_uint32])

        fields = [
            PointField('x', 0, PointField.FLOAT32, 1),
            PointField('y', 4, PointField.FLOAT32, 1),
            PointField('z', 8, PointField.FLOAT32, 1),
            PointField('rgb', 12, PointField.UINT32, 1),
        ]

        return pc2.create_cloud(header, fields, cloud_data)

    # def remove_out_of_height_range(self, pcd, z_min=-0.1, z_max=2.7):
    #     points = np.asarray(pcd.points)
    #     mask = np.logical_and(points[:,2] > z_min, points[:,2] < z_max)
    #     filtered_pcd = pcd.select_by_index(np.where(mask)[0])
    #     return filtered_pcd

    #z축 필터링 (바닥, 천장 제외), 노멀 => 튜닝 시도 (함수 다시 검토 후 수정 예정)
    def filter_pcd(self,pcd,use_normal=True,use_height=True,z_min=-1.0,z_max=2.7,normal_threshold=0.95):
        if use_normal and not pcd.has_normals():
            pcd.estimate_normals(
                search_param=o3d.geometry.KDTreeSearchParamHybrid(radius=0.2, max_nn=50)
            )

        mask = np.ones(len(pcd.points), dtype=bool)

        if use_height:
            points = np.asarray(pcd.points)
            mask_z = np.logical_and(points[:, 2] > z_min, points[:, 2] < z_max)
            mask = np.logical_and(mask, mask_z)

        if use_normal:
            normals = np.asarray(pcd.normals)
            mask_normal = np.abs(normals[:, 2]) < normal_threshold
            mask = np.logical_and(mask, mask_normal)

        return pcd.select_by_index(np.where(mask)[0])
    def get_start_pose(self):
        if self.icp_initialized: # 첫 번째 이후에는 last_icp_pose를 기준으로 초기 위치 추정
            start_pose = self.last_icp_pose.copy() # last_icp_pose를 초기 추정값으로 사용
        else: 
            # 첫 번째 정합은 AMCL pose 사용 - initial_guess를 3D 맵 기준으로 변환
            start_pose =  self.amcl_pose_matrix.copy() #3d맵 기준임 
        if self.debug_mode:
            print("start_pose[2, 3] BEFORE:", start_pose[2, 3])
        
        z_min = np.min(np.asarray(self.reference_map.points)[:, 2]) 
        z_start = z_min + 1.0
        start_pose[2, 3] = z_start

        if self.debug_mode:
            print("start_pose[2, 3] AFTER:", start_pose[2, 3])

        return start_pose

    def extract_yaw(self, matrix):
        return np.arctan2(matrix[1, 0], matrix[0, 0]) 
    
    
    def run_coarse_icp(self, curr_pcd, cropped_map, start_pose):
        print("[run_coarse_icp]")
        # Coarse ICP 수행
        coarse_map = cropped_map.voxel_down_sample(voxel_size=0.1)  # coarse 단계에서는 큰 voxel size 사용
        reg_coarse = o3d.pipelines.registration.registration_icp(
            curr_pcd, coarse_map, self.icp_max_correspondence_distance, start_pose,
            estimation_method=o3d.pipelines.registration.TransformationEstimationForColoredICP(),
            criteria=o3d.pipelines.registration.ICPConvergenceCriteria(max_iteration=self.icp_max_iteration)
        )
        print(f"[coarse_icp] Fitness: {reg_coarse.fitness:.4f}, RMSE: {reg_coarse.inlier_rmse:.4f}, Correspondences: {len(reg_coarse.correspondence_set)}")
        print(f"[coarse_icp] transformation :")
        print(reg_coarse.transformation)
        return reg_coarse.fitness, reg_coarse.inlier_rmse, reg_coarse.transformation
    
    def run_fine_icp(self, curr_pcd, cropped_map, start_pose):
        print("[run_fine_icp]")
        # Fine ICP 수행
        fine_map = cropped_map  # 원본 맵 그대로 사용
        best_fitness = -1.0
        best_rmse = None
        best_T = None
        best_score = -float('inf')  
        best_yaw_deg = None
        best_correspondences = 0

        # ICP 등록 파라미터 설정
        criteria = o3d.pipelines.registration.ICPConvergenceCriteria(max_iteration=self.icp_max_iteration) #100
        # 1) lambda_geometric 을 설정한 estimation_method 객체 생성
        est_method = o3d.pipelines.registration.TransformationEstimationForColoredICP(
                lambda_geometric=0.9
            )

        # coarse 단계에서 얻은 초기 추정값을 기반으로 fine 단계 시작
        true_yaw = self.extract_yaw(start_pose) # 시작 yaw 추출
        initial_position = start_pose[:3, 3] 
        pose = start_pose.copy()

        print(f"--------------------------------------------------------------------------------")
        print(f"testing yaw...")


        for yaw_deg in range(-45, 46, 15): # -60°에서 60°까지 15° 간격으로 yaw 값을 시도
            yaw_offset = np.deg2rad(yaw_deg)  # yaw_deg 각도를 라디안으로 변환
            R_yaw = tf.transformations.euler_matrix(0, 0, yaw_offset)
            guess = pose @ R_yaw

            # ICP 정합 수행 
            reg_fine = o3d.pipelines.registration.registration_icp(
                curr_pcd, fine_map, self.icp_max_correspondence_distance, guess,
                estimation_method=o3d.pipelines.registration.TransformationEstimationForColoredICP(),
                criteria=o3d.pipelines.registration.ICPConvergenceCriteria(max_iteration=self.icp_max_iteration)
            )

            # yaw 결과와 true_yaw 비교
            yaw_result = self.extract_yaw(reg_fine.transformation)# fine 단계에서 나온 yaw 추출
            print(f"Yaw Result: {np.deg2rad(yaw_result)}")
            yaw_diff = np.abs(self.angle_diff(yaw_result, true_yaw))# yaw 차이 계산
            yaw_penalty = 1.0 - np.cos(yaw_diff) # yaw 오차에 대한 패널티 계산

            
            # 이동 후 위치 차이 계산 (위치 오차)
            icp_position = reg_fine.transformation[:2, 3]
            icp_position = np.append(icp_position, 0.0)  # z값을 0.0으로 추가하여 (3,) 형태로 변경
            delta_vec = icp_position - initial_position  # 이동 벡터
            location_penalty_factor = np.linalg.norm(delta_vec) / 2.0  # 위치 차이의 절반만큼 패널티를 부여
            location_penalty_factor = min(location_penalty_factor, 1.0)  # 최대값 1로 제한

            # direction_score 계산 (정방향, 후방)
            dot = np.dot(delta_vec[:2], np.array([np.cos(true_yaw), np.sin(true_yaw)]))  # delta_vec의 x, y 값만 사용
            direction_score = np.clip(dot, -1.0, 1.0)  # 방향성 점수 클리핑

            # 이동 거리 계산
            delta_distance = np.linalg.norm(delta_vec)
            if delta_distance < 0.1:
                direction_score = 0.0  # 너무 작은 이동에 대해서는 방향성 점수 반영 안 함

            # 최종 score 계산
            score = (
                self.rmse_weight * reg_fine.fitness
                - (1 - self.rmse_weight) * reg_fine.inlier_rmse
                #+ self.yaw_penalty_weight * yaw_penalty  # yaw 오차 패널티 반영
               
            )

            # 너무 큰 yaw 오차는 필터링
            if yaw_diff > np.deg2rad(60):  # 60도 이상 차이나면 skip
                if self.debug_mode:
                    print(f"[Yaw Filtering] Skipped Yaw {yaw_deg}° (yaw diff: {np.rad2deg(yaw_diff):.2f}°)")
                continue
            
            # 결과를 best와 비교하여 갱신
            if reg_fine.inlier_rmse is not None:
                print(f"Yaw {yaw_deg}° → fitness: {reg_fine.fitness:.4f}, RMSE: {reg_fine.inlier_rmse:.4f}, Score: {score:.3f}, Correspondences: {len(reg_fine.correspondence_set)}")
            else:
                print(f"Yaw {yaw_deg}° → fitness: {reg_fine.fitness:.4f}, RMSE: None")

            # 점수와 RMSE가 좋은 경우 업데이트
            if score > best_score and reg_fine.inlier_rmse < 0.2:
                best_score = score
                best_T = reg_fine.transformation.copy()
                best_fitness = reg_fine.fitness
                best_rmse = reg_fine.inlier_rmse
                best_yaw_deg = yaw_deg
                best_correspondences = len(reg_fine.correspondence_set)

        if best_yaw_deg is not None:
            print(f"                                                                                ")
            print(f"[Best Yaw: {best_yaw_deg}°] Final Score: {best_score:.4f}, Fitness: {best_fitness:.4f}, RMSE: {best_rmse:.4f}, Correspondences: {best_correspondences:.4f}")
            print(f"                                                                                ")
            print("ICP fitness:", best_fitness) #0~1 사이 값 -> 높을수록 정합 잘 된 것 (0에 가까우면 정합 실패)
            print("ICP inlier RMSE:", best_rmse)
            print("ICP score:", best_score)
            print(f"--------------------------------------------------------------------------------")
        else:
            rospy.logwarn("[ICP]  No valid Best T.")
            return None

        return best_fitness, best_rmse, best_T, best_score


    def remove_low_density_points(self, pcd, nb_neighbors=20, std_ratio=2.0):
        filtered_pcd, ind = pcd.remove_statistical_outlier(
            nb_neighbors=nb_neighbors,
            std_ratio=std_ratio
        )
        return filtered_pcd



    def angle_diff(self, a, b):
        d = a - b
        return np.arctan2(np.sin(d), np.cos(d))
   
    def adjust_final_pose_z(self, transformation):
        transformation = transformation.copy()

        z_min = np.min(np.asarray(self.reference_map.points)[:, 2])
        z_max = np.max(np.asarray(self.reference_map.points)[:, 2])
        z_start = z_min + 1.0

        transformation[2, 3] = z_start
        transformation = enforce_yaw_only(transformation)
        return transformation


    """ICP수행 """   
    # ICP 등록을 통한 위치 추정
    def estimate_pose(self, curr_pcd, cropped_map, last_icp_pose=None):
        print("[estimate_pose]")
        if len(cropped_map.points) == 0:  # cropped_map이 비어 있으면 처리
            rospy.logwarn(f"Cropped map is empty! Skipping ICP.")
            return None, None, None  # 또는 적절한 처리를 해주세요.
        
        
        start_pose = self.get_start_pose() # 첫 번째는 amcl_pose, 두번째부터는 last_icp_pose
        start_pose = enforce_yaw_only(start_pose)  # yaw만 남기기
        start_position = start_pose[:3, 3]

        #법선 벡터 계산     
        #curr_pcd.estimate_normals(search_param=o3d.geometry.KDTreeSearchParamHybrid(radius=0.1, max_nn=30))
        #cropped_map.estimate_normals(search_param=o3d.geometry.KDTreeSearchParamHybrid(radius=0.1, max_nn=30))
        
        if not self.icp_initialized:
            coarse_fitness, coarse_rmse, coarse_transformation = self.run_coarse_icp(curr_pcd, cropped_map, start_pose)
            coarse_transformation = coarse_transformation.copy()
            coarse_transformation = self.adjust_final_pose_z(coarse_transformation)

            #fine
            fine_fitness, fine_rmse, fine_transformation, fine_score = self.run_fine_icp(curr_pcd, cropped_map, coarse_transformation)
            fine_transformation = fine_transformation.copy()
            fine_transformation = self.adjust_final_pose_z(fine_transformation)
            print("[estimate_pose] not self.icp_initialized : coarse -> fine done! ")
        else:
            #fine
            fine_fitness, fine_rmse, fine_transformation, fine_score = self.run_fine_icp(curr_pcd, cropped_map, self.last_icp_pose)
            fine_transformation = fine_transformation.copy()
            fine_transformation = self.adjust_final_pose_z(fine_transformation)          
            print("[estimate_pose] self.icp_initialized : fine done! ")

        # 시각화: 원본 + 맵 + 정합된 포인트
        aligned_pcd = copy.deepcopy(curr_pcd)
        aligned_pcd.transform(fine_transformation)
        cropped_map.paint_uniform_color([0.0, 1.0, 0.0])      # 초록: 맵
        #curr_pcd.paint_uniform_color([1.0, 0.0, 0.0])          # 빨강: 원본
        #aligned_pcd.paint_uniform_color([0.0, 0.0, 1.0])       # 파랑: 정합된 curr_pcd

        o3d.visualization.draw_geometries([cropped_map,  aligned_pcd])


        aligned_pcd = copy.deepcopy(curr_pcd)
        aligned_pcd.transform(fine_transformation)
        
        
        if not self.icp_initialized :
            delta = np.linalg.norm(aligned_pcd.get_center()[:2] - self.amcl_pose_matrix[:2, 3])
            
            if delta > 5.0:  # 너무 멀면 reject
                rospy.logwarn(f"ICP 결과가 AMCL보다 {delta:.2f}m 떨어져 있어 무시됨")
                return None, None, None
        else:
            delta = np.linalg.norm(aligned_pcd.get_center()[:2] - self.last_icp_pose[:2, 3])
            
            if delta > 2.0:  # 너무 멀면 reject
                rospy.logwarn(f"ICP 결과가 AMCL보다 {delta:.2f}m 떨어져 있어 무시됨")
                return None, None, None 

        # transformation이랑 정합정보 같이 리턴 
        return fine_transformation, fine_fitness, fine_rmse, fine_score

    # 위치 추정 결과 발행 (map 기준)
    def publish_pose(self, transformation, fitness, rmse):
        pose_msg = PoseWithCovarianceStamped()
        pose_msg.header.stamp = rospy.Time.now()
        pose_msg.header.frame_id = "map"  

        pose_msg.pose.pose.position.x = transformation[0, 3]
        pose_msg.pose.pose.position.y = transformation[1, 3]
        pose_msg.pose.pose.position.z = transformation[2, 3]


        # Quaternion 변환 (yaw각 추출)
        yaw = np.arctan2(transformation[1, 0], transformation[0, 0])
        quat = tf.transformations.quaternion_from_euler(0, 0, yaw)

        pose_msg.pose.pose.orientation.x = quat[0]
        pose_msg.pose.pose.orientation.y = quat[1]
        pose_msg.pose.pose.orientation.z = quat[2]
        pose_msg.pose.pose.orientation.w = quat[3]

        # 동적으로 공분산 계산
        pos_cov = max(0.01, 1.0 - fitness)  # fitness가 낮으면 공분산 크게
        rot_cov = max(0.01, rmse)           # RMSE가 크면 회전 추정이 불안정하다고 가정

        # covariance는 신뢰도에 따라 임의로 작게 지정
        pose_msg.pose.covariance = [
        pos_cov, 0, 0, 0, 0, 0,
        0, pos_cov, 0, 0, 0, 0,
        0, 0, pos_cov, 0, 0, 0,
        0, 0, 0, 0.1, 0, 0,
        0, 0, 0, 0, 0.1, 0,
        0, 0, 0, 0, 0, rot_cov
        ]

        self.icp_pose_pub.publish(pose_msg)
        rospy.loginfo(f"Published ICP PoseWithCovarianceStamped!")

    def publish_icp_done(self):
        msg = Bool()
        msg.data = True  
        self.icp_initial_pose_pub.publish(msg)
        rospy.loginfo(f" ICP 초기 정합 완료: True")
            
    """main loop"""
    def rgbd_callback(self, rgb_msg, depth_msg):
        try:
            if self.amcl_pose_msg is not None and self.amcl_pose_matrix is not None:
                        
                curr_pcd = self.preprocess_rgbd(rgb_msg, depth_msg)
                if self.debug_mode:
                    print("[Debug] 초기 curr_pcd 중심:", np.asarray(curr_pcd.get_center()))
                
                # z축 보정 
                center_before_rotation = np.asarray(curr_pcd.get_center())
                reference_center_z = self.reference_map.get_center()[2]
                z_shift = reference_center_z - center_before_rotation[2]
                T_z_shift = np.eye(4)
                T_z_shift[2, 3] = z_shift
                curr_pcd.transform(T_z_shift)
                
                # 이후 좌표계 회전 (Open3D → ROS)
                curr_pcd = rotate_to_ros(curr_pcd)

                """추가"""
                # 1) 실제 카메라(pointcloud)의 평균 z
                z_cam = np.asarray(curr_pcd.get_center())[2]
                # 2) AMCL 이 준 z(맵 프레임)
                z_map_cam = self.amcl_pose_matrix[2, 3]
                # 3) 둘의 차이만큼 올려서 맞춰줌
                delta_z = z_map_cam - z_cam
                Tz = np.eye(4)
                Tz[2, 3] = delta_z
                curr_pcd.transform(Tz)

                if self.debug_mode:
                    publish_center(np.asarray(curr_pcd.get_center()),self.debug_curr_point_pub) 
                    print("[Debug] 좌표계 회전 보정 이후 curr_pcd 중심:", np.asarray(curr_pcd.get_center()))
                
                # crop 
                cropped_map = self.crop_map(curr_pcd)
                print("cropped_map points:", len(cropped_map.points))                        
                cropped_map = self.filter_cropped_map(cropped_map) 
                
                #시각화 함수 호출 -- 
                            
                z_vals = np.asarray(cropped_map.points)[:, 2]
                if self.debug_mode:
                    print(f"Z range in cropped_map: min={z_vals.min():.3f}, max={z_vals.max():.3f}")

                curr_pcd = self.remove_low_density_points(curr_pcd)
                # ICP estimation
                transformation, fitness, rmse, score = self.estimate_pose(curr_pcd, cropped_map, self.last_icp_pose)
                if transformation is None:
                    self.icp_fail_count += 1
                    rospy.logwarn(f"ICP failed {self.icp_fail_count} times.")

                    if self.icp_fail_count >= self.icp_fail_threshold:
                        rospy.logwarn("Triggering fallback ICP using full map.")
                        fallback_map = self.reference_map  # 전체 맵 그대로 사용
                        fallback_map.estimate_normals(search_param=o3d.geometry.KDTreeSearchParamHybrid(radius=0.1, max_nn=30))
                        transformation, fitness, rmse = self.estimate_pose(curr_pcd, fallback_map)
                        
                        if transformation is None:
                            rospy.logwarn("Fallback ICP also failed.")
                            return
                        else:
                            self.icp_fail_count = 0  # 성공 시 초기화
                    else:
                        return
                else:
                    self.icp_fail_count = 0  # 성공 시 초기화

                print("Transformation matrix:")
                print(transformation)


                if self.last_icp_pose is not None:
                    prev_position = self.last_icp_pose[:2, 3]
                else:
                    prev_position = self.amcl_pose_matrix[:2, 3]  # 또는 초기값 사용

                curr_position = transformation[:2, 3]
                delta_distance = np.linalg.norm(curr_position - prev_position)
                print(f"delta_distance: {delta_distance}")
                if not self.icp_initialized:
                    self.max_dist = 7.0  # 초기에는 넉넉하게
                else:
                    self.max_dist = 1.5  # 이후부터는 정밀하게
                
                if score > self.score_threshold and delta_distance < self.max_dist:
                    print(f"score > self.score_threshold : {score > self.score_threshold }")
                    print(f"delta_distance > 0.2 : {delta_distance > 0.2}")
                    print(f"delta_distance < self.max_dist : {delta_distance < self.max_dist}")
                    rospy.loginfo(f"Updating last_icp_pose with transformation: {transformation}")
                    self.last_icp_pose = transformation
                    if not self.icp_initialized :      
                        self.publish_icp_done() 
                        self.icp_initialized = True
                        print("self.icp_initialized == True!")
                    final_transformation = self.T_3d_to_2d @ transformation #2d맵 기준으로 변환 
                    
                    self.publish_pose(final_transformation, fitness, rmse) 
                    self.visualize_curr_icp_result(curr_pcd, cropped_map, final_transformation, )
                    publish_position_estimation(final_transformation,self.debug_estimation_point_pub)
                    print("[ICP SUCCESS]")
                else:
                    print(f"score > self.score_threshold : {score > self.score_threshold }")
                    print(f"delta_distance > 0.2 : {delta_distance > 0.2}")
                    print(f"delta_distance < self.max_dist : {delta_distance < self.max_dist}")
                    print("[ICP FAIL]")
                print(f"################################################################################")
                print(f"###############################callback End!####################################")
                print(f"################################################################################")
            else:
                if not self.amcl_warn_logged:
                    rospy.logwarn("AMCL pose is not yet available.")
                    self.amcl_warn_logged = True
        except Exception as e:
            rospy.logerr(f"RGBD callback error: {e}")
    
    def odom_callback(self, msg):
        vx = msg.twist.twist.linear.x
        vy = msg.twist.twist.linear.y
        self.latest_linear_vel = np.sqrt(vx**2 + vy**2)    

    def amcl_pose_callback(self, msg):
        if self.amcl_pose_msg is None:
            self.amcl_pose_msg = msg  # 메시지 전체 저장

            # 변환: amcl pose를 3D 기준으로 변환
            amcl_matrix_2d = pose_to_matrix(msg)
            amcl_matrix_3d = self.T_2d_to_3d @ amcl_matrix_2d  # 2D->3D로 변환

            amcl_matrix_3d[2, 3] = 1.0  # 카메라 실제 장착 높이로 변경  
            # 3D맵 기준의 위치 저장
            self.amcl_position = amcl_matrix_3d[:3, 3]
            print("AMCL pose in 3D map:", self.amcl_position)
            # 변환된 pose 저장
            self.amcl_pose_matrix = amcl_matrix_3d
            print(f"[초기 위치 확정] AMCL pose (3D맵 기준) 저장됨: {self.amcl_position}")
            
            if self.debug_mode:
                publish_amcl_pose_point(self.debug_amcl_pub,self.amcl_pose_msg)  # 디버깅용
            
            # 콜백 해제
            self.amcl_pose_sub.unregister()
            rospy.loginfo(f"초기 위치 확정 완료 → AMCL 구독 해제됨. ")
        
        else:
            pass 

  
# main
if __name__ == "__main__":
    try:
        rospy.loginfo(f"[icp_localization.py]-icp_localization node")
        node = ICPLocalizer()
        rospy.spin()
        
    except rospy.ROSInterruptException:
        pass
