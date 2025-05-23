import rospy
import open3d as o3d
import numpy as np
import rospkg
import os
import sensor_msgs.msg
import sensor_msgs.point_cloud2 as pc2
from visualization_msgs.msg import Marker
from sensor_msgs.msg import PointCloud2
from geometry_msgs.msg import Point
from std_msgs.msg import Header

# Open3D 포인트 클라우드를 ROS 메시지로 변환
def create_point_cloud_msg(pcd):
    header = Header()
    header.stamp = rospy.Time.now()
    header.frame_id = "map"  # 프레임 설정
    pc_data = pc2.create_cloud_xyz32(header, np.asarray(pcd.points).tolist())
    return pc_data

# 포인트 클라우드 발행
def publish_point_cloud(pub, pcd):
    pc_data = create_point_cloud_msg(pcd)  # Open3D 포인트 클라우드를 ROS 메시지로 변환
    pub.publish(pc_data)
    
# 평면을 맞추는 함수
def align_and_extract_plane(pcd, package_path):
    # 1. 평면 추출 (RANSAC)
    plane_model, inliers = pcd.segment_plane(distance_threshold=0.01, ransac_n=5, num_iterations=1000)
    print(f"[INFO] 평면 추출 완료 - 평면 포인트 수: {len(inliers)}개")

    # 2. 평면 법선 벡터 기준 회전
    normal = plane_model[:3]
    up = np.array([0, 0, 1])
    rotation_axis = np.cross(normal, up)
    rotation_angle = np.arccos(np.dot(normal, up))
    rotation_matrix = o3d.geometry.get_rotation_matrix_from_axis_angle(rotation_axis * rotation_angle)

    pcd.rotate(rotation_matrix, center=(0, 0, 0))
    print("[INFO] 평면을 수평(z축)과 맞추는 회전 적용 완료")

    # 3. 평면에 속하는 점, 아닌 점 분리
    plane_pcd = pcd.select_by_index(inliers)
    non_plane_pcd = pcd.select_by_index(inliers, invert=True)

    # 4. 각각 저장
    output_dir = os.path.join(package_path, 'map')
    if not os.path.exists(output_dir):
        os.makedirs(output_dir)

    plane_path = os.path.join(output_dir, "renaissance_0416_3_good_plane_only.ply")
    non_plane_path = os.path.join(output_dir, "renaissance_0416_3_good_without_plane.ply")
    rotated_full_path = os.path.join(output_dir, "renaissance_0416_3_good_rotated_full.ply")

    o3d.io.write_point_cloud(plane_path, plane_pcd)
    o3d.io.write_point_cloud(non_plane_path, non_plane_pcd)
    o3d.io.write_point_cloud(rotated_full_path, pcd)

    print(f"[INFO] 평면만 저장 완료 → {plane_path}")
    print(f"[INFO] 평면 제외 버전 저장 완료 → {non_plane_path}")
    print(f"[INFO] 전체 회전 완료본 저장 완료 → {rotated_full_path}")


    # Statistical Outlier Removal 적용
    pcd_clean, ind = pcd.remove_statistical_outlier(nb_neighbors=20, std_ratio=2.0)

    denoised_full_path = os.path.join(output_dir, "renaissance_0416_3_good_denoised.ply")
    # 결과 저장
    o3d.io.write_point_cloud(denoised_full_path, pcd_clean)
    return pcd_clean  

# 2. 수동 회전
def rotate_pcd(pcd, angle_x_deg=0, angle_y_deg=0, angle_z_deg=0):
    angle_x = np.deg2rad(angle_x_deg)
    angle_y = np.deg2rad(angle_y_deg)
    angle_z = np.deg2rad(angle_z_deg)

    R = pcd.get_rotation_matrix_from_xyz((angle_x, angle_y, angle_z))
    pcd.rotate(R, center=(0, 0, 0))
    return pcd



if __name__ == "__main__":
    rospack = rospkg.RosPack()
    package_path = rospack.get_path('localization_correction')  # 패키지 이름을 지정
    input_path = "myungsin_final.ply"
    input_full_path = os.path.join(package_path, 'map', input_path)  # 'map' 폴더 내 파일을 참조
    
    pcd = o3d.io.read_point_cloud(input_full_path)  # Open3D로 포인트 클라우드 불러오기
    
    
    #수동 회전 
    """**빨간색 X축**: `camera_link` 좌표계에서 **X축**은 카메라의 **전방**
       **초록색 Y축**: `camera_link` 좌표계에서 **Y축**은 카메라의 **좌측**
       **파란색 Z축**: `camera_link` 좌표계에서 **Z축**은 카메라의 **상방**  """
    pcd = rotate_pcd(pcd, angle_x_deg=180, angle_y_deg=0, angle_z_deg=0)


    # 평면 정렬 + 분리 + 저장
    #pcd_aligned = align_and_extract_plane(pcd, package_path)

    # 포인트 클라우드 발행
    pcd_path = "renaissance_0416_3_good_final_reverse.ply"
    pcd_full_path = os.path.join(package_path, 'map', pcd_path)  # 'map' 폴더 내 파일을 참조
    
    # 저장 
    o3d.io.write_point_cloud(pcd_full_path, pcd)
    
    # 이후 rviz에서 토픽 선택해서 시각화 수행 
    print("저장완료")

