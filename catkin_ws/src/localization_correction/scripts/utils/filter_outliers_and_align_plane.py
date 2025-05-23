import open3d as o3d
import numpy as np

def load_pcd(path):
    pcd = o3d.io.read_point_cloud(path)
    print("[INFO] 원래 포인트 수:", np.asarray(pcd.points).shape[0])
    return pcd


# 3. 이상치 제거 (x축이 통로 앞뒤)
def filter_and_denoise(filtered_pcd):  # z_min을 0.0으로 설정하여 바닥 이상치 제거
    # 이상치 제거
    cl, ind = filtered_pcd.remove_statistical_outlier(nb_neighbors=50, std_ratio=2.0)
    denoised_pcd = filtered_pcd.select_by_index(ind)

    print("[INFO] 이상치 제거 후 포인트 수:", len(denoised_pcd.points))
    return denoised_pcd

# 4. RANSAC을 통한 평면 추출
def extract_plane(pcd):
    print("평면 추출 시작...")
    plane_model, inliers = pcd.segment_plane(distance_threshold=0.01, ransac_n=5, num_iterations=1000)
    print("평면 추출 완료")
    return plane_model, inliers

# 5. 평면 기준으로 수평 맞추기
def rotate_to_horizontal(pcd, plane_model):
    normal = plane_model[:3]  # 평면의 법선 벡터
    up = np.array([0, 0, 1])  # 기준으로 할 수평 벡터 (z축)

    # 법선 벡터와 수평 벡터를 맞추는 회전 행렬 계산
    rotation_axis = np.cross(normal, up)
    rotation_angle = np.arccos(np.dot(normal, up))
    rotation_matrix = o3d.geometry.get_rotation_matrix_from_axis_angle(rotation_axis * rotation_angle)

    # 회전 적용
    pcd.rotate(rotation_matrix, center=(0, 0, 0))
    return pcd
    
if __name__ == "__main__":
    input_path = "/home/hyemin/catkin_ws/src/localization_correction/map/renaissance_0409_rotated.ply"

    # 1. 포인트 클라우드 불러오기
    pcd = load_pcd(input_path)
    print("pcl 불러오기 완료")
      
    # 3. 이상치 제거
    denoised_pcd = filter_and_denoise(pcd)
    print("이상치 제거 완료")
    
    #4. RANSAC을 통한 평면 추출
    plane_model, inliers = extract_plane(denoised_pcd)
    # 5. 평면 기준으로 수평 맞추기
    denoised_pcd = rotate_to_horizontal(denoised_pcd, plane_model)


    # 7. 시각화 및 저장
    o3d.visualization.draw_geometries([denoised_pcd])
    o3d.io.write_point_cloud("/home/hyemin/catkin_ws/src/localization_correction/map/renaissance_0409_rotated_ransac.ply", denoised_pcd)
    print("저장완료")
