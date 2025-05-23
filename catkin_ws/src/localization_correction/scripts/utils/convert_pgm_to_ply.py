import numpy as np
import cv2
import yaml
import open3d as o3d
import os
import rospkg
# YAML 경로 설정
yaml_path = "/home/hyemin/catkin_ws/src/localization_correction/map/myungsin.yaml"
# rospkg를 사용하여 패키지 경로를 찾음
rospack = rospkg.RosPack()
package_path = rospack.get_path('localization_correction')  # 패키지 이름을 지정
yaml_path = "myungsin.yaml"
yaml_full_path = os.path.join(package_path, 'map', yaml_path)  # 'map' 폴더 내 파일을 참조
# 1. yaml 로드
with open(yaml_full_path, 'r') as f:
    info = yaml.safe_load(f)

resolution = info["resolution"]
origin = np.array(info["origin"][:2])
pgm_rel_path = info["image"]
pgm_path = os.path.join(os.path.dirname(yaml_full_path), pgm_rel_path)

# 2. PGM 이미지 로드
img = cv2.imread(pgm_path, cv2.IMREAD_UNCHANGED)
if img is None:
    raise FileNotFoundError(f"PGM 파일을 불러올 수 없습니다: {pgm_path}")

points = []

# 3. occupancy 셀 좌표를 실제 월드 좌표계로 변환
for y in range(img.shape[0]):
    for x in range(img.shape[1]):
        if img[y, x] < 128:  # 점유 셀로 판단 (검은색 계열)
            wx = x * resolution + origin[0]
            wy = (img.shape[0] - y) * resolution + origin[1]
            points.append([wx, wy, 0.0])  # Z=0으로 평면 투영

# 4. PointCloud 객체로 변환 및 저장
pcd = o3d.geometry.PointCloud()
pcd.points = o3d.utility.Vector3dVector(np.array(points))


save_path = "myungsin_2dmap_from_pgm.ply"
save_full_path = os.path.join(package_path, 'map', save_path)  # 'map' 폴더 내 파일을 참조
o3d.io.write_point_cloud(save_full_path, pcd)

print(f"저장 완료: {save_path}, 총 점 수: {len(points)}")

