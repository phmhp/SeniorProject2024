import numpy as np
from tf.transformations import euler_from_matrix, euler_matrix

T = np.load("/home/hyemin/catkin_ws/src/localization_correction/cfg/myungsin_T_3d_to_2d.npy")

# 기존 회전 행렬에서 yaw만 추출
R = T[:3, :3]
_, _, yaw = euler_from_matrix(R, axes='sxyz')

# yaw만으로 다시 회전행렬 생성
R_yaw_only = euler_matrix(0, 0, yaw, axes='sxyz')[:3, :3]

# 새 변환행렬 구성
T_fixed = np.eye(4)
T_fixed[:3, :3] = R_yaw_only
T_fixed[:3, 3] = T[:3, 3]  # 위치는 그대로 사용

# 저장
np.save("/home/hyemin/catkin_ws/src/localization_correction/cfg/myungsin_T_3d_to_2d_fixed.npy", T_fixed)
print("Yaw-only transformation matrix saved.")
