import numpy as np
import os

# 저장 경로
save_dir = "catkin_ws/src/localization_correction/cfg"
os.makedirs(save_dir, exist_ok=True)

# 1. myungsin_T_3d_to_2d.npy
myungsin_array = np.array([
    [0.9924637127756059, 0.1225388869855595, 0.0, -18.528],
    [-0.1225388869855595, 0.9924637127756059, -0.0, 2.084],
    [-0.0, 0.0, 1.0, -1.55],
    [0.0, 0.0, 0.0, 1.0]
])
#np.save(os.path.join(save_dir, "myungsin_T_3d_to_2d.npy"), myungsin_array)
np.save(os.path.join("myungsin_T_3d_to_2d.npy"), myungsin_array)

# 2. renaissance_T_3d_to_2d.npy
renaissance_array = np.array([
    [0.065, -1.151, -0.017, 8.46],
    [1.151, 0.066, -0.045, 8.742],
    [0.046, -0.015, 1.152, -1.949],
    [0.0, 0.0, 0.0, 1.0]
])
#np.save(os.path.join(save_dir, "renaissance_T_3d_to_2d.npy"), renaissance_array)
np.save(os.path.join("renaissance_T_3d_to_2d.npy"), renaissance_array)

print(".npy 파일들이 생성되었습니다.")

