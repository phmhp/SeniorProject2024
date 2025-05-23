import numpy as np

# 주어진 transformation matrix
T = np.array([
    [0.980,  0.121,  0.004, -18.528],
    [-0.121, 0.976,  0.097,   2.084],
    [0.008, -0.097,  0.983,  -1.550],
    [0.000,  0.000,  0.000,   1.000]
])

# 저장
np.save('/home/hyemin/catkin_ws/src/localization_correction/cfg/myungsin_T_3d_to_2d.npy', T)

# 확인용 출력
print("Transformation matrix saved to 'T_icp_result.npy'")

