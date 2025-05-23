import numpy as np
import open3d as o3d

pcd = o3d.io.read_point_cloud("/home/hyemin/catkin_ws/src/localization_correction/map/myungsin_final_reverse.ply")

T_align = np.array([
    [ 0.999940276146,  0.000189510567,  0.010928814292, -0.000196456909],
    [-0.000189510567,  0.99939827553,  -0.034669443965, -0.000621914864],
    [-0.010928814292,  0.034669443965,  0.999339103699,  0.249706553631],
    [ 0.0,             0.0,             0.0,             1.0]
])
pcd.transform(T_align)
o3d.visualization.draw_geometries([pcd])
o3d.io.write_point_cloud("/home/hyemin/catkin_ws/src/localization_correction/map/myungsin_aligned.ply", pcd)

