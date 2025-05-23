import numpy as np
import tf

#roll, pitch 제거 (카메라가 지면에 대해 기울어진 결과에 맵 평면에 평행하도록 수정)
def enforce_yaw_only(T):
    yaw = np.arctan2(T[1, 0], T[0, 0])
    R_yaw_only = tf.transformations.euler_matrix(0, 0, yaw)[:3, :3]
    T_new = np.eye(4)
    T_new[:3, :3] = R_yaw_only
    T_new[:3, 3] = T[:3, 3]
    return T_new

# 좌표축 방향 변경 
def rotate_to_ros(curr_pcd):
    # Open3D 좌표계가 Z-forward인데, RViz map은 Z-up임
    R_z = curr_pcd.get_rotation_matrix_from_xyz((0, 0, -np.pi/2))  # Z축 기준으로 -90도 회전
    R_x = curr_pcd.get_rotation_matrix_from_xyz((-np.pi/2, 0, 0))  # X축 기준으로 -90도 회전

    # 두 회전 행렬을 곱해서 최종 회전 행렬을 구함
    R_total = np.dot(R_z, R_x)
    #print("rotate_to_ros의 회전 행렬:\n", R_total)

    # 포인트 클라우드 회전
    curr_pcd.rotate(R_total, center=(0, 0, 0))  # Open3D -> ROS 좌표계로 변환   

    # 회전된 포인트 클라우드를 반환
    return curr_pcd

# pose 메시지에서 행렬 생성 
def pose_to_matrix(pose_msg):
    # 위치
    trans = [
        pose_msg.pose.pose.position.x,
        pose_msg.pose.pose.position.y,
        pose_msg.pose.pose.position.z        ]
    
    # 자세 (쿼터니언 → 회전행렬)
    quat = [
        pose_msg.pose.pose.orientation.x,
        pose_msg.pose.pose.orientation.y,
        pose_msg.pose.pose.orientation.z,
        pose_msg.pose.pose.orientation.w        ]
    
    # 회전 행렬
    mat = tf.transformations.quaternion_matrix(quat)  # 4x4 회전행렬 포함
    mat[:3, 3] = trans  # 위치 넣기

    return mat  # 최종 4x4 행렬
