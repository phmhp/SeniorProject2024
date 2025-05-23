import rospy
import open3d as o3d
import numpy as np    
import struct  
import rospkg
import os
import sensor_msgs.point_cloud2 as pc2
from sensor_msgs.msg import PointCloud2
from std_msgs.msg import Header

# Open3D 포인트 클라우드를 ROS 메시지로 변환 (RGB 포함)
def create_point_cloud_msg_rgb(pcd):
    header = Header()
    header.stamp = rospy.Time.now()
    header.frame_id = "map"  # rviz에서 보는 프레임 이름

    points = np.asarray(pcd.points)
    colors = np.asarray(pcd.colors)

    # RGB 값을 0~255로 변환
    colors = (colors * 255).astype(np.uint8)

    # xyzrgb 포인트 리스트 생성
    cloud_data = []
    for i in range(len(points)):
        x, y, z = points[i]
        r, g, b = colors[i]
        rgb = struct.unpack('I', struct.pack('BBBB', b, g, r, 0))[0]
        cloud_data.append([x, y, z, rgb])

    return pc2.create_cloud(
        header,
        fields=[
            pc2.PointField('x', 0, pc2.PointField.FLOAT32, 1),
            pc2.PointField('y', 4, pc2.PointField.FLOAT32, 1),
            pc2.PointField('z', 8, pc2.PointField.FLOAT32, 1),
            pc2.PointField('rgb', 12, pc2.PointField.UINT32, 1),
        ],
        points=cloud_data
    )

def publish_point_cloud(pub, pcd):
    pc_msg = create_point_cloud_msg_rgb(pcd)
    pub.publish(pc_msg)
    rospy.loginfo("pub.publish(pc_msg)")

def point_cloud_publisher(pcd):
    rospy.init_node("point_cloud_publisher")
    pc_pub = rospy.Publisher("/point_cloud_topic", PointCloud2, queue_size=1)
    rate = rospy.Rate(1)  # 1Hz

    while not rospy.is_shutdown():
        publish_point_cloud(pc_pub, pcd)
        rate.sleep()

if __name__ == "__main__":

    rospack = rospkg.RosPack()
    package_path = rospack.get_path('localization_correction')  # 패키지 이름을 지정
    input_path = "renaissance_0416_3_good.ply"
    input_full_path = os.path.join(package_path, 'map', input_path)  

    pcd = o3d.io.read_point_cloud(input_full_path)

    # RGB 정보가 없으면 에러 표시
    if not pcd.has_colors():
        print("경고: 포인트 클라우드에 색상 정보가 없습니다.")
    else:
        print("색상 정보 포함됨. RViz에서 RGB 표시 가능.")

    # 다운샘플링 제거
    voxel_size = 0.1
    pcd = pcd.voxel_down_sample(voxel_size)

    point_cloud_publisher(pcd)

