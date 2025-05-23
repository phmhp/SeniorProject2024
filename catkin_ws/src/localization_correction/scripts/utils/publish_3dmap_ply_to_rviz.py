import open3d as o3d
import numpy as np
import rospy
import rospkg
import os
import sensor_msgs.point_cloud2 as pc2
from std_msgs.msg import Header
from sensor_msgs.msg import PointCloud2


def transform_and_publish_pcd(pcd_path, transformation, topic_name="/3d_map"):
    rospy.init_node("pcd_publisher", anonymous=True)
    pub = rospy.Publisher(topic_name, PointCloud2, queue_size=1)
    rate = rospy.Rate(1)

   # rospkg를 사용하여 패키지 경로를 찾음
    rospack = rospkg.RosPack()
    package_path = rospack.get_path('localization_correction')  # 패키지 이름을 지정
    pcd_full_path = os.path.join(package_path, 'map', pcd_path)  # 'map' 폴더 내 파일을 참조

    pcd = o3d.io.read_point_cloud(pcd_full_path)
    #pcd.transform(transformation)

    while not rospy.is_shutdown():
        header = Header()
        header.stamp = rospy.Time.now()
        header.frame_id = "map"  # 2D map과 동일한 프레임으로 설정 

        points = np.asarray(pcd.points)
        cloud_msg = pc2.create_cloud_xyz32(header, points)

        pub.publish(cloud_msg)
        rate.sleep()

if __name__=="__main__":
    pcd_path = "myungsin_final.ply"

    T = np.eye(4)  # 4x4 단위 행렬

    #T = np.load("/home/hyemin/catkin_ws/src/localization_correction/config/T_3d_to_2d.npy")
    topic_name = "/3d_map"
    transform_and_publish_pcd(pcd_path, T, topic_name)
