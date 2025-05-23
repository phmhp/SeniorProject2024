#!/usr/bin/env python3
import rospy
from sensor_msgs.msg import LaserScan, PointCloud2
from costmap_converter.msg import ObstacleArrayMsg, ObstacleMsg

class ObstaclePublisher:
    def __init__(self):
        rospy.init_node('obstacle_publisher', anonymous=True)

        # 퍼블리셔 생성
        self.pub = rospy.Publisher('/move_base/local_costmap/obstacles', ObstacleArrayMsg, queue_size=10)

        # LiDAR & Depth 카메라 구독
        rospy.Subscriber('/scan', LaserScan, self.lidar_callback)
        rospy.Subscriber('/camera/depth/color/points', PointCloud2, self.depth_callback)

        self.obstacles = ObstacleArrayMsg()

    def lidar_callback(self, data):
        # LiDAR 데이터를 기반으로 장애물 메시지 생성
        obstacle = ObstacleMsg()
        obstacle.id = 1  # 고유 ID
        obstacle.polygon.points = []  # 장애물 좌표 추가 (여기서 변환 필요)
        self.obstacles.obstacles.append(obstacle)

        # 퍼블리시
        self.pub.publish(self.obstacles)

    def depth_callback(self, data):
        # Depth 데이터를 기반으로 장애물 메시지 생성 (변환 로직 필요)
        obstacle = ObstacleMsg()
        obstacle.id = 2
        obstacle.polygon.points = []  # 장애물 좌표 추가 (여기서 변환 필요)
        self.obstacles.obstacles.append(obstacle)

        self.pub.publish(self.obstacles)

if __name__ == '__main__':
    try:
        node = ObstaclePublisher()
        rospy.spin()
    except rospy.ROSInterruptException:
        pass

