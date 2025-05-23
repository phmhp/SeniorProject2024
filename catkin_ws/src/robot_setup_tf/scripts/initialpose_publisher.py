import tf
import math
import rospy
import threading
import numpy as np
import cv2
from nav_msgs.msg import Odometry
from geometry_msgs.msg import PoseWithCovarianceStamped
from nav_msgs.msg import OccupancyGrid
from collections import deque

#초기 위치 설정
x = 0
y = 0
yaw = 0
map_data = None  
erode_map_data = None
robot_radius = 0.2 # m단위 

"""
0: 빈 공간 (이동 가능)
100: 장애물 (이동 불가)
-1: 스캔되지 않은 영역 (이동 불가)
"""

# 로봇 크기의 반지름을 기준으로 로봇이 이동 가능한 영역만을 남겨서 OccupanchGrid 맵을 반환함 
def erode_map(map_data, robot_radius_in_meters):
    width = map_data.info.width
    height = map_data.info.height
    resolution = map_data.info.resolution

    # 맵 데이터를 2D NumPy 배열로 변환
    raw = np.array(map_data.data, dtype=np.int8).reshape((height, width))

    # 이동 가능 영역(0) → 255, 이동 불가 영역(나머지) → 0
    """ 0 빈 공간 -> 255(흰색-opencv기준)
        100 장애물, -1 미탐색 -> 0 (검은색-opencv기준)"""
    binary_map = np.where(raw == 0, 255, 0).astype(np.uint8)

    # 로봇 반지름만큼 커널 크기 계산 (픽셀 단위)
    kernel_size = int(robot_radius_in_meters / resolution)
    if kernel_size % 2 == 0:
        kernel_size += 1  # 홀수로 맞춤

    kernel = cv2.getStructuringElement(cv2.MORPH_ELLIPSE, (kernel_size, kernel_size))
    eroded = cv2.erode(binary_map, kernel)

    # 다시 0: 자유공간 / 100: 장애물 형태로 변환
    processed_map = np.where(eroded == 255, 0, 100).astype(np.int8)
    
    map_data.data = tuple(processed_map.flatten())  # ROS OccupancyGrid는 튜플 형태 사용
    return map_data    

# BFS 너비 우선 탐색 방식으로 빈 공간 확장하며 면적 계산 후 가장 중심 셀 반환 
def find_largest_free_area(map_data):
    width = map_data.info.width
    height = map_data.info.height
    data = map_data.data

    visited = [[False] * width for _ in range(height)]
    max_area = 0
    best_seed = None

    def in_bounds(i, j):
        return 0 <= i < width and 0 <= j < height

    def bfs(i, j):
        queue = deque()
        queue.append((i, j))
        visited[j][i] = True
        area = 1
        cells = [(i, j)]

        while queue:
            x, y = queue.popleft()
            for dx, dy in [(-1,0), (1,0), (0,-1), (0,1)]:
                nx, ny = x + dx, y + dy
                if in_bounds(nx, ny) and not visited[ny][nx]:
                    index = ny * width + nx
                    if data[index] == 0:
                        visited[ny][nx] = True
                        queue.append((nx, ny))
                        cells.append((nx, ny))
                        area += 1
        return area, cells

    for j in range(height):
        for i in range(width):
            if not visited[j][i] and data[j * width + i] == 0:
                area, cells = bfs(i, j)
                if area > max_area:
                    max_area = area
                    best_seed = cells[len(cells) // 2]  # 중심점
    return best_seed  # (i, j)
    
    
# 실제 월드 좌표로 변환 후 /initialpose 토픽으로 발행 
def publish_initialpose_from_map_index(yaw=0.0, map_data=None):
    pub = rospy.Publisher('/initialpose', PoseWithCovarianceStamped, queue_size=10, latch=True)

    rospy.sleep(1.0)
    
    width = map_data.info.width
    height = map_data.info.height
    resolution = map_data.info.resolution
    origin_x = map_data.info.origin.position.x
    origin_y = map_data.info.origin.position.y

    found = False
    """
    for j in range(height):
        for i in range(width):
            if is_free(i, j, map_data):  # 맵 좌표계에서 검사
                found = True
                break
        if found:
            break
    if found:
    """
    i,j = find_largest_free_area(map_data)
    
    # 퍼블리시 시점에만 실제 좌표로 변환(중심점)
    x = origin_x + (i + 0.5) * resolution
    y = origin_y + (j + 0.5) * resolution
    rospy.loginfo(f"map index ({i}, {j}) → real ({x:.2f}, {y:.2f})")

    # 쿼터니언으로 변환
    q = tf.transformations.quaternion_from_euler(0, 0, yaw)

    pose = PoseWithCovarianceStamped()
    pose.header.stamp = rospy.Time.now()
    pose.header.frame_id = "map"
    pose.pose.pose.position.x = x
    pose.pose.pose.position.y = y
    pose.pose.pose.position.z = 0.0
    pose.pose.pose.orientation.x = q[0]
    pose.pose.pose.orientation.y = q[1]
    pose.pose.pose.orientation.z = q[2]
    pose.pose.pose.orientation.w = q[3]
    pub.publish(pose)
    rospy.loginfo(f"/initialpose published at map index ({i}, {j}) → real ({x:.2f}, {y:.2f})")



def is_free(i, j, map_data):
    width = map_data.info.width
    index = j * width + i
    return map_data.data[index] == 0 #True or False 
    
    
    
def map_callback(msg):
    global map_data, robot_radius, erode_map_data

    map_data = msg  # map_data에 맵 데이터를 저장
    erode_map_data = erode_map(map_data, robot_radius)
    rospy.loginfo("[map_callback] Received map data")
    eroded_map_pub.publish(erode_map_data)
    
    #rospy.loginfo(f"Map data: {map_data.data}")
    #rospy.loginfo(f"Received map data: {map_data.data[:10]}")  # 첫 10개 데이터만 확인

    publish_initialpose_from_map_index(yaw=0,map_data=map_data)
    rospy.loginfo(f"publish_initial_pose_from_map_index(yaw=0,map_data=map_data)")


if __name__ == "__main__":
    rospy.init_node("initialpose_publisher")
    rospy.loginfo("[initialpose_publisher.py]-initialpose_publisher node")
    eroded_map_pub = rospy.Publisher("/eroded_map", OccupancyGrid, queue_size=1, latch=True)
    
    rospy.Subscriber("/map", OccupancyGrid, map_callback)
    while map_data is None and not rospy.is_shutdown():
        rospy.loginfo("Waiting for map data...")
        rospy.sleep(1.0)    
    


    rospy.spin()



