import rospy
import threading
import tf
import os
import rospkg
import math
import numpy as np 
from collections import deque
from collections import defaultdict
from std_msgs.msg import Bool
from nav_msgs.msg import Path
from nav_msgs.msg import Odometry
from geometry_msgs.msg import PoseWithCovarianceStamped, PoseStamped
from nav_msgs.msg import OccupancyGrid
#초기 위치 설정
x = 0
y = 0
yaw = 0

speed = 0.5

clockwise_failed = set()
counter_clockwise_failed = set()

recent_positions = deque(maxlen=20)
rotation_attempts = defaultdict(int)
ROTATION_THRESHOLD = 4



"""tf 발행"""
class TFPublisher:
    def __init__(self):
        self.broadcaster = tf.TransformBroadcaster()  

    # 고정 TF 발행 
    def publish_static_tf(self):
        rate = rospy.Rate(1)
        while not rospy.is_shutdown():
            
            # map -> odom
            self.broadcaster.sendTransform(
                (0.0, 0.0, 0.0),  # 고정 위치
                tf.transformations.quaternion_from_euler(0, 0, 0),
                rospy.Time.now(),
                "odom", "map"
            )
            #rospy.loginfo(f"map -> odom tf published!")
            

            # base_link -> camera_link
            self.broadcaster.sendTransform(
            (0.25, 0.0, 1.0),
            tf.transformations.quaternion_from_euler(0, 0, 0), 
            rospy.Time.now(),
            "camera_link", "base_link"
            )
            #tf.transformations.quaternion_from_euler(-1.57,0, -1.57),  # OpenCV → ROS 변환
            #rospy.loginfo(f"base_link -> camera_link tf published!")
            
            rate.sleep()
   
    def publish_base_link_to_odom(self, pose):
        position = pose.pose.pose.position
        orientation = pose.pose.pose.orientation
        self.broadcaster.sendTransform(
            (position.x, position.y, position.z),
            (orientation.x, orientation.y, orientation.z, orientation.w),
            rospy.Time.now(),
            "base_link", "odom"
        )
        #rospy.loginfo(f"odom->base_link published!") 


         
        

class MapHandler:
    def __init__(self):
        self.map_data = None  
        self.width = None
        self.height = None
        self.resolution = 0.05 #해상도 
        self.origin_x = None
        self.origin_y = None
        self.erode_map_data = None

    # 원본 2D map 
    def map_callback(self, msg):
        self.map_data = msg  # map_data에 맵 데이터를 저장
        self.width = self.map_data.info.width
        self.height = self.map_data.info.height
        self.resolution = self.map_data.info.resolution
        self.origin_x = self.map_data.info.origin.position.x
        self.origin_y = self.map_data.info.origin.position.y
        rospy.loginfo("Received map data")
        #rospy.loginfo(f"Map data: {map_data.data}")
        #rospy.loginfo(f"Received map data: {map_data.data[:10]}")  # 첫 10개 데이터만 확인

    def eroded_map_callback(self,msg):
        self.eroded_map_data = msg
        rospy.loginfo("Received eroded map!")



class PosePublisher:
    def __init__(self, tf_publisher, map_handler, linear_speed=0.1, angular_speed=0.0):
        self.tf_publisher = tf_publisher
        self.map_handler = map_handler
        self.linear_speed = linear_speed
        self.angular_speed = angular_speed
        self.amcl_pub = rospy.Publisher("/amcl_pose", PoseWithCovarianceStamped, queue_size=10) 
        self.odom_pub = rospy.Publisher("/odom", Odometry, queue_size=10)

    def publish_amcl_pose(self, i=0, j=0.0, yaw=0):
        rate = rospy.Rate(1)
                
        pose = PoseWithCovarianceStamped()
        pose.header.stamp = rospy.Time.now()
        pose.header.frame_id = "map"
        
        # x,y 실제 좌표로 변환 (중심점)
        pose.pose.pose.position.x = self.map_handler.origin_x + (i + 0.5) * self.map_handler.resolution
        pose.pose.pose.position.y = self.map_handler.origin_y + (j + 0.5) * self.map_handler.resolution
        pose.pose.pose.position.z = 0.0

        # 쿼터니언으로 변환
        q = tf.transformations.quaternion_from_euler(0, 0, yaw)
        pose.pose.pose.orientation.x = q[0]
        pose.pose.pose.orientation.y = q[1]
        pose.pose.pose.orientation.z = q[2]
        pose.pose.pose.orientation.w = q[3]

        self.amcl_pub.publish(pose)
        rospy.loginfo(f"/amcl_pose published!")

        # /odom 발행 함수 호출
        self.publish_odom(pose)

  
    """/amcl_pose 발행"""         
    def publish_odom(self,pose):
        odom = Odometry()
        odom.header = pose.header
        odom.header.frame_id = "odom"
        odom.child_frame_id = "base_link"
        odom.pose = pose.pose

        odom.twist.twist.linear.x = self.linear_speed  # 전진 속도 0.1 m/s
        odom.twist.twist.linear.y = 0.0
        odom.twist.twist.linear.z = 0.0

        odom.twist.twist.angular.x = 0.0
        odom.twist.twist.angular.y = 0.0
        odom.twist.twist.angular.z = self.angular_speed  # 회전 속도 rad/s
    
        self.odom_pub.publish(odom)
        self.tf_publisher.publish_base_link_to_odom(pose)
    
    def publish_first_amcl_pose(self, msg):
        pub = rospy.Publisher('/amcl_pose', PoseWithCovarianceStamped, queue_size=10,latch=True)
        rate = rospy.Rate(1)
        #msg에서 받아오기 
        x, y = msg.pose.pose.position.x, msg.pose.pose.position.y
        
        # 메시지에서 quaternion을 가져오고 yaw 값 계산
        orientation_q = msg.pose.pose.orientation 
        (roll, pitch, yaw) = tf.transformations.euler_from_quaternion(
            [orientation_q.x, orientation_q.y, orientation_q.z, orientation_q.w]
        )
        rospy.loginfo(f"/initialpose received: x={x:.2f}, y={y:.2f}, yaw(deg)={math.degrees(yaw):.1f}")
        
        pose = PoseWithCovarianceStamped()
        pose.header.stamp = rospy.Time.now()
        pose.header.frame_id = "map"
        pose.pose.pose.position.x = x
        pose.pose.pose.position.y = y
        pose.pose.pose.position.z = 0.0

        # 쿼터니언으로 변환
        q = tf.transformations.quaternion_from_euler(0, 0, yaw)
        pose.pose.pose.orientation.x = q[0]
        pose.pose.pose.orientation.y = q[1]
        pose.pose.pose.orientation.z = q[2]
        pose.pose.pose.orientation.w = q[3]

        pub.publish(pose)
        rospy.loginfo(f"First /amcl_pose published!")
        
        self.publish_odom(pose)

     
class RobotController:
    def __init__(self, map_handler, pose_publisher):
        self.map_handler = map_handler
        self.pose_publisher = pose_publisher
        self.clockwise = True

    def move_robot_in_map(self, x, y, yaw):
        rospy.loginfo(f"[move_robot_in_map]")
 
        i_idx = int((x - self.map_handler.origin_x) / self.map_handler.resolution)
        j_idx = int((y - self.map_handler.origin_y) / self.map_handler.resolution)
        
        yaw = self.snap_yaw_to_discrete(yaw)
        dx = int(round(math.cos(yaw)))
        dy = int(round(math.sin(yaw)))
        
        next_i = i_idx + dx
        next_j = j_idx + dy
        
        key = (round(x, 2), round(y, 2), round(yaw, 2))  # 위치 + 각도 기준 키
        rospy.loginfo(f"key = {key}")
        
            
        if self.is_free(next_i, next_j):
            self.pose_publisher.publish_amcl_pose(next_i, next_j, yaw)
        else:
            rospy.loginfo(f"Blocked at ({next_i},{next_j}) → rotation_attempts = {rotation_attempts[key]}")
            yaw = (yaw + math.pi) % (2 * math.pi)

            yaw = self.rotate_robot(yaw)
            self.pose_publisher.publish_amcl_pose(i_idx, j_idx, yaw)

    def is_free(self, i, j):
        map_data = self.map_handler.eroded_map_data

        if 0 <= i < map_data.info.width and 0 <= j < map_data.info.height:
            index = j * map_data.info.width + i
            return map_data.data[index] == 0
        else:
            return False
        
    def rotate_robot(self, yaw):
        delta = math.pi / 2 if self.clockwise else -math.pi / 2
        return (yaw + delta) % (2 * math.pi)

    # 현재 yaw 값 (라디안)을 0, 90, 180, 270 중 가장 가까운 각도로 정렬
    def snap_yaw_to_discrete(self, yaw):
        angle_deg = round(math.degrees(yaw) / 90.0) * 90
        return math.radians(angle_deg % 360)


class SimulationManager:
    def __init__(self):

        # 속도 설정
        self.linear_speed = 0.1  # m/s
        self.angular_speed = 0.0  # rad/s

        self.tf_publisher = TFPublisher()
        self.map_handler = MapHandler()
        self.pose_publisher = PosePublisher(self.tf_publisher, self.map_handler, self.linear_speed, self.angular_speed)
        self.robot_controller = RobotController(self.map_handler, self.pose_publisher)
        self.icp_done = False
        self.initialpose_msg = None
        self.path_pub = rospy.Publisher('/path', Path, queue_size=10)
        
        package_path = rospkg.RosPack().get_path('localization_correction')
        path_file = os.path.join(package_path, 'cfg', 'path.txt')  

        path_file = os.path.abspath(path_file)

        self.path_points = self.load_path_from_file(path_file)

        self.path_msg = Path()
        self.path_msg.header.frame_id = "map"
        self.create_path_message()


        rospy.sleep(1.0)  # 퍼블리셔 준비 대기
        self.publish_path()

        
    # Subscribers
        rospy.Subscriber("/map", OccupancyGrid, self.map_handler.map_callback)
        rospy.Subscriber("/eroded_map", OccupancyGrid, self.map_handler.eroded_map_callback)
        self.initialpose_sub = rospy.Subscriber("/initialpose", PoseWithCovarianceStamped, self.initialpose_callback)
        rospy.Subscriber("/amcl_pose", PoseWithCovarianceStamped, self.amcl_pose_callback)
        self.icp_done_sub = rospy.Subscriber("/icp_done", Bool, self.icp_done_callback)

        threading.Thread(target=self.tf_publisher.publish_static_tf).start()


    def initialpose_callback(self,msg):
        self.initialpose_msg = msg
        rospy.loginfo("[initialpose_callback] initialpose received.")

        #msg에서 받아오기 
        x, y = msg.pose.pose.position.x, msg.pose.pose.position.y
        quat = msg.pose.pose.orientation
        while not self.icp_done: 
            self.pose_publisher.publish_first_amcl_pose(msg)
            rospy.sleep(1.0)
            rospy.loginfo("self.publish_first_amcl_pose executed")

            if self.icp_done:
                self.initialpose_sub.unregister()
                rospy.loginfo("/initialpose subscriber unregistered")
        
            # icp_done True 시 콜백을 트리거하기 위해 amcl_pose 초기 발행
            self.pose_publisher.publish_first_amcl_pose(msg)   
        
    """/amcl_pose 콜백"""
    def amcl_pose_callback(self, msg):
        global speed
        rospy.loginfo("[amcl_pose_callback] amcl_pose_callback triggered")
        
        if self.icp_done == True :   
            #/amcl_pose에서 받아오기 
            x, y = msg.pose.pose.position.x, msg.pose.pose.position.y
            
            # 메시지에서 quaternion을 가져오고 yaw 값 계산
            orientation_q = msg.pose.pose.orientation  
            (roll, pitch, yaw) = tf.transformations.euler_from_quaternion(
                [orientation_q.x, orientation_q.y, orientation_q.z, orientation_q.w]
            )
            """
            #동적 이동 
            self.robot_controller.move_robot_in_map(x,y,yaw)
            rospy.sleep(speed) #속도 조절 
            """
        

    # 동적 이동 시작 기준 
    def icp_done_callback(self, msg):
        self.icp_done = msg.data  # std_msgs/Bool 라고 가정
        if self.icp_done :
            
            self.icp_done_sub.unregister()
            rospy.loginfo("/icp_done_sub subscriber unregistered")
            
            rospy.loginfo("[icp_done_callback] ICP Done True → follow_path 시작!")
            self.follow_path()

    def load_path_from_file(self, file_path):
        path_points = []
        with open(file_path, 'r') as f:
            for line in f:
                x, y = map(float, line.strip().split())
                path_points.append((x, y))
        return path_points
    
    def create_path_message(self):
        for x, y in self.path_points:
            pose_stamped = PoseStamped()
            pose_stamped.header.frame_id = "map"
            pose_stamped.pose.position.x = x
            pose_stamped.pose.position.y = y
            pose_stamped.pose.orientation.w = 1.0  # 방향은 일단 모두 동일 (회전 없음 가정)
            self.path_msg.poses.append(pose_stamped)

    def publish_path(self):
        self.path_pub.publish(self.path_msg)
        rospy.loginfo("Published path for RViz visualization.")

    def follow_path(self):
        rate = rospy.Rate(0.05)  
        
        for idx, (x, y) in enumerate(self.path_points):
            # map 좌표 (x, y) → 맵 인덱스 (i, j) 변환
            i_idx = int((x - self.map_handler.origin_x) / self.map_handler.resolution)
            j_idx = int((y - self.map_handler.origin_y) / self.map_handler.resolution)

            # yaw는 0으로 가정 (필요하면 경로에 따라 다르게 넣을 수도 있음)
            yaw = 0.0

            self.pose_publisher.publish_amcl_pose(i_idx, j_idx, yaw)

            rospy.loginfo(f"[follow_path] Published AMCL and ODOM at ({x:.2f}, {y:.2f})")
            rate.sleep()


if __name__ == "__main__":
    rospy.init_node("mock_simulation")
    rospy.loginfo("[mock_simulation.py]-mock_simulation node")
    manager = SimulationManager()
    rospy.spin()



