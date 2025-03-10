#!/usr/bin/env python3
import rospy
import re
import firebase_admin
from firebase_admin import credentials, db
from std_msgs.msg import String

# Firebase 연결 설정
cred = credentials.Certificate("/home/roverosong/catkin_ws/src/robot_setup_tf/rovero-9059c-firebase-adminsdk-rrzbn-9bd7f92153.json")
firebase_admin.initialize_app(cred, {
    'databaseURL': 'https://rovero-9059c-default-rtdb.asia-southeast1.firebasedatabase.app/'
})

battery_ref = db.reference('robot_battery')  # Firebase 경로 설정

# ROS 메시지 콜백 함수
def monitor_callback(msg):
    data = msg.data
    battery_match = re.search(r"Battery:\s*(\d+)%", data)

    if battery_match:
        battery_level = int(battery_match.group(1))
        rospy.loginfo(f"배터리 상태 업데이트: {battery_level}%")
        
        # Firebase에 데이터 업로드
        battery_ref.set({'battery': battery_level})

# ROS 노드 실행
rospy.init_node('battery_firebase_uploader', anonymous=True)
rospy.Subscriber('/robot/monitor', String, monitor_callback)
rospy.spin()

