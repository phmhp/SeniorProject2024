#!/usr/bin/env python3
import rospy
import re
import firebase_admin
from firebase_admin import credentials, db
from std_msgs.msg import String
from robot_setup_tf.msg import RobotState

# Firebase 연결 설정
cred = credentials.Certificate("/home/roverosong/catkin_ws/src/robot_setup_tf/rovero-9059c-firebase-adminsdk-rrzbn-9bd7f92153.json")
firebase_admin.initialize_app(cred, {
    'databaseURL': 'https://rovero-9059c-default-rtdb.asia-southeast1.firebasedatabase.app/'
})

battery_ref = db.reference('robot_battery')  # Firebase 데이터베이스 경로

def upload_to_firebase(state_msg):
    """Firebase에 배터리 데이터 업로드"""
    try:
        data = {
            "battery": state_msg.battery,
            "rear_bumper": state_msg.rear_bumper,
            "ultrasonic": state_msg.ultrasonic,
            "timestamp": rospy.Time.now().to_sec()
        }
        battery_ref.set(data)
        rospy.loginfo(f"Firebase 업데이트: {data}")
    except Exception as e:
        rospy.logerr(f"Firebase 업로드 오류: {e}")

def state_callback(msg):
    """/robot/state 토픽에서 데이터를 받아 Firebase로 전송"""
    rospy.loginfo(f"수신된 로봇 상태: 배터리={msg.battery}, 후방 범퍼={msg.rear_bumper}")
    upload_to_firebase(msg)

def main():
    rospy.init_node('battery_firebase_uploader', anonymous=True)
    rospy.Subscriber('/robot/state', RobotState, state_callback)
    rospy.loginfo("Firebase 업로드 노드 실행 중...")
    rospy.spin()

if __name__ == "__main__":
    main()

