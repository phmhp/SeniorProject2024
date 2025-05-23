#!/usr/bin/env python3
import rospy
import re
from std_msgs.msg import String

# 후방 범퍼 상태 저장 변수
rear_bumper_state = None

# /robot/monitor 콜백 함수
def monitor_callback(msg):
    global rear_bumper_state
    data = msg.data  # 수신된 데이터 (문자열)

    # 정규 표현식으로 데이터 추출
    battery_match = re.search(r"Battery:\s*(\d+)%", data)
    ultrasonic_match = re.search(r"Ultrasonic:\s*\[([\d,\s]+)\]", data)
    rear_bumper_match = re.search(r"Rear Bumper:\s*(\S+)", data)  # 후방 범퍼 값 추출!

    # 값이 존재하는 경우만 추출
    battery = int(battery_match.group(1)) if battery_match else None
    ultrasonic_values = [int(val) for val in ultrasonic_match.group(1).split(",")] if ultrasonic_match else []
    rear_bumper_state = rear_bumper_match.group(1) if rear_bumper_match else "데이터 없음"

    # 후방 범퍼 상태 텍스트 정리
    if rear_bumper_state == "정상":
        bumper_status_text = "정상"
    elif rear_bumper_state == "충돌 감지됨":
        bumper_status_text = "충돌 감지됨"
    else:
        bumper_status_text = "데이터 없음"

    # 데이터 출력
    rospy.loginfo("로봇 센서 데이터 수신 ")
    rospy.loginfo(f"  배터리 잔량(%): {battery}")
    rospy.loginfo(f"  초음파 센서(cm): {ultrasonic_values}")
    rospy.loginfo(f"  후방 범퍼 상태: {bumper_status_text}")

# ROS 노드 초기화
rospy.init_node('robot_monitor_parser', anonymous=True)

# /robot/monitor 토픽 구독
rospy.Subscriber('/robot/monitor', String, monitor_callback)

# 노드 실행
rospy.spin()

