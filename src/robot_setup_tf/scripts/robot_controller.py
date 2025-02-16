#!/usr/bin/env python3
# teleop 으로 map 생성

import rospy
import serial
import threading
import sys
import math
import tf

from geometry_msgs.msg import Twist
from std_msgs.msg import String
from nav_msgs.msg import Odometry
from tf.broadcaster import TransformBroadcaster
sys.path.append('/home/roverosong/catkin_ws/src')
from robot_setup_tf.settings import Params

prm = Params()

# ROS 토픽 이름
MONITOR_TOPIC = "/robot/monitor"
CMD_TOPIC = "/cmd_vel"  # TurtleBot3 teleop의 기본 토픽 이름
ODOM_TOPIC= "/odom"

# 기본 설정
prm.sPort = '/dev/ttyUSB0'  # 시리얼 포트
prm.sVelo = 57600  # 통신 속도
prm.sID = 1  # 로봇 ID

# Publisher 정의
monitor_pub = rospy.Publisher(MONITOR_TOPIC, String, queue_size=10)
odom_pub = rospy.Publisher(ODOM_TOPIC, Odometry, queue_size=10)
odom_broadcaster = TransformBroadcaster()

# 로봇의 초기 위치와 속도
x = 0.0
y = 0.0
theta = 0.0
vx = 0.0
vth = 0.0
last_time = None

def connect_serial_port():
    """시리얼 포트 연결"""
    try:
        if hasattr(prm, 'trans') and prm.trans:
            prm.trans.close()
            rospy.loginfo("Previous connection closed.")
    except AttributeError:
        pass

    try:
        prm.trans = serial.Serial(prm.sPort, prm.sVelo, timeout=1)
        rospy.loginfo(f"Connected to {prm.sPort} at {prm.sVelo} baud.")
    except serial.SerialException as e:
        rospy.logerr(f"Failed to connect: {e}")
        rospy.signal_shutdown("Serial connection failed.")

prmmap = {
    'pid_robot_monitor': lambda mode: prm.pid_robot_monitor(mode),
    'pid_robot_cmd': lambda mode: prm.pid_robot_cmd(mode),
}

def process_state_data(state, mode):
    """PID 처리"""
    value = prmmap.get(state, lambda mode: None)(mode)
    if value is None:
        raise ValueError(f"Invalid state: {state}")

    if mode == "put":
        for i in range(len(value)):
            prm.RingBF.append(value[i])
            prm.cPut += 1
            prm.cRing += 1
        if prm.cPut >= prm.limitBF:
            prm.cPut = 0
    elif mode == 'sum':
        byChkSend = value & 0xFF
        byCHK = (~byChkSend + 1) & 0xFF
        return byCHK
    elif mode == 'display':
        rospy.loginfo(f"display3: {value}")

def read_sensor_data():
    """센서 데이터를 읽어서 ROS 토픽으로 퍼블리시"""
    global x, y, theta, vx, vth, last_time
    try:
        while not rospy.is_shutdown():
            header = None

            while header is None or len(header)<2 or header[0] != 172 or header[1] != 184:
                header = prm.trans.read(2)  # 시작 바이트 2개 읽기
                print(f"header : {header}")
                rospy.loginfo(f"Searching for header: {header}")

            # 유효한 헤더를 발견하면 나머지 데이터를 읽음
            payload = prm.trans.read(24)
            if len(payload) < 24:
                rospy.logwarn("Incomplete payload received. Skipping.")
                continue

            # 헤더 + payload
            response = list(header) + list(payload)
            rospy.loginfo(f"Raw data received: {response}")

            # 데이터 파싱
            parse_sensor_data(response)

    except Exception as e:
        rospy.logerr(f"Error reading data: {e}")

def parse_sensor_data(response):
    """센서 데이터 파싱"""
    global x, y, theta, vx, vth, last_time
    try:
        # 현재 시간 계산
        current_time = rospy.Time.now()
        if last_time is None:
            last_time = current_time
        dt = (current_time - last_time).to_sec()
        last_time = current_time

        dt = max(0.01, min(0.1, dt))

        data_start = 5  # D0 위치
        x_coord = int.from_bytes(response[data_start:data_start+4], byteorder='little', signed=True)
        y_coord = int.from_bytes(response[data_start+4:data_start+8], byteorder='little', signed=True)
        angle = int.from_bytes(response[data_start+8:data_start+10], byteorder='little', signed=False) / 10.0
        battery = response[data_start+10]
        ultrasonic = [response[data_start+11], response[data_start+12], response[data_start+13], response[data_start+14]]
        status = response[data_start+15]
        linear_velocity = int.from_bytes(response[data_start+16:data_start+18], byteorder='little', signed=True)  * 0.001 # mm/s -> m/s
        angular_velocity = int.from_bytes(response[data_start+18:data_start+20], byteorder='little', signed=True) / 10.0 # 0.1 deg/s -> deg/s
        # 로봇 위치와 방향 업데이트 (각도를 라디안으로 변환)
        vth = math.radians(angular_velocity)  # deg/s -> rad/s
        x += linear_velocity * dt * math.cos(theta)
        y += linear_velocity * dt * math.sin(theta)
        theta += vth * dt

        # Theta 값 범위 정리 (-pi ~ pi)
        theta = math.atan2(math.sin(theta), math.cos(theta))

        # 쿼터니언 생성     
        orientation_quat = tf.transformations.quaternion_from_euler(0, 0, theta)

        # odom 메시지 생성
        odom = Odometry()
        odom.header.stamp = current_time
        odom.header.frame_id = "odom"

        # 위치 설정
        odom.pose.pose.position.x = x * 0.001
        odom.pose.pose.position.y = y * 0.001
        odom.pose.pose.position.z = 0.0
        
        # 쿼터니언 값을 각각 설정
        odom.pose.pose.orientation.x = orientation_quat[0]
        odom.pose.pose.orientation.y = orientation_quat[1]
        odom.pose.pose.orientation.z = orientation_quat[2]
        odom.pose.pose.orientation.w = orientation_quat[3]
        # 속도 설정
        odom.child_frame_id = "base_link"
        odom.twist.twist.linear.x = linear_velocity
        odom.twist.twist.linear.y = 0
        odom.twist.twist.angular.z = vth


        # odom 토픽 발행
        odom_pub.publish(odom)

        # 기존 180도 회전 쿼터니언
        rotation_z_180 = tf.transformations.quaternion_from_euler(0, 0, math.pi)

        adjusted_orientation_quat = tf.transformations.quaternion_multiply(
            orientation_quat,
            rotation_z_180
        )

        # tf 변환 발행
        odom_broadcaster.sendTransform(
            (x, y, 0),
            adjusted_orientation_quat,
            current_time,
            "base_link",
            "odom"
        )

        # LiDAR의 상대적 위치 설정
        laser_offset_x = -0.25
        laser_offset_y = 0.0
        laser_offset_z = 0.09

        # 쿼터니언 생성
        laser_orientation_quat = tf.transformations.quaternion_from_euler(0,0,0)

        # base_link -> laser 변환 발행
        odom_broadcaster.sendTransform(
            (laser_offset_x, laser_offset_y, laser_offset_z),
            laser_orientation_quat,
            current_time,
            "laser",
            "base_link"
        )

        status_bits = {
            "Emergency Switch": bool(status & 0b00000001),
            "Moving": bool(status & 0b00000010),
            "Charger Connected": bool(status & 0b00000100),
            "Front Bumper": bool(status & 0b00001000),
            "Reserved 1": bool(status & 0b00010000),
            "Rear Bumper": bool(status & 0b00100000),
            "Reserved 2": bool(status & 0b01000000),
            "Fully Charged": bool(status & 0b10000000),
        }

        rospy.loginfo("Parsed Data:")
        rospy.loginfo(f"  X Coordinate (mm): {x_coord}")
        rospy.loginfo(f"  Y Coordinate (mm): {y_coord}")
        rospy.loginfo(f"  Angle (°): {angle}")
        rospy.loginfo(f"  Battery (%): {battery}")
        rospy.loginfo(f"  Ultrasonic Sensors (cm): {ultrasonic}")
        rospy.loginfo(f"  Status Bits: {status_bits}")
        rospy.loginfo(f"  Linear Velocity (mm/s): {linear_velocity}")
        rospy.loginfo(f"  Angular Velocity (°/s): {angular_velocity}")

        monitor_msg = String()
        monitor_msg.data = (f"X: {x_coord}, Y: {y_coord}, Angle: {angle}°, "
                            f"Battery: {battery}%, Ultrasonic: {ultrasonic}, "
                            f"Linear Velocity: {linear_velocity} mm/s, "
                            f"Angular Velocity: {angular_velocity}°/s")
        monitor_pub.publish(monitor_msg)

    except Exception as e:
        rospy.logerr(f"Error parsing sensor data: {e}")

        
def cmd_vel_callback(msg):
    """/cmd_vel 토픽을 통해 속도 명령 수신"""
    linear_x = int(msg.linear.x * 1000)  # mm/s로 변환
    angular_z = int(msg.angular.z * 100)  # 0.1deg/s로 변환

    rospy.loginfo(f"Received cmd_vel: linear_x={linear_x}, angular_z={angular_z}")

    # 데이터 변환
    prm.rc_d1, prm.rc_d2 = linear_x & 0xFF, (linear_x >> 8) & 0xFF
    prm.rc_d3, prm.rc_d4 = angular_z & 0xFF, (angular_z >> 8) & 0xFF

    # 항상 NORMAL 모드
    prm.rc_d0 = 1

    rospy.loginfo(f"Control Mode (D0): {prm.rc_d0}, rc_d1={prm.rc_d1}, rc_d2={prm.rc_d2}, rc_d3={prm.rc_d3}, rc_d4={prm.rc_d4}")

    prm.state = 'pid_robot_cmd'

    process_state_data('pid_robot_cmd', 'put')
    send_message(prm.state)


def send_message(state):
    """로봇으로 명령 전송"""
    try:
        byCHK = process_state_data(state, 'sum')
        message = prm.RingBF + [byCHK]
        for number in message:
            prm.trans.write(number.to_bytes(1, 'big'))

        rospy.loginfo(f"Message sent: {message}")

        prm.RingBF.clear()
    except Exception as e:
        rospy.logerr(f"Error sending message: {e}")

def serial_read_thread():
    """명령 재전송 및 센서 데이터 읽기"""
    while not rospy.is_shutdown():
        process_state_data('pid_robot_cmd', 'put')
        send_message('pid_robot_cmd')
        read_sensor_data()
        rospy.sleep(0.1)  # 1초에 20회 반복

if __name__ == "__main__":
    rospy.init_node("robot_controller")

    connect_serial_port()

    rospy.Subscriber(CMD_TOPIC, Twist, cmd_vel_callback)

    threading.Thread(target=serial_read_thread, daemon=True).start()

    rospy.spin()
