#!/usr/bin/env python3
import sys
import rospy
import serial
import threading
import math
import tf
import tf2_ros
from geometry_msgs.msg import Twist,Point32
from std_msgs.msg import String
from nav_msgs.msg import Odometry
from tf.broadcaster import TransformBroadcaster
from robot_setup_tf.msg import RobotState, RobotCommand
from robot_setup_tf.settings import Params

sys.path.append('/home/roverosong/catkin_ws/src')
sys.path.append('/home/roverosong/catkin_ws/devel/lib/python3/dist-packages')

prm = Params()

# ROS 토픽 정의
MONITOR_TOPIC = "/robot/monitor"
CMD_TOPIC_VEL = "/cmd_vel"  # TurtleBot3 호환
CMD_TOPIC_CMD = "/robot/cmd"   # TurtleBot3 teleop의 기본 토픽 이름
ODOM_TOPIC= "/odom"
STATE_TOPIC = "/robot/state"

# 기본 설정
prm.sPort = '/dev/robotInfo'  # 시리얼 포트
prm.sVelo = 57600  # 통신 속도
prm.sID = 1  # 로봇 ID

# Publisher 정의
monitor_pub = rospy.Publisher(MONITOR_TOPIC, String, queue_size=10)
state_pub = rospy.Publisher(STATE_TOPIC, RobotState, queue_size=10)
odom_pub = rospy.Publisher(ODOM_TOPIC, Odometry, queue_size=10)

odom_broadcaster = tf.TransformBroadcaster()

# 로봇의 초기 위치와 속도
x, y, theta = 0.0, 0.0, 0.0
vx, vth = 0.0, 0.0
last_time = None

position_lock = threading.Lock()

def connect_serial_port():
    """시리얼 포트 연결"""
    try:
        if hasattr(prm, 'trans') and prm.trans:
            prm.trans.close()
            rospy.loginfo("Previous connection closed.")
    except AttributeError:
        pass

    try:
        prm.trans = serial.Serial(prm.sPort, prm.sVelo, timeout=3)
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

def read_sensor_data(tf_buffer):
    """센서 데이터를 읽어서 ROS 토픽으로 퍼블리시"""
    global x, y, theta, vx, vth, last_time
    rate = rospy.Rate(10)  # 10Hz로 실행
    try:
        while not rospy.is_shutdown():
            header = None

            while header is None or len(header) < 2 or header[0] != 172 or header[1] != 184:
                header = prm.trans.read(2)  # 시작 바이트 2개 읽기
                rospy.loginfo(f"Searching for header: {header}")

            # 유효한 헤더를 발견하면 나머지 데이터를 읽음
            payload = prm.trans.read(24)
            if len(payload) < 24:
                rospy.logwarn("Incomplete payload received. Skipping.")
                continue

            # 헤더 + payload
            response = list(header) + list(payload)
            rospy.loginfo(f"Raw data received: {response}")

            # 데이터 파싱 및 TF 변환 처리
            parse_sensor_data(response, tf_buffer)

            rate.sleep()  # 10Hz로 실행 (센서 데이터 처리 주기 유지)

    except Exception as e:
        rospy.logerr(f"Error reading data: {e}")




def get_tf_transform(tf_buffer, target_frame="map", source_frame="base_link"):
    """TF 변환을 가져오는 함수 (tf_buffer를 main()에서 전달받음)"""
    try:
        current_time = rospy.Time.now()
        transform = tf_buffer.lookup_transform(target_frame, source_frame, current_time, rospy.Duration(1.0))
        return transform
    except tf2_ros.LookupException as e:
        rospy.logwarn(f"TF LookupException: {e}")
    except tf2_ros.ExtrapolationException as e:
        rospy.logwarn(f"TF ExtrapolationException: {e}")
    return None


def parse_sensor_data(response,tf_buffer):
    """센서 데이터 파싱"""
    global x, y, theta, vx, vth, last_time

    try:
        current_time = rospy.Time.now()
        if last_time is None:
            last_time = current_time
        dt = (current_time - last_time).to_sec()
        last_time = current_time
        dt = max(0.01, min(0.1, dt))  # dt 범위 제한

        # TF 변환 가져오기 (현재 base_link의 위치를 map 기준으로 변환)
        transform = get_tf_transform(tf_buffer, "map", "base_link")
        if transform:
            rospy.loginfo(f"TF Transform Data: {transform.transform.translation}")


        # 센서 데이터 파싱
        data_start = 5
        x_coord = int.from_bytes(response[data_start:data_start+4], byteorder='little', signed=True)
        y_coord = int.from_bytes(response[data_start+4:data_start+8], byteorder='little', signed=True)
        angle = int.from_bytes(response[data_start+8:data_start+10], byteorder='little', signed=False) / 10.0
        battery = response[data_start+10]
        ultrasonic = [response[data_start+11], response[data_start+12], response[data_start+13], response[data_start+14]]
        status = response[data_start+15]
        linear_velocity = int.from_bytes(response[data_start+16:data_start+18], byteorder='little', signed=True) * 0.001  # mm/s -> m/s
        angular_velocity = int.from_bytes(response[data_start+18:data_start+20], byteorder='little', signed=True) / 10.0  # 0.1 deg/s -> deg/s

        rospy.loginfo(f"Parsed data - x: {x_coord}, y: {y_coord}, angle: {angle}, battery: {battery}, v: {linear_velocity}, w: {angular_velocity}")

        # 로봇 상태 업데이트
        vth = math.radians(angular_velocity)  # deg/s -> rad/s
        with position_lock:
            x += linear_velocity * dt * math.cos(theta)
            y += linear_velocity * dt * math.sin(theta)
            theta += vth * dt
            theta = math.atan2(math.sin(theta), math.cos(theta))  # -pi ~ pi 범위로 유지

        # RobotState 메시지 생성 및 퍼블리시
        state_msg = RobotState()
        state_msg.header.stamp = current_time
        state_msg.header.frame_id = "base_link"
        state_msg.x = x_coord
        state_msg.y = y_coord
        state_msg.theta = angle
        state_msg.battery = battery
        state_msg.ultrasonic = ultrasonic
        state_msg.emergency_switch = bool(status & 0b00000001)
        state_msg.moving = bool(status & 0b00000010)
        state_msg.charger_connected = bool(status & 0b00000100)
        state_msg.front_bumper = bool(status & 0b00001000)
        state_msg.rear_bumper = bool(status & 0b00100000)
        state_msg.fully_charged = bool(status & 0b10000000)
        state_msg.linear_velocity = linear_velocity
        state_msg.angular_velocity = angular_velocity
        state_pub.publish(state_msg)

        # Odometry 메시지 생성 및 퍼블리시
        orientation_quat = tf.transformations.quaternion_from_euler(0, 0, theta)
        odom = Odometry()
        odom.header.stamp = current_time
        odom.header.frame_id = "odom"
        odom.pose.pose.position.x = x * 0.001
        odom.pose.pose.position.y = y * 0.001
        odom.pose.pose.position.z = 0.0
        odom.pose.pose.orientation.x = orientation_quat[0]
        odom.pose.pose.orientation.y = orientation_quat[1]
        odom.pose.pose.orientation.z = orientation_quat[2]
        odom.pose.pose.orientation.w = orientation_quat[3]
        odom.child_frame_id = "base_link"
        odom.twist.twist.linear.x = linear_velocity
        odom.twist.twist.angular.z = vth
        odom_pub.publish(odom)

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



def tf_broadcast_loop():
    """TF 변환을 일정 주기로 발행하는 쓰레드"""
    rate = rospy.Rate(10)  # TF를 50Hz로 발행
    while not rospy.is_shutdown():
        current_time = rospy.Time.now()

        # 오도메트리 변환
        orientation_quat = tf.transformations.quaternion_from_euler(0, 0, theta)
        odom_broadcaster.sendTransform(
            (x * 0.001, y * 0.001, 0),
            orientation_quat,
            current_time,
            "base_link",
            "odom"
        )

        # LiDAR 변환
        odom_broadcaster.sendTransform(
            (0.25, 0.0, 0.09),
            tf.transformations.quaternion_from_euler(0, 0, math.pi),
            current_time,
            "laser",
            "base_link"
        )
        # 카메라 변환
        
        odom_broadcaster.sendTransform(
        (0.26, 0.0, 0.91),
        tf.transformations.quaternion_from_euler(0,-0.35, 0),
        current_time,
        "camera_link",
        "base_link"
        )
        # camera_color_optical_frame을 camera_color_frame의 자식으로 변경
        odom_broadcaster.sendTransform(
            (0.0, 0.0, 0.0),  # 위치 변화 없음
            tf.transformations.quaternion_from_euler(-1.57,0, -1.57),  # OpenCV → ROS 변환
            current_time,
            "camera_color_optical_frame",
            "camera_color_frame"  # 부모를 camera_color_frame으로 변경
        )
        # odom_broadcaster.sendTransform(
        #     (0.0, 0.0, 0.0),
        #     tf.transformations.quaternion_from_euler(-0.5, 0.5, -0.5),
        #     current_time,
        #     "camera_depth_optical_frame",
        #     "camera_link"
        # )

        # odom_broadcaster.sendTransform(
        #     (0.0, 0.0, 0.0),
        #     tf.transformations.quaternion_from_euler(-0.5, 0.5, -0.5),
        #     current_time,
        #     "camera_color_optical_frame",
        #     "camera_link"
        # )

        rate.sleep()

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

def serial_read_thread(tf_buffer):
    """명령 재전송 및 센서 데이터 읽기"""
    while not rospy.is_shutdown():
        process_state_data('pid_robot_cmd', 'put')
        send_message('pid_robot_cmd')
        read_sensor_data(tf_buffer)
        rospy.sleep(0.1)  # 1초에 20회 반복



if __name__ == "__main__":
    rospy.init_node("robot_controller")
    
    tf_buffer = tf2_ros.Buffer(cache_time=rospy.Duration(5.0))
    tf_listener = tf2_ros.TransformListener(tf_buffer)

    rospy.loginfo("TF Listener initialized.") 
    
    # TF 쓰레드 실행
    threading.Thread(target=tf_broadcast_loop, daemon=True).start()

    rospy.sleep(2)
    connect_serial_port()
    rospy.Subscriber(CMD_TOPIC_VEL, Twist, cmd_vel_callback)
    
    # tf_buffer를 thread에 전달하도록 수정
    threading.Thread(target=serial_read_thread, args=(tf_buffer,), daemon=True).start()

    rospy.spin()
