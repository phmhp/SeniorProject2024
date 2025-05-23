#!/usr/bin/env python3

import rospy
import tf
import math
from nav_msgs.msg import Odometry

# 전역 변수 (로봇의 위치 및 자세)
x, y, theta = 0.0, 0.0, 0.0
linear_velocity, angular_velocity = 0.0, 0.0

# odom 데이터 콜백 함수 (센서 데이터를 받아 위치 업데이트)
def odom_callback(msg):
    global x, y, theta, linear_velocity, angular_velocity
    
    # 센서에서 받은 실제 위치 데이터 업데이트
    x = msg.pose.pose.position.x
    y = msg.pose.pose.position.y

    # 오일러 각도로 변환 (쿼터니언 → Yaw 값)
    orientation_q = msg.pose.pose.orientation
    _, _, theta = tf.transformations.euler_from_quaternion([
        orientation_q.x, orientation_q.y, orientation_q.z, orientation_q.w
    ])

    # 속도 업데이트
    linear_velocity = msg.twist.twist.linear.x
    angular_velocity = msg.twist.twist.angular.z

def main():
    global x, y, theta

    # ROS 노드 초기화
    rospy.init_node('robot_tf', anonymous=True)

    # TF Broadcaster 초기화
    odom_broadcaster = tf.TransformBroadcaster()

    # Odometry 데이터 구독 (센서 기반 데이터 사용)
    rospy.Subscriber("/odom", Odometry, odom_callback)

    rospy.loginfo("TF Broadcaster started.")

    # 루프 실행
    rate = rospy.Rate(50)  # 50Hz 업데이트
    while not rospy.is_shutdown():
        current_time = rospy.Time.now()  # 최신 시간으로 동기화

        # 쿼터니언 변환 (Yaw 값 → Quaternion)
        orientation_quat = tf.transformations.quaternion_from_euler(0, 0, theta)

        # **map → odom 변환**
        map_quat = tf.transformations.quaternion_from_euler(0, 0, 0)
        odom_broadcaster.sendTransform(
            (0.0, 0.0, 0.0),
            map_quat,
            current_time,
            "odom",
            "map"
        )

        # **odom → base_link 변환 (실제 센서 데이터 기반)**
        odom_broadcaster.sendTransform(
            (x, y, 0),
            orientation_quat,
            rospy.Time(0),
            "base_link",
            "odom"
        )

        # **base_link → laser 변환 (정적)**
        laser_offset = (0.25, 0.0, 0.09)
        laser_quat = tf.transformations.quaternion_from_euler(0, 0, math.pi)
        odom_broadcaster.sendTransform(
            laser_offset,
            laser_quat,
            current_time,
            "laser",
            "base_link"
        )

        # **base_link → camera_link 변환 (정적)**
        depth_offset = (0.26, 0.0, 0.1)
        depth_quat = tf.transformations.quaternion_from_euler(-math.radians(45), 0, 0)
        odom_broadcaster.sendTransform(
            depth_offset,
            depth_quat,
            current_time,
            "camera_link",
            "base_link"
        )

        # **base_link → base_footprint 변환 (정적)**
        odom_broadcaster.sendTransform(
            (0, 0, 0),
            tf.transformations.quaternion_from_euler(0, 0, 0),
            current_time,
            "base_footprint",
            "base_link"
        )

        # ✅ 기존 `robot_navigate.py`처럼 실제 센서 데이터 기반 터미널 출력
        rospy.loginfo(f"""
        ==============================
        Odom Data (From Sensor):
          X: {x:.3f} m
          Y: {y:.3f} m
          Theta: {math.degrees(theta):.2f}°
          Linear Velocity: {linear_velocity:.3f} m/s
          Angular Velocity: {angular_velocity:.3f} rad/s
        ------------------------------
        Transform Published:
          [map] → [odom]
          [odom] → [base_link]
          [base_link] → [laser]
          [base_link] → [camera_link]
          [base_link] → [base_footprint]
        ==============================
        """)

        rate.sleep()

if __name__ == '__main__':
    try:
        main()
    except rospy.ROSInterruptException:
        pass

