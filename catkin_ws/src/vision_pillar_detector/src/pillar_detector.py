#!/usr/bin/env python3
import cv2
import numpy as np
import rospy
from sensor_msgs.msg import Image
from cv_bridge import CvBridge

# OpenCV ORB 객체 생성
orb = cv2.ORB_create()

# 기둥 이미지 (참조용)
pillar_img = cv2.imread("/home/hyemin/catkin_ws/src/vision_pillar_detector/images/pillar_reference.jpg", cv2.IMREAD_GRAYSCALE)
kp1, des1 = orb.detectAndCompute(pillar_img, None)

# OpenCV-ROS 변환 브리지
bridge = CvBridge()

def detect_pillar(img):
    gray = cv2.cvtColor(img, cv2.COLOR_BGR2GRAY)
    kp2, des2 = orb.detectAndCompute(gray, None)
    
    bf = cv2.BFMatcher(cv2.NORM_HAMMING, crossCheck=True)
    matches = bf.match(des1, des2)
    matches = sorted(matches, key=lambda x: x.distance)
    
    result_img = cv2.drawMatches(pillar_img, kp1, img, kp2, matches[:10], None, flags=2)
    cv2.imshow("Pillar Detection", result_img)
    cv2.waitKey(1)

def image_callback(image_msg):
    np_arr = np.frombuffer(image_msg.data, dtype=np.uint8)
    img = cv2.imdecode(np_arr, cv2.IMREAD_COLOR)
    detect_pillar(img)

rospy.init_node("pillar_detector")
rospy.Subscriber("/camera/color/image_raw", Image, image_callback)
rospy.spin()

