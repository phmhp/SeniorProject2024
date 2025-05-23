from geometry_msgs.msg import PointStamped
import rospy

 # amcl_pose 토픽 발행 
def publish_amcl_pose_point(pub,pose_msg):
    pt = PointStamped()
    pt.header.frame_id = "map"
    pt.header.stamp = rospy.Time.now()
    pt.point.x = pose_msg.pose.pose.position.x
    pt.point.y = pose_msg.pose.pose.position.y
    #pt.point.z = 0.3  # 시각화를 위해 고정 높이
    pub.publish(pt)
    
    # 중심 point 발행 
def publish_center(crop_center, pub):
    pt = PointStamped()
    pt.header.frame_id = "map"
    pt.header.stamp = rospy.Time.now()
    pt.point.x = crop_center[0]
    pt.point.y = crop_center[1]
    pt.point.z = crop_center[2]
    pub.publish(pt)

    # transformation 행렬 기반 추정 결과 발행 
def publish_position_estimation(transformation,pub):
    # 변환 행렬에서 위치를 추출
    position = transformation[:3, 3]

    # 위치를 PointStamped 메시지로 변환하여 발행
    pt = PointStamped()
    pt.header.frame_id = "map"
    pt.header.stamp = rospy.Time.now()
    pt.point.x = position[0]
    pt.point.y = position[1]
    pt.point.z = position[2]
    
    # 위치 추정 결과 발행
    pub.publish(pt)
