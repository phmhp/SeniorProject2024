#!/usr/bin/env python3
import rospy
from nav_msgs.msg import Path
from geometry_msgs.msg import PoseWithCovarianceStamped
from geometry_msgs.msg import PoseStamped
class PathVisualizer:
    def __init__(self):
        rospy.init_node('path_visualizer', anonymous=True)

        # 퍼블리셔
        self.static_path_pub = rospy.Publisher('/target_path', Path, queue_size=10, latch=True)
        self.icp_path_pub = rospy.Publisher('/icp_path', Path, queue_size=10, latch=True)
        self.amcl_path_pub = rospy.Publisher('/amcl_path', Path, queue_size=10, latch=True)

        # Path 메시지 생성
        self.static_path_msg = Path()
        self.static_path_msg.header.frame_id = "map"

        self.icp_path_msg = Path()
        self.icp_path_msg.header.frame_id = "map"

        self.amcl_path_msg = Path()
        self.amcl_path_msg.header.frame_id = "map"

        # ICP pose, AMCL pose 구독
        self.icp_sub = rospy.Subscriber('/icp_pose', PoseWithCovarianceStamped, self.icp_callback)
        self.amcl_sub = rospy.Subscriber('/amcl_pose', PoseWithCovarianceStamped, self.amcl_callback)


    def icp_callback(self, msg):
        new_pose = PoseStamped()
        new_pose.header = msg.header
        new_pose.pose = msg.pose.pose

        self.icp_path_msg.poses.append(new_pose)
        self.icp_path_msg.header.stamp = rospy.Time.now()
        self.icp_path_pub.publish(self.icp_path_msg)

    def amcl_callback(self, msg):
        new_pose = PoseStamped()
        new_pose.header = msg.header
        new_pose.pose = msg.pose.pose

        self.amcl_path_msg.poses.append(new_pose)
        self.amcl_path_msg.header.stamp = rospy.Time.now()
        self.amcl_path_pub.publish(self.amcl_path_msg)

if __name__ == "__main__":
    visualizer = PathVisualizer()
    rospy.spin()

