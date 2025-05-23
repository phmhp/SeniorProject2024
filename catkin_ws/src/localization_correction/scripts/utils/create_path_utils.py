from geometry_msgs.msg import PointStamped
import rospy
clicked_points = []

def clicked_point_callback(msg):
    clicked_points.append((msg.point.x, msg.point.y))
    rospy.loginfo(f"Point clicked: ({msg.point.x:.2f}, {msg.point.y:.2f})")

if __name__ == "__main__":
    rospy.init_node("clicked_point_collector")
    rospy.Subscriber("/clicked_point", PointStamped, clicked_point_callback)
    rospy.spin()
    
    # 종료될 때 파일 저장
    with open("/home/hyemin/catkin_ws/src/localization_correction/cfg/path.txt", "w") as f:
        for x, y in clicked_points:
            f.write(f"{x} {y}\n")

