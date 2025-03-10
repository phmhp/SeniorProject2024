import rospy
import tf 
import time
import math
import firebase_admin
from firebase_admin import credentials, db
from geometry_msgs.msg import PoseStamped, Quaternion
from actionlib_msgs.msg import GoalStatusArray 

# Firebase 초기화
cred = credentials.Certificate("----.json")
firebase_admin.initialize_app(cred, {
    'databaseURL': '---.app/'
})

# Firebase Realtime Database 참조
status_ref = db.reference('/UserResults/status')
destination_ref = db.reference('/UserResults/requiredIngredients')


# ROS 노드 초기화
rospy.init_node('navigation_controller', anonymous=True)
goal_pub = rospy.Publisher('/move_base_simple/goal', PoseStamped, queue_size=10) #목적지 전달 토픽,데이터 타입 
tf_listener = tf.TransformListener() 
rospy.sleep(2)  # 퍼블리셔 초기화 대기

# 로봇의 초기 위치 저장 (복귀 시 사용)
start_x, start_y, start_yaw = 0.0, 0.0, 0.0  # 수정 예정 

goal_reached = False  # 목적지 도착 여부 
last_status = None  #  이전 상태 저장 변수 (중복 출력 방지)
pending_logged = False  
succeeded_logged = False  
failed_logged = False
canceled_logged = False 


def get_current_position():
    """ 로봇의 현재 위치(x, y, yaw)를 반환하는 함수 """
    try:
        tf_listener.waitForTransform("/map", "/base_link", rospy.Time(0), rospy.Duration(3.0))
        (trans, rot) = tf_listener.lookupTransform("/map", "/base_link", rospy.Time(0))

        # 변환된 좌표 (x, y)
        x = trans[0]
        y = trans[1]

        # 회전값(Yaw) 계산 (Quaternion -> Euler 변환)
        _, _, yaw = tf.transformations.euler_from_quaternion(rot)

        return x, y, yaw

    except (tf.LookupException, tf.ConnectivityException, tf.ExtrapolationException):
        rospy.logwarn(" 현재 로봇 위치를 가져올 수 없습니다.")
        return None, None, None


def send_goal(x,y, yaw):
    """ 지정된 목적지 좌표로 로봇을 이동시키는 함수 """

    global goal_reached
    goal_reached = False #새로운 목적지 설정 시 초기화 

    #목표 위치 설정 
    goal = PoseStamped()
    goal.header.stamp = rospy.Time.now()
    goal.header.frame_id = "map"  # 맵 좌표계 기준 

    goal.pose.position.x = x
    goal.pose.position.y = y

    # 🔹 Yaw를 Quaternion으로 변환
    quaternion = tf.transformations.quaternion_from_euler(0, 0, yaw)
    goal.pose.orientation = Quaternion(*quaternion)  # (x, y, z, w)

    rospy.loginfo(f"📍 목표 지점 설정: X={x}, Y={y}, Yaw={yaw} rad")
    goal_pub.publish(goal)


def finish_action():
    """ 현재 위치를 기준으로 반바퀴 회전한 후, 초기 위치로 복귀하는 함수 """
    x, y, yaw = get_current_position()  # 로봇의 현재 위치를 가져옴

    if x is None or y is None or yaw is None:
        rospy.logwarn("⚠️ 현재 위치를 가져올 수 없어 회전 없이 복귀합니다.")
        send_goal(start_x, start_y, start_yaw) # 초기 위치 좌표 적용 업데이트 필요 (수정 얘정)  
        return

    rospy.loginfo(f"📍 현재 위치: X={x}, Y={y}, Yaw={yaw} rad")

    # 180도(π 라디안) 회전
    new_yaw = yaw + math.pi
    send_goal(x, y, new_yaw)
    rospy.loginfo("로봇 180도 회전 완료")

    rospy.sleep(2)  # 회전 후 대기

    # 초기 위치로 복귀
    send_goal(start_x, start_y, start_yaw)  # 초기 위치 좌표 적용 업데이트 필요 (수정 얘정)  
    rospy.loginfo("📍 초기 위치 복귀 완료")


def move_base_status_callback(msg):
    """ /move_base/status를 구독하여 목적지 도착 여부를 확인하는 콜백 함수 """
    global goal_reached, last_status, pending_logged, succeeded_logged, failed_logged, canceled_logged

    if msg.status_list:
        current_status = msg.status_list[-1].status  # 마지막 상태 값(현재 이동 상태)
        
        # PENDING(0) 상태 출력 (최초 1회 및 새로운 목적지 설정 하는 경우)
        if current_status == 0:
            if not pending_logged or last_status in [3, 4, 5]:  
                rospy.loginfo("[목적지 설정] 경로 탐색 시작")
                pending_logged = True  
            return

        # ACTIVE(1) 상태 출력 (이동 중인 경우)
        if current_status == 1:
            rospy.loginfo("[로봇 이동] 경로 탐색 진행 중...")
            rospy.sleep(2)  # 2초마다 한 번씩 출력
            return

        # SUCCEEDED(3) 상태 출력 (도착 시 1회)
        if current_status == 3 and not succeeded_logged:
            
            finish_action() #도착 시 동작 

            rospy.loginfo("[목적지 도착] 경로 탐색 종료")
            succeeded_logged = True
            goal_reached = True  
            pending_logged = False  
            return

        # ABORTED(4) 또는 REJECTED(5) 상태 출력 (경로 실패 시 1회)
        if current_status in [4, 5] and not failed_logged:
            rospy.loginfo("[실패] 경로 탐색 실패 / 목적지 설정 오류")
            failed_logged = True
            return

        # PREEMPTED(2), RECALLED(8), LOST(9) 상태 출력 (경로 취소 시 1회)
        if current_status in [2, 8, 9] and not canceled_logged:
            rospy.loginfo("[중단] 목적지 재설정 / 목적지 설정 취소")
            canceled_logged = True
            return

        # 중복 출력 방지
        if current_status == last_status:
            return
        last_status = current_status  

rospy.Subscriber('/move_base/status', GoalStatusArray, move_base_status_callback)


def simplify_status(status):
    """move_base의 상태 코드를 단순화하여 반환"""

    if status == 0:  # PENDING
        return "경로 탐색 시작" #목적지 설정됨 
    elif status == 1:  # ACTIVE
        return "이동 중"
    elif status == 3:  # SUCCEEDED
        return "목적지 도착"
    elif status in [4, 5]:  # ABORTED, REJECTED
        return "경로 탐색 실패"
    elif status in [2, 8, 9]:  # PREEMPTED, RECALLED, LOST
        return "목적지 안내 취소"
    else:
        return "알 수 없는 상태"



"""Firebase 관련 함수"""

def update_status(new_status):
    """ Firebase status 업데이트 함수 """
    status_ref.set(new_status)


def clear_required_ingredients():
    """ Firebase에서 requiredIngredients 값 삭제 """
    destination_ref.delete()


def wait_for_new_destination():
    """ Firebase에서 새로운 목적지 데이터를 가져옴 """
    while True:
        destination = destination_ref.get()
        if destination:  # 새로운 목적지가 입력됨
            rospy.loginfo(f"📍 새로운 목적지 입력됨: {destination}")
            return destination
        rospy.sleep(1)  # 1초마다 Firebase 확인


def monitor_status():
    """ Firebase에서 status 값을 실시간 모니터링하고 로봇을 제어하는 함수 """
    print("📡 Firebase에서 status 모니터링 중...")

    def callback(event):
        status = event.data
        print(f"현재 status: {status}")

        if status == "대기":
            print("목적지 확인 중...")
            destination = wait_for_new_destination()  # 새로운 목적지가 입력될 때까지 기다림
            

            """목적지를 좌표로 매핑하는 함수 필요함"""
            temp_x = 1
            temp_y = 1
            temp_yaw = 1

            send_goal(temp_x, temp_y, temp_yaw) # 목적지 <-> 좌표 매핑 필요 (수정 예정)
            #send_goal(coordinate(destination)) 이런 식으로 수정 에정 

            #move_to_destination(destination)  # 목적지 이동 정보 출력 

            # 목적지 도착 후 status를 '종료'로 업데이트하고 requiredIngredients를 삭제
            update_status("종료")
            clear_required_ingredients()

        elif status == "종료":
            finish_action()
            print(" 로봇이 시작 위치로 돌아갑니다.")

            # 시작 위치 도착 후 status를 '준비'로 업데이트
            update_status("준비")

    # Firebase 실시간 리스너 설정
    status_ref.listen(callback)


if __name__ == "__main__":
    monitor_status()

