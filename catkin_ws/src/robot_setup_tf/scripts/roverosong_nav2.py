import os
import time
import firebase_admin
from firebase_admin import credentials, db
from gtts import gTTS
import subprocess
import threading

# Firebase 초기화
cred = credentials.Certificate("/home/roverosong/catkin_ws/src/robot_setup_tf/rovero-9059c-firebase-adminsdk-rrzbn-9bd7f92153.json")
firebase_admin.initialize_app(cred, {
    'databaseURL': 'https://rovero-9059c-default-rtdb.asia-southeast1.firebasedatabase.app/'
})

# Firebase Realtime Database 참조
status_ref = db.reference('/UserResults/status')
destination_ref = db.reference('/UserResults/requiredIngredients')

#리스너 중복 방지 플래그
firebase_listener_initialized = False
audio_lock = threading.Lock()

def play_audio(text):
    """음성을 생성하고 로봇 스피커로 출력하는 함수 (중복 방지)"""
    with audio_lock:  # 여러 스레드에서 동시에 실행되지 않도록 방지
        tts = gTTS(text=text, lang='ko')
        tts.save("/tmp/tts.mp3")

        # 기존 실행 중인 ffplay 프로세스 종료 (중복 재생 방지)
        os.system("pkill -f 'ffplay'")

        # 새로운 음성 재생 (출력 숨김)
        subprocess.call(["ffplay", "-nodisp", "-autoexit", "/tmp/tts.mp3"], stdout=subprocess.DEVNULL, stderr=subprocess.DEVNULL)
    
def update_status(new_status):
    """ Firebase status 업데이트 함수 """
    status_ref.set(new_status)

def clear_required_ingredients():
    """ Firebase에서 requiredIngredients 값 삭제 """
    destination_ref.delete()

def check_termination():
    """ Firebase에서 status가 종료인지 확인하는 함수 """
    return status_ref.get() == "종료"

def generate_path(destination):
    """ 목적지 경로 생성 함수 """
    play_audio("안내를 시작합니다. 저를 따라오세요.")
    print(f"{destination} 경로 생성 중...")
    for _ in range(4):  # 2초 동안 상태 확인
        if check_termination():
            print("경로 생성 중단: 종료 신호 감지")
            clear_required_ingredients()
            return_to_start()
            return None  # 경로 생성 중단 시 이동하지 않도록 None 반환
        time.sleep(0.5)
    print(f" 완료: {destination}로 이동!")
    return destination  # 정상적으로 경로 생성 완료되면 목적지 반환

def move_to_destination(destination):
    """ 로봇 이동 함수 """
    if not destination:
        return  # 목적지가 None이면 이동하지 않음
    print(f"{destination}로 이동 중...")
    for _ in range(10):  # 5초 동안 상태 확인
        if check_termination():
            print("이동 중단: 종료 신호 감지")
            clear_required_ingredients()
            return_to_start()
            return
        time.sleep(0.5)
    print(f" {destination} 도착!")
    update_status("종료")
    play_audio("목적지에 도착했습니다.")
    clear_required_ingredients()
    return_to_start()

def return_to_start():
    """ 로봇이 시작 위치로 복귀하는 함수 """
    play_audio("처음 위치로 돌아갑니다. 이용해주셔서 감사합니다.")
    print("복귀 중...")
    time.sleep(3)
    print(" 복귀 완료!")
    update_status("준비")

def wait_for_new_destination():
    """ 새로운 목적지가 입력될 때까지 대기 """
    while True:
        destination = destination_ref.get()
        if destination:  # 새로운 목적지가 입력됨
            print(f" 새로운 목적지 입력됨: {destination}")
            return destination
        time.sleep(1)

def monitor_status():
    """ Firebase에서 status 값을 실시간 모니터링하고 로봇을 제어하는 함수 """
    print("📡 Firebase에서 status 모니터링 중...")
    
    def callback(event):
        status = event.data
        print(f"현재 status: {status}")

        if status == "대기":
            print("목적지 확인 중...")
            destination = wait_for_new_destination()  # 새로운 목적지가 입력될 때까지 기다림
            destination = generate_path(destination)  # 경로 생성
            move_to_destination(destination) # 목적지가 None이 아닐 때만 이동

    # Firebase 실시간 리스너 설정
    status_ref.listen(callback)

if __name__ == "__main__":
    monitor_status()

