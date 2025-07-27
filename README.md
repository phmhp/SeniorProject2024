![header](https://capsule-render.vercel.app/api?type=Rounded&color=FFFFFF&height=200&section=header&text=roverosong👾&desc=2024%20Senior%20Design%20Project%20❄️&descAlignY=10&fontSize=90&fontColor=000080&animation=twinkling&fontAlign=50)

## 시연 영상

[![Watch the video](https://img.youtube.com/vi/uujwvvBMSfM/maxresdefault.jpg)](https://youtu.be/uujwvvBMSfM?si=SKXc7GLhewVxGCYx)
## 시스템 개요 및 구성도 
<img width="1600" height="730" alt="system 구성도" src="https://github.com/user-attachments/assets/0ff9b128-3d08-4837-85f9-e8bd4368f75b" />
본 프로젝트는 사용자가 교내에서 선택한 목적지까지 로봇이 자율적으로 이동할 수 있도록 하는 실내 안내 시스템을 구현함. 로봇은 글로벌 경로를 기반으로 실시간 장애물 회피 기능을 수행하며 안전하게 목적지까지 주행함.

- ROS 노드를 중심으로 각 기능이 모듈화되어, 센서데이터 처리, 위치 추정, 경로 생성, 주행 제어, 장애물 회피 등을 수행

- 사용자는 태블릿 PC의 어플리케이션을 통해 목적지 선택 가능하며, 로봇은 해당 정보를 바탕으로 경로 생성 및 실시간 장애물 회피 수행

| 하드웨어 구성 | 설명 |
|------------|-------|
| 센서 |LiDAR, RGB-D 카메라 (RealSense D435)|
|구동계 | 휠 모듈, 모터 드라이버|
|제어 장치 |  MDUI (미들웨어)|
|기반 플랫폼 | MD로봇 사의 자율주행 로봇 플랫폼| 
|구동계 | 휠 모듈, 모터 드라이버|

## 로봇 통신 시스템 설계 
통신 구조 요약 

- 센서와 제어장치 간 RS232/RS435/USB시리얼 통신 구조 사용

- ROS시스템 ↔ 로봇 하드웨어 간 양방향 실시간 데이터 송수신


### 주요 ROS ↔ 제어기 통신 흐름

| 방향 | 내용 |
|------|------|
| ROS → 로봇 | `/cmd_vel`의 속도 명령이 시리얼 포트를 통해 제어기로 전달됨 |
| 로봇 → ROS | 위치, 배터리, 속도 등 센서 데이터가 `/robot/monitor`로 퍼블리시됨 |

### 사용 토픽 요약

|토픽|설명|
|-------|-------|
|/cmd_vel|PID 제어기에서 퍼블리시한 속도 명령|
|/odom|오도메트리 위치 추정 데이터|
|/robot/monitor|로봇의 실시간 상태 정보 (위치, 속도, 센서 등)|
|/cmd_destination|사용자 앱에서 목적지 전송|


## 실행 방법

**1. 2D LiDAR 기반 SLAM**

```bash
roscore
roslaunch turtlebot3_teleop turtlebot3_teleop_key.launch #조종기 수동 조종 명령어
rosrun robot_setup_tf robot_controller.py #로봇 상태 데이터 수신 
roslaunch rplidar_ros karto_rplidar_a2m8.launch #2D Lidar 실행 

rosrun map_server map_saver -f /home/roverosong/Maps/mapname #맵 저장
```

**2. RGBD 카메라 기반 SLAM**
```bash
roscore
roslaunch realsense2_camera rs_camera.launch align_depth:=true #카메라 실행 
roslaunch rtabmap_ros rtabmap.launch \
    localization:=false \
    rtabmapviz:=true \
    rviz:=true \
    database_path:=~/.ros/new_rtabmap.db \
    rgb_topic:=/camera/color/image_raw \
    depth_topic:=/camera/aligned_depth_to_color/image_raw \
    camera_info_topic:=/camera/color/camera_info \
    frame_id:=camera_link #rtabmap slam 실행 


rtabmap-databaseViewer ~/.ros/new_rtabmap.db #생성한 3D맵 확인
```

**3. Global Path를 활용한 자율주행 안내 실행**

해당 스크립트 실행 전, 3-1을 통해 맵 위에서 시작점부터 경유점, 종점까지 최소 4개 이상 점을 찍어 글로벌 경로 생성 필요

**3-1. Global Path 사전 생성**
```bash
rosrun my_robot_navigation make_path.py 
```

```bash
roscore
roslaunch turtlebot3_teleop turtlebot3_teleop_key.launch #조종기 수동 조종 명령어
rosrun robot_setup_tf robot_global_navi.py #로봇 상태 정보 수신 
roslaunch my_robot_navigation move_base.launch #경로 계획 launch 실행 
rosrun my_robot_navigation globl_obstacle_m_arc.py #실시간 장애물 회피 실행 
```
**4. ICP 기반 Localization 보정 실행**
```bash
roslaunch localization_correction icp.launch #icp 위치 추정 launch 실행 
```

## ICP 기반 위치 추정
<img width="400" height="250" alt="image" src="https://github.com/user-attachments/assets/fa003330-5791-44be-8174-20604f86bd15" /> <img width="400" height="250" alt="image" src="https://github.com/user-attachments/assets/8422c2b7-0bb8-43a1-b56a-69944a14ac32" />


### 목적
기존 2D 위치 추정 기법인 **AMCL**은 반복되거나 대칭된 환경에서 누적 오차(drift)가 발생하며 정확도가 떨어졌음. 이를 보완하기 위해 **Point-to-Plane ICP** 기반의 보조 위치 추정 파이프라인을 별도로 구축함.



### 핵심 구성 요소 및 역할

| 구성 요소 | 설명 |
|------------|-------|
| `/camera/color/image_raw`<br>`/camera/aligned_depth_to_color/image_raw` | RealSense D435에서 RGB 및 depth 데이터 수신 |
| `/map_cloud` | 사전 생성된 3D reference 맵(PointCloud2) |
| `localization_correction` | 실시간 pointcloud ↔ reference map 정합 수행 (ICP) |
| `/amcl_pose` | AMCL 위치 추정 결과, ICP 초기값으로 사용 |
| `/mock_tf`<br>`/initialpose` | 시뮬레이션 환경에서 테스트용 초기 pose 발행 |
| `/icp_pose` | ICP 위치 정합 결과 발행 |
| `visualize_path` | `/icp_pose`, `/amcl_pose`, `/odom`을 받아 `/icp_path`로 시각화 |



### rqt_graph 로직 흐름 설명
<img width="4070" height="688" alt="rosgraph" src="https://github.com/user-attachments/assets/569a899a-6113-4251-a412-1cfe66d36855" />

1. **센서 입력**
   - RealSense D435 RGB-D 카메라에서 실시간 영상 및 깊이 정보 수신
   - `/camera/color/image_raw`, `/camera/aligned_depth_to_color/image_raw` → `localization_correction` 노드로 전달
     
2. **기준 맵**
   - `/map_cloud`를 기준으로 ICP 정합 수행
     
3. **초기 위치 입력**
   - `/amcl_pose` 또는 `/initialpose` → `/mock_tf`를 통해 ICP 초기 추정 위치 제공
     
4. **위치 정합 및 발행**
   - ICP 연산을 통해 `/icp_pose` 결과 발행
     
5. **시각화**
   - `visualize_path` 노드가 `/icp_pose`, `/amcl_pose`, `/odom`을 기반으로 `/icp_path` 생성
   - 결과를 Rviz에서 실시간으로 시각화



### 장점 및 한계
| 장점 | 한계 |
|------|------|
| 정밀한 3D 맵 기반 위치 정합 가능 | 연산량이 높아 실시간 적용에 부적합 |
| 누적 오차 없이 절대 위치 기준 정합 수행 | 초기 추정 위치 오류 시 정합 실패 가능 |



### 활용 방식 및 향후 방향
- `/icp_pose`는 robot_localization 등 연동 가능
- 현재는 시뮬레이션 기반 연산 성능 검증
- 향후 실시간 적용을 위한 연산 최적화 필요
---
## 실행 환경
- Ubuntu 20.04 ROS1 noetic
  librealsense2 version 2.50
  
## 포트 고정 방법
```bash
1. 규칙 파일 위치 이동 및 확인
cd /etc/udev/rules.d && ls
2. 규칙 파일 생성 또는 기존 파일 수정
sudo gedit 50-myusbrules.rules
```
3. 아래 내용 추가
```bash
SUBSYSTEM=="usb", ATTRS{idVendor}=="2109", ATTRS{idProduct}=="2812", SYMLINK+="multiHub"
SUBSYSTEM=="tty", ATTRS{idVendor}=="0403", ATTRS{idProduct}=="6001", SYMLINK+="robotInfo"
SUBSYSTEM=="tty", ATTRS{idVendor}=="10c4", ATTRS{idProduct}=="ea60", MODE="0666", SYMLINK+="rplidar"
※ 기존에 다른 .rules 파일이 있는 경우 위 내용을 그대로 추가하면 됩니다.
```
4. 규칙 적용
```bash
sudo udevadm control --reload-rules
sudo udevadm trigger
```
