<!--
참고 링크 
이모지 : https://emojipedia.org/ 
헤더 : https://github.com/kyechan99/capsule-render?tab=readme-ov-file#transparent 
마크다운 문법 : https://www.heropy.dev/p/B74sNE
기술스택 아이콘 : https://simpleicons.org/
깃허브 꾸미기 벨로그 글 : https://velog.io/@zerra18/%EB%A6%B3%EA%BE%B8readme-%EA%BE%B8%EB%AF%B8%EA%B8%B0-%EB%B1%83%EC%A7%80%EB%8F%84-%EB%8B%AC%EA%B3%A0-%EB%B0%B0%EA%B2%BD%EB%8F%84-%EA%BE%B8%EB%A9%B0%EB%B3%B4%EC%9E%90
-->
![header](https://capsule-render.vercel.app/api?type=Rounded&color=FFFFFF&height=200&section=header&text=roverosong👾&desc=2024%20Senior%20Design%20Project%20❄️&descAlignY=10&fontSize=90&fontColor=000080&animation=twinkling&fontAlign=50)
---
## 실행 방법
```sh
roslaunch turtlebot3_teleop turtlebot3_teleop_key.launch
```
```sh
roslaunch robot_setup_tf robot_navigate.launch
```
```sh
roslaunch move_base_navigation navigation.launch
```
---
## 포트고정 방법 
```sh
cd /etc/udev/rules.d && ls
```
```sh
sudo gedit 50-myusbrules.rules
```
```sh
SUBSYSTEM=="usb", ATTRS{idVendor}=="2109", ATTRS{idProduct}=="2812", SYMLINK+="multiHub"
SUBSYSTEM=="tty", ATTRS{idVendor}=="0403", ATTRS{idProduct}=="6001", SYMLINK+="robotInfo"
SUBSYSTEM=="tty", ATTRS{idVendor}=="10c4", ATTRS{idProduct}=="ea60",MODE="0666",SYMLINK+="rplidar"
```

기존의 다른 이름의.rules가 존재할 경우엔 위 내용 추가 적용하면 됨 
```sh
sudo udevadm control --reload-rules
sudo udevadm trigger
```
