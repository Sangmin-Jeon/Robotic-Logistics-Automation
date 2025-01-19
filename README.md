# 로봇을 이용한 물류 운반 자동화 시스템
<div style="display: flex; justify-content: space-between; align-items: center;">
  <img src="https://github.com/user-attachments/assets/d84e535f-16b6-4160-a024-b68428ab2795" alt="Image 1" width="50%" />
  <img src="https://github.com/user-attachments/assets/a0c99171-80f0-4b44-a1bf-4444d9ad41f0" alt="Image 2" width="46%" />
</div>



## Overview
- **프로젝트 설명**: 제품 공정에서 부품을 골라 컨베이어 벨트로 옮겨 목적지로 이동시키는 작업을, 로봇을 이용하여 자동화하는 시스템입니다.  
- **프로젝트 기간**: 2024.12.3 ~ 2024.12.9
- **팀 구성**: 3명

## 구현 내용 
| 기능    | 설명 |
|----------|---------------|
| 로봇 위치 인식 및 이동 | ArUco Marker를 이용한 로봇 위치 인식 및 이동 |, 
| 부품 객체 검출 | Yolo를 사용하여 로봇 카메라를 통해 부품 객체 실시간 검출 |
| 부품 이송 | Manipulator로 부품을 집어서 컨베이어벨트로 이송 | 
| 컨베이어벨트 제어| 부품이 올라오면 컨베이어벨트 작동 |


## Tech / Skill
-	Ros2 humble  
-	Python  
-	YOLOv8  
-	Git  
-	Jetson Nano    
-	OpenCV  
-	PyQt  
-	Arduino Uno  
-	OpenManipulator-X  
-	Turtlebot3 Waffle Pi  


## Quick Start

### # Connect to TurtleBot3 via SSH for remote access
- 사용 중인 PC에서 SSH를 통해 TurtleBot3에 접속하세요.
~~~bash
ssh -X [username]@[TurtleBot3_IP_Address]
~~~

### # Access TurtleBot3 and run it there
- Turtlebot3 manipulation bringup
~~~bash
ros2 launch turtlebot3_manipulation_bringup hardware.launch.py
~~~

- Turtlebot3 manipulation servo
~~~bash
ros2 launch turtlebot3_manipulation_moveit_config servo.launch.py
~~~

- Robot node
~~~bash
ros2 run b4_fulfillment robot
~~~

- Manipulation node
~~~bash
ros2 run b4_fulfillment manipulation  
~~~

- Robot_eyes node
~~~bash
ros2 run b4_fulfillment robot_eyes ~/B4_Fulfillment_ws/src/b4_fulfillment/best.pt
~~~

### # Execute this command on Your Pc
- webcam node
~~~bash
ros2 run b4_fulfillment webcam
~~~

- Gui node
~~~bash
ros2 run b4_fulfillment gui
~~~

- Conveyor node
~~~bash
ros2 run b4_fulfillment conveyor
~~~
