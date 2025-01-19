# 로봇을 이용한 물류 운반 자동화 시스템
<div style="display: flex; justify-content: space-between; align-items: center;">
  <img src="https://github.com/user-attachments/assets/d84e535f-16b6-4160-a024-b68428ab2795" alt="Image 1" width="45%" />
  <img src="https://github.com/user-attachments/assets/a4acd769-f215-4d1e-8660-c0e7f7e1cc5d" alt="Image 2" width="45%" />
</div>

## Overview
제품 공정에서 부품을 골라 컨베이어 벨트로 옮겨 목적지로 이동시키는 작업을, 로봇을 이용하여 자동화하는 시스템입니다.  

## Tech / Skill
Ros2, Python, YOLO, Git, Jetson_Nano, Linux, OpenCV,   
PyQT, Arduino_Uno, OpenManipulator-X, Turtlebot3_waffle_pi

## Quick Start

### # Connect to TurtleBot3 using SSH
- ssh를 사용해 사용중인 pc에서 turtlebot3에 접속하세요.
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
