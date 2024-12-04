import sys
import time
import rclpy
from rclpy.node import Node
from rclpy.action import ActionClient, GoalResponse, CancelResponse
from rclpy.executors import MultiThreadedExecutor
from rclpy.callback_groups import ReentrantCallbackGroup
from PyQt5.QtWidgets import (QApplication, QWidget, QPushButton, QLabel, QLineEdit, QListWidget, QTextEdit,
                             QStackedWidget, QDialog, QVBoxLayout, QHBoxLayout, QGroupBox)
from PyQt5.QtCore import Qt, QTimer
from PyQt5.QtGui import QPixmap, QPalette, QColor, QImage
from PyQt5.QtWidgets import QLabel
from b4_fulfillment_interfaces.msg import Button, RobotStatus  # Import the custom message type
from b4_fulfillment_interfaces.action import Conveyor
from std_msgs.msg import String
import numpy as np
from sensor_msgs.msg import Image


class LoginUI(QWidget):
    def __init__(self, stacked_widget):
        super().__init__()
        self.stacked_widget = stacked_widget
        self.initUI()

    def initUI(self):
        # 아이디 입력 필드
        self.id_input = QLineEdit(self)
        self.id_input.setPlaceholderText("아이디")
        self.id_input.setGeometry(500, 100, 200, 40)  # 위치 (x, y)와 크기 (width, height) 설정

        # 패스워드 입력 필드
        self.password_input = QLineEdit(self)
        self.password_input.setPlaceholderText("패스워드")
        self.password_input.setEchoMode(QLineEdit.Password)
        self.password_input.setGeometry(500, 160, 200, 40)  # 위치 (x, y)와 크기 (width, height) 설정

        # 로그인 버튼
        self.login_btn = QPushButton('로그인', self)
        self.login_btn.setGeometry(550, 220, 100, 40)  # 위치 (x, y)와 크기 (width, height) 설정
        self.login_btn.clicked.connect(self.check_login)

        self.setFixedSize(800, 400)  # 로그인 창 크기 고정
        self.setWindowTitle('로그인')

    def check_login(self):
        # 간단한 로그인 검증 (아이디와 패스워드가 모두 입력되었는지 확인)
        if self.id_input.text() and self.password_input.text():
            # 로그인 성공 시 메인 화면으로 이동
            self.stacked_widget.setCurrentIndex(1)


class RobotControlUI(QWidget):
    def __init__(self, gui_node):
        super().__init__()
        self.gui_node = gui_node
        self.initUI()

    def initUI(self):
        # 작업에 소요된 시간
        time_group = QGroupBox("작업에 소요된 시간", self)
        time_group.setGeometry(20, 20, 150, 80)  # 작업에 소요된 시간 그룹 박스 크기 조정
        time_layout = QVBoxLayout()
        self.time_label = QLabel('00:00', self)
        time_layout.addWidget(self.time_label)
        time_group.setLayout(time_layout)

        # 실시간 로봇 상태 표시
        status_group = QGroupBox("실시간 로봇 상태 표시", self)
        status_group.setGeometry(200, 20, 200, 80)  # 실시간 로봇 상태 표시 그룹 박스 크기 조정
        status_layout = QVBoxLayout()
        self.robot_status_label = QLabel('상태 없음', self)
        status_layout.addWidget(self.robot_status_label)
        status_group.setLayout(status_layout)

        # 이메일 주소 입력 필드
        self.email_input = QLineEdit(self)
        self.email_input.setPlaceholderText('이메일: jiin772@naver.com')
        self.email_input.setGeometry(450, 60, 250, 30)  # 위치 (x, y)와 크기 (width, height) 설정

        # 월드 뷰 카메라 이미지
        self.world_view_label = QLabel(self)
        self.world_view_label.setGeometry(50, 130, 400, 300)  # 위치 (x, y)와 크기 (width, height) 설정

        # 로봇 카메라 이미지
        self.robot_image_label = QLabel(self)
        self.robot_image_pixmap = QPixmap("wait_robot.gif")
        self.robot_image_label.setPixmap(self.robot_image_pixmap)
        self.robot_image_label.setScaledContents(True)
        self.robot_image_label.setGeometry(500, 130, 400, 300)  # 위치 (x, y)와 크기 (width, height) 설정

        # 학습 데이터 수집 버튼
        self.data_collect_btn = QPushButton('학습 데이터 수집 버튼', self)
        self.data_collect_btn.setGeometry(950, 100, 150, 40)  # 위치 (x, y)와 크기 (width, height) 설정
        self.data_collect_btn.setStyleSheet("background-color: lightgreen; color: black;")

        # 작업 목록 버튼
        self.job_list_btn = QPushButton('작업 목록 보기', self)
        self.job_list_btn.setGeometry(950, 150, 150, 40)  # 위치 (x, y)와 크기 (width, height) 설정
        self.job_list_btn.clicked.connect(self.show_job_list)
        self.job_list_btn.setStyleSheet("background-color: lightblue; color: black;")

        # 작업 제어 버튼들
        self.start_btn = QPushButton('start', self)
        self.start_btn.setGeometry(50, 450, 100, 40)  # 위치 (x, y)와 크기 (width, height) 설정
        self.start_btn.clicked.connect(self.start_timer)
        self.start_btn.clicked.connect(lambda: self.job_list_btn.setDisabled(True))  # 작업 목록 보기 비활성화
        self.start_btn.clicked.connect(lambda: self.gui_node.publish_button_press("start"))
        self.start_btn.setStyleSheet("background-color: lightblue; color: black;")

        self.stop_btn = QPushButton('stop', self)
        self.stop_btn.setGeometry(160, 450, 100, 40)  # 위치 (x, y)와 크기 (width, height) 설정
        self.stop_btn.clicked.connect(self.stop_timer)
        self.stop_btn.clicked.connect(lambda: self.gui_node.publish_button_press("stop"))
        self.stop_btn.setStyleSheet("background-color: red; color: black;")

        self.pause_btn = QPushButton('pause', self)
        self.pause_btn.setGeometry(270, 450, 100, 40)  # 위치 (x, y)와 크기 (width, height) 설정
        self.pause_btn.clicked.connect(self.pause_timer)
        self.pause_btn.clicked.connect(lambda: self.gui_node.publish_button_press("pause"))
        self.pause_btn.setStyleSheet("background-color: yellow; color: black;")

        self.resume_btn = QPushButton('resume', self)
        self.resume_btn.setGeometry(380, 450, 100, 40)  # 위치 (x, y)와 크기 (width, height) 설정
        self.resume_btn.clicked.connect(self.resume_timer)
        self.resume_btn.clicked.connect(lambda: self.gui_node.publish_button_press("resume"))
        self.resume_btn.setStyleSheet("background-color: green; color: black;")

        self.reset_btn = QPushButton('reset', self)
        self.reset_btn.setGeometry(490, 450, 100, 40)  # 위치 (x, y)와 크기 (width, height) 설정
        self.reset_btn.clicked.connect(self.reset_timer)
        self.reset_btn.clicked.connect(lambda: self.job_list_btn.setDisabled(False))  # 작업 목록 보기 활성화
        self.reset_btn.clicked.connect(lambda: self.gui_node.publish_button_press("reset"))
        self.reset_btn.setStyleSheet("background-color: purple; color: black;")

        self.manual_control_btn = QPushButton('컨베이어 이동', self)
        self.manual_control_btn.setGeometry(600, 450, 150, 40)  # 위치 (x, y)와 크기 (width, height) 설정
        self.manual_control_btn.clicked.connect(self.move_conveyor)
        self.manual_control_btn.setStyleSheet("background-color: lightpink; color: black;")

        self.manual_stop_btn = QPushButton('컨베이어 정지', self)
        self.manual_stop_btn.setGeometry(760, 450, 150, 40)  # 위치 (x, y)와 크기 (width, height) 설정
        self.manual_stop_btn.setStyleSheet("background-color: lightyellow; color: black;")

        # 작업 선택 표시 라벨
        self.selected_job_label = QLabel('선택된 작업: 없음', self)
        self.selected_job_label.setGeometry(950, 200, 200, 40)  # 위치 (x, y)와 크기 (width, height) 설정
        self.selected_job_label.setStyleSheet("border: 1px solid black; padding: 5px; background-color: #f0f0f0;")

        # 타이머 설정
        self.timer = QTimer(self)
        self.timer.timeout.connect(self.update_time)
        self.time_seconds = 0

        # ROS2 노드로부터 상태 업데이트를 위한 타이머 설정
        self.ros_timer = QTimer(self)
        self.ros_timer.timeout.connect(self.gui_node.spin_once)
        self.ros_timer.start(100)

        self.job_selected = False

        self.setWindowTitle('로봇 작업 제어 시스템')
        self.setGeometry(100, 100, 1200, 600)
        self.show()

    def show_job_list(self):
        # 작업 목록을 보여주는 파이어냐처 생성
        job_dialog = QDialog(self)
        job_dialog.setWindowTitle("작업 목록 선택")
        job_dialog.setFixedSize(200, 200)

        job_layout = QVBoxLayout()

        # Job 버튼들 생성
        job_buttons_layout = QVBoxLayout()
        for job_name in ["Job 1", "Job 2", "Job 3"]:
            job_button = QPushButton(job_name, job_dialog)
            job_button.clicked.connect(lambda checked, name=job_name: self.select_job(name, job_dialog))
            job_buttons_layout.addWidget(job_button)

        job_layout.addLayout(job_buttons_layout)

        close_btn = QPushButton("닫기", job_dialog)
        close_btn.clicked.connect(job_dialog.accept)
        job_layout.addWidget(close_btn)

        job_dialog.setLayout(job_layout)
        job_dialog.exec_()

    def select_job(self, job_name, dialog):
        # 선택된 작업을 라벨에 표시
        self.selected_job_label.setText(f'선택된 작업: {job_name}')
        self.job_selected = True
        dialog.accept()

    def start_timer(self):
        # 타이머 시작 (작업이 선택된 경우에만)
        if self.job_selected:
            self.timer.start(1000)

    def stop_timer(self):
        # 타이머 정지
        self.timer.stop()
        # reset 버튼을 제어한 모두의 버튼 비활성화
        self.start_btn.setDisabled(True)
        self.pause_btn.setDisabled(True)
        self.resume_btn.setDisabled(True)
        self.data_collect_btn.setDisabled(True)
        self.job_list_btn.setDisabled(True)
        self.manual_control_btn.setDisabled(True)
        self.manual_stop_btn.setDisabled(True)

    def pause_timer(self):
        # 타이머 일시 정지
        self.timer.stop()

    def resume_timer(self):
        # 타이머 재개 (작업이 선택된 경우에만)
        if self.job_selected:
            self.timer.start(1000)

    def reset_timer(self):
        # 타이머 리셋 및 작업 선택 해제
        self.timer.stop()
        self.time_seconds = 0
        self.time_label.setText('00:00')
        self.job_selected = False
        self.selected_job_label.setText('선택된 작업: 없음')
        # 모든 버튼 다시 활성화
        self.start_btn.setDisabled(False)
        self.pause_btn.setDisabled(False)
        self.resume_btn.setDisabled(False)
        self.data_collect_btn.setDisabled(False)
        self.job_list_btn.setDisabled(False)
        self.manual_control_btn.setDisabled(False)
        self.manual_stop_btn.setDisabled(False)

    def update_time(self):
        # 타이머 업데이트
        self.time_seconds += 1
        minutes = self.time_seconds // 60
        seconds = self.time_seconds % 60
        self.time_label.setText(f'{minutes:02}:{seconds:02}')

    def update_robot_status(self, status):
        # 로봇 상태 업데이트
        self.robot_status_label.setText(status)


import sys
import time
import rclpy
from rclpy.node import Node
from rclpy.action import ActionClient
from rclpy.action import ActionServer, GoalResponse, CancelResponse
from PyQt5.QtWidgets import (QApplication, QWidget, QPushButton, QLabel, QLineEdit, QListWidget, QTextEdit,
                             QStackedWidget, QDialog, QVBoxLayout, QHBoxLayout, QGroupBox)
from PyQt5.QtCore import Qt, QTimer
from PyQt5.QtGui import QPixmap, QPalette, QColor, QImage
from PyQt5.QtWidgets import QLabel
from b4_fulfillment_interfaces.msg import Button, RobotStatus  # Import the custom message type
from b4_fulfillment_interfaces.action import Conveyor
from std_msgs.msg import String
import numpy as np
from sensor_msgs.msg import Image


class LoginUI(QWidget):
    def __init__(self, stacked_widget):
        super().__init__()
        self.stacked_widget = stacked_widget
        self.initUI()

    def initUI(self):
        # 아이디 입력 필드
        self.id_input = QLineEdit(self)
        self.id_input.setPlaceholderText("아이디")
        self.id_input.setGeometry(500, 100, 200, 40)  # 위치 (x, y)와 크기 (width, height) 설정

        # 패스워드 입력 필드
        self.password_input = QLineEdit(self)
        self.password_input.setPlaceholderText("패스워드")
        self.password_input.setEchoMode(QLineEdit.Password)
        self.password_input.setGeometry(500, 160, 200, 40)  # 위치 (x, y)와 크기 (width, height) 설정

        # 로그인 버튼
        self.login_btn = QPushButton('로그인', self)
        self.login_btn.setGeometry(550, 220, 100, 40)  # 위치 (x, y)와 크기 (width, height) 설정
        self.login_btn.clicked.connect(self.check_login)

        self.setFixedSize(800, 400)  # 로그인 창 크기 고정
        self.setWindowTitle('로그인')

    def check_login(self):
        # 간단한 로그인 검증 (아이디와 패스워드가 모두 입력되었는지 확인)
        if self.id_input.text() and self.password_input.text():
            # 로그인 성공 시 메인 화면으로 이동
            self.stacked_widget.setCurrentIndex(1)


class RobotControlUI(QWidget):
    def __init__(self, gui_node):
        super().__init__()
        self.gui_node = gui_node
        self.initUI()

    def initUI(self):
        # 작업에 소용된 시간
        time_group = QGroupBox("작업에 소용된 시간", self)
        time_group.setGeometry(20, 20, 150, 80)  # 작업에 소용된 시간 그룹 박스 크기 조정
        time_layout = QVBoxLayout()
        self.time_label = QLabel('00:00', self)
        time_layout.addWidget(self.time_label)
        time_group.setLayout(time_layout)

        # 실시간 로벌 상태 표시
        status_group = QGroupBox("실시간 로벌 상태 표시", self)
        status_group.setGeometry(200, 20, 200, 80)  # 실시간 로벌 상태 표시 그룹 박스 크기 조정
        status_layout = QVBoxLayout()
        self.robot_status_label = QLabel('상태 없음', self)
        status_layout.addWidget(self.robot_status_label)
        status_group.setLayout(status_layout)

        # 이메일 주소 입력 필드
        self.email_input = QLineEdit(self)
        self.email_input.setPlaceholderText('이메일: jiin772@naver.com')
        self.email_input.setGeometry(450, 60, 250, 30)  # 위치 (x, y)와 크기 (width, height) 설정

        # 월드 뷰 카메라 이미지
        self.world_view_label = QLabel(self)
        self.world_view_label.setGeometry(50, 130, 400, 300)  # 위치 (x, y)와 크기 (width, height) 설정

        # 로벌 카메라 이미지
        self.robot_image_label = QLabel(self)
        self.robot_image_pixmap = QPixmap("wait_robot.gif")
        self.robot_image_label.setPixmap(self.robot_image_pixmap)
        self.robot_image_label.setScaledContents(True)
        self.robot_image_label.setGeometry(500, 130, 400, 300)  # 위치 (x, y)와 크기 (width, height) 설정

        # 학습 데이터 수집 버튼
        self.data_collect_btn = QPushButton('학습 데이터 수집 버튼', self)
        self.data_collect_btn.setGeometry(950, 100, 150, 40)  # 위치 (x, y)와 크기 (width, height) 설정
        self.data_collect_btn.setStyleSheet("background-color: lightgreen; color: black;")

        # 작업 목록 버튼
        self.job_list_btn = QPushButton('작업 목록 보기', self)
        self.job_list_btn.setGeometry(950, 150, 150, 40)  # 위치 (x, y)와 크기 (width, height) 설정
        self.job_list_btn.clicked.connect(self.show_job_list)
        self.job_list_btn.setStyleSheet("background-color: lightblue; color: black;")

        # 작업 제어 버튼들
        self.start_btn = QPushButton('start', self)
        self.start_btn.setGeometry(50, 450, 100, 40)  # 위치 (x, y)와 크기 (width, height) 설정
        self.start_btn.clicked.connect(self.start_timer)
        self.start_btn.clicked.connect(lambda: self.job_list_btn.setDisabled(True))  # 작업 목록 보기 비활성화
        self.start_btn.clicked.connect(lambda: self.gui_node.publish_button_press("start"))
        self.start_btn.clicked.connect(self.move_conveyor)
        self.start_btn.setStyleSheet("background-color: lightblue; color: black;")

        self.stop_btn = QPushButton('stop', self)
        self.stop_btn.setGeometry(160, 450, 100, 40)  # 위치 (x, y)와 크기 (width, height) 설정
        self.stop_btn.clicked.connect(self.stop_timer)
        self.stop_btn.clicked.connect(lambda: self.gui_node.publish_button_press("stop"))
        self.stop_btn.setStyleSheet("background-color: red; color: black;")

        self.pause_btn = QPushButton('pause', self)
        self.pause_btn.setGeometry(270, 450, 100, 40)  # 위치 (x, y)와 크기 (width, height) 설정
        self.pause_btn.clicked.connect(self.pause_timer)
        self.pause_btn.clicked.connect(lambda: self.gui_node.publish_button_press("pause"))
        self.pause_btn.setStyleSheet("background-color: yellow; color: black;")

        self.resume_btn = QPushButton('resume', self)
        self.resume_btn.setGeometry(380, 450, 100, 40)  # 위치 (x, y)와 크기 (width, height) 설정
        self.resume_btn.clicked.connect(self.resume_timer)
        self.resume_btn.clicked.connect(lambda: self.gui_node.publish_button_press("resume"))
        self.resume_btn.setStyleSheet("background-color: green; color: black;")

        self.reset_btn = QPushButton('reset', self)
        self.reset_btn.setGeometry(490, 450, 100, 40)  # 위치 (x, y)와 크기 (width, height) 설정
        self.reset_btn.clicked.connect(self.reset_timer)
        self.reset_btn.clicked.connect(lambda: self.job_list_btn.setDisabled(False))  # 작업 목록 보기 활성화
        self.reset_btn.clicked.connect(lambda: self.gui_node.publish_button_press("reset"))
        self.reset_btn.setStyleSheet("background-color: purple; color: black;")

        self.manual_control_btn = QPushButton('컨베이어 이동', self)
        self.manual_control_btn.setGeometry(600, 450, 150, 40)  # 위치 (x, y)와 크기 (width, height) 설정
        self.manual_control_btn.setStyleSheet("background-color: lightpink; color: black;")

        self.manual_stop_btn = QPushButton('컨베이어 정지', self)
        self.manual_stop_btn.setGeometry(760, 450, 150, 40)  # 위치 (x, y)와 크기 (width, height) 설정
        self.manual_stop_btn.setStyleSheet("background-color: lightyellow; color: black;")

        # 작업 선택 표시 라벨
        self.selected_job_label = QLabel('선택된 작업: 없음', self)
        self.selected_job_label.setGeometry(950, 200, 200, 40)  # 위치 (x, y)와 크기 (width, height) 설정
        self.selected_job_label.setStyleSheet("border: 1px solid black; padding: 5px; background-color: #f0f0f0;")

        # 타이머 설정
        self.timer = QTimer(self)
        self.timer.timeout.connect(self.update_time)
        self.time_seconds = 0

        # ROS2 노드로부터 상태 업데이트를 위한 타이머 설정
        self.ros_timer = QTimer(self)
        self.ros_timer.timeout.connect(self.gui_node.spin_once)
        self.ros_timer.start(100)

        self.job_selected = False

        self.setWindowTitle('로벌 작업 제어 시스템')
        self.setGeometry(100, 100, 1200, 600)
        self.show()

    def show_job_list(self):
        # 작업 목록을 보여주는 파이어뉴차 생성
        job_dialog = QDialog(self)
        job_dialog.setWindowTitle("작업 목록 선택")
        job_dialog.setFixedSize(200, 200)

        job_layout = QVBoxLayout()

        # Job 버튼들 생성
        job_buttons_layout = QVBoxLayout()
        for job_name in ["Job 1", "Job 2", "Job 3"]:
            job_button = QPushButton(job_name, job_dialog)
            job_button.clicked.connect(lambda checked, name=job_name: self.select_job(name, job_dialog))
            job_buttons_layout.addWidget(job_button)

        job_layout.addLayout(job_buttons_layout)

        close_btn = QPushButton("닫기", job_dialog)
        close_btn.clicked.connect(job_dialog.accept)
        job_layout.addWidget(close_btn)

        job_dialog.setLayout(job_layout)
        job_dialog.exec_()

    def select_job(self, job_name, dialog):
        # 선택된 작업을 라벨에 표시
        self.selected_job_label.setText(f'선택된 작업: {job_name}')
        self.job_selected = True
        dialog.accept()

    def start_timer(self):
        # 타이머 시작 (작업이 선택되었는 경우에만)
        if self.job_selected:
            self.timer.start(1000)

    def stop_timer(self):
        # 타이머 정지
        self.timer.stop()
        # reset 버튼을 제어한 모든의 버튼 비활성화
        self.start_btn.setDisabled(True)
        self.pause_btn.setDisabled(True)
        self.resume_btn.setDisabled(True)
        self.data_collect_btn.setDisabled(True)
        self.job_list_btn.setDisabled(True)
        self.manual_control_btn.setDisabled(True)
        self.manual_stop_btn.setDisabled(True)

    def pause_timer(self):
        # 타이머 일시 정지
        self.timer.stop()

    def resume_timer(self):
        # 타이머 재개 (작업이 선택되었는 경우에만)
        if self.job_selected:
            self.timer.start(1000)

    def reset_timer(self):
        # 타이머 리셋 및 작업 선택 해제
        self.timer.stop()
        self.time_seconds = 0
        self.time_label.setText('00:00')
        self.job_selected = False
        self.selected_job_label.setText('선택된 작업: 없음')
        # 모든 버튼 다시 활성화
        self.start_btn.setDisabled(False)
        self.pause_btn.setDisabled(False)
        self.resume_btn.setDisabled(False)
        self.data_collect_btn.setDisabled(False)
        self.job_list_btn.setDisabled(False)
        self.manual_control_btn.setDisabled(False)
        self.manual_stop_btn.setDisabled(False)

    def update_time(self):
        # 타이머 업데이트
        self.time_seconds += 1
        minutes = self.time_seconds // 60
        seconds = self.time_seconds % 60
        self.time_label.setText(f'{minutes:02}:{seconds:02}')

    def update_robot_status(self, status):
        # 로벌 상태 업데이트
        self.robot_status_label.setText(status)

    def update_camera_view(self, img_msg):
        # ROS2 노드에서 수신한 이미지를 QLabel에 표시
        img_data = np.frombuffer(img_msg.data, dtype=np.uint8)
        qt_image = QImage(img_data, img_msg.width, img_msg.height, QImage.Format_RGB888)
        pixmap = QPixmap.fromImage(qt_image)
        self.world_view_label.setPixmap(pixmap)

    def move_conveyor(self):
        # Conveyer 액션 클라이언트 목표 전송
        goal_msg = Conveyor.Goal()
        goal_msg.goal = 8000  # 예시로 11000의 goal을 전송
        self.gui_node.send_conveyor_goal(goal_msg)


class GUINode(Node):
    def __init__(self):
        super().__init__('gui_node')
        self.publisher_ = self.create_publisher(Button, 'button_topic', 10)
        self.subscription = self.create_subscription(RobotStatus, 'robot_status_topic', self.robot_status_callback, 10)
        self.image_subscription = self.create_subscription(Image, 'world_video', self.image_callback,
                                                           10)  # 이미지 수신 토픽 구독
        self.action_client = ActionClient(self, Conveyor, 'conveyor_action')

    def image_callback(self, msg):
        # 수신한 이미지 데이터를 처리하고 GUI 업데이트
        if hasattr(self, 'robot_control_ui'):
            self.robot_control_ui.update_camera_view(msg)

    def publish_button_press(self, button_name):
        msg = Button()
        msg.button = button_name
        self.publisher_.publish(msg)
        self.get_logger().info(f'Published button press: {button_name}')

    def robot_status_callback(self, msg):
        # 로벌 상태 업데이트 메시지 수신 시 처리
        self.get_logger().info(f'Received robot status: {msg.robot_status}')
        if hasattr(self, 'robot_control_ui'):
            self.robot_control_ui.update_robot_status(msg.robot_status)

    def send_conveyor_goal(self, goal_msg):
        # Conveyer 액션 클라이언트 목표 전송 및 결과 핸들러 설정
        self.get_logger().info('Sending conveyor goal...')
        send_goal_future = self.action_client.send_goal_async(goal_msg, feedback_callback=self.feedback_callback)
        send_goal_future.add_done_callback(self.goal_response_callback)

    def goal_response_callback(self, future):
        goal_handle = future.result()
        if not goal_handle.accepted:
            self.get_logger().info('Goal rejected')
            return

    def update_camera_view(self, img_msg):
        # ROS2 노드에서 수신한 이미지를 QLabel에 표시
        img_data = np.frombuffer(img_msg.data, dtype=np.uint8)
        qt_image = QImage(img_data, img_msg.width, img_msg.height, QImage.Format_RGB888)
        pixmap = QPixmap.fromImage(qt_image)
        self.world_view_label.setPixmap(pixmap)

    def move_conveyor(self):
        # Conveyer 액션 클라이언트 목표 전송
        goal_msg = Conveyor.Goal()
        goal_msg.goal = 5000  # 예시로 10000의 goal을 전송
        self.gui_node.send_conveyor_goal(goal_msg)


class GUINode(Node):
    def __init__(self):
        super().__init__('gui_node')
        self.callback_group = ReentrantCallbackGroup()

        self.publisher_ = self.create_publisher(Button, 'button_topic', 10)
        self.subscription = self.create_subscription(RobotStatus, 'robot_status_topic', self.robot_status_callback, 10,
                                                     callback_group=self.callback_group)
        self.image_subscription = self.create_subscription(Image, 'world_video', self.image_callback, 10,
                                                           callback_group=self.callback_group)  # 이미지 수신 토픽 구독
        self.action_client = ActionClient(self, Conveyor, 'conveyor_action', callback_group=self.callback_group)
        self.total = 0

    def image_callback(self, msg):
        # 수신한 이미지 데이터를 처리하고 GUI 업데이트
        if hasattr(self, 'robot_control_ui'):
            self.robot_control_ui.update_camera_view(msg)

    def publish_button_press(self, button_name):
        msg = Button()
        msg.button = button_name
        self.publisher_.publish(msg)
        self.get_logger().info(f'Published button press: {button_name}')

    def robot_status_callback(self, msg):
        # 로봇 상태 업데이트 메시지 수신 시 처리
        self.get_logger().info(f'Received robot status: {msg.robot_status}')
        if hasattr(self, 'robot_control_ui'):
            self.robot_control_ui.update_robot_status(msg.robot_status)

    # Conveyor Action
    def send_conveyor_goal(self, goal_msg):
        # Conveyer 액션 클라이언트 목표 전송 및 결과 핸들러 설정
        self.get_logger().info('Sending conveyor goal...')
        send_goal_future = self.action_client.send_goal_async(goal_msg, feedback_callback=self.feedback_callback)
        send_goal_future.add_done_callback(self.goal_response_callback)

    def goal_response_callback(self, future):
        goal_handle = future.result()
        if not goal_handle.accepted:
            self.get_logger().info('Goal rejected')
            return
        self.get_logger().info('Goal accepted')
        get_result_future = goal_handle.get_result_async()
        get_result_future.add_done_callback(self.get_result_callback)

    def get_result_callback(self, future):
        result = future.result().result
        self.get_logger().info(f'Conveyor action finished: {result.cfinished}')

    def feedback_callback(self, feedback_msg):
        feedback = feedback_msg.feedback
        self.total += int(f"{feedback.cfeedback}")
        self.get_logger().info(f'Conveyor action feedback: {self.total}')

    def spin_once(self):
        rclpy.spin_once(self, timeout_sec=0.1)


def main(args=None):
    rclpy.init(args=args)
    gui_node = GUINode()

    app = QApplication(sys.argv)
    stacked_widget = QStackedWidget()

    login_ui = LoginUI(stacked_widget)
    robot_control_ui = RobotControlUI(gui_node)

    stacked_widget.addWidget(login_ui)
    stacked_widget.addWidget(robot_control_ui)

    gui_node.robot_control_ui = robot_control_ui  # 연결을 위해 추가

    stacked_widget.setCurrentIndex(0)
    stacked_widget.setWindowTitle('로봇 제어 시스템')
    stacked_widget.setGeometry(100, 100, 1200, 600)
    stacked_widget.show()

    executor = MultiThreadedExecutor()
    executor.add_node(gui_node)

    try:
        sys.exit(app.exec_())
    except SystemExit:
        pass
    finally:
        rclpy.shutdown()


if __name__ == '__main__':
    main()