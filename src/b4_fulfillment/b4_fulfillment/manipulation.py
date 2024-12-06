#!/usr/bin/env python
# Set linear and angular values of Turtlesim's speed and turning.

import os
import select
import sys
import getkey

import rclpy  # Needed to create a ROS node
from geometry_msgs.msg import Twist  # Message that moves base
# from torch.distributed.tensor import empty
from trajectory_msgs.msg import JointTrajectory, JointTrajectoryPoint
from rclpy.action import ActionClient
from control_msgs.action import GripperCommand
from std_msgs.msg import Header
from rclpy.node import Node
from collections import namedtuple
from b4_fulfillment_interfaces.srv import DataCollect
from rclpy.executors import MultiThreadedExecutor
from rclpy.callback_groups import ReentrantCallbackGroup
from sensor_msgs.msg import Image

import math
import time
import cv2
from threading import Event
from cv_bridge import CvBridge
from geometry_msgs.msg import Point  # 좌표 메시지 타입 추가

r1 = 130
r2 = 124
r3 = 126

th1_offset = - math.atan2(0.024, 0.128)
th2_offset = - 0.5 * math.pi - th1_offset

usage = """
Control Your OpenManipulator!
---------------------------
Joint Space Control:
- Joint1 : Increase (Y), Decrease (H)
- Joint2 : Increase (U), Decrease (J)
- Joint3 : Increase (I), Decrease (K)
- Joint4 : Increase (O), Decrease (L)

INIT : (1)

CTRL-C to quit
"""
joint_angle_delta = 0.05  # radian

Coordinate = namedtuple("Coordinate", ["x", "y", "z"])


class ManipulationNode(Node):
    # settings = None
    # if os.name != 'nt':
    # 	settings = termios.tcgetattr(sys.stdin)

    def __init__(self):
        super().__init__('ManipulationNode')
        key_value = ''

        self.cmd_vel = self.create_publisher(Twist, '/cmd_vel', 10)
        self.joint_pub = self.create_publisher(JointTrajectory, '/arm_controller/joint_trajectory', 10)
        self.gripper_action_client = ActionClient(self, GripperCommand, 'gripper_controller/gripper_cmd')

        self.gripper_state = 0

        # self.timer = self.create_timer(1.0, self.timer_callback)

        # Twist is geometry_msgs for linear and angular velocity
        self.move_cmd = Twist()
        # Linear speed in x in meters/second is + (forward) or
        #    - (backwards)
        self.move_cmd.linear.x = 1.3  # Modify this value to change speed
        # Turn at 0 radians/s
        self.move_cmd.angular.z = 0.8
        # Modify this value to cause rotation rad/s

        self.trajectory_msg = JointTrajectory()

        # # 박스 잡는 위치 테스트용
        # self.send_joint_pose_goal(200, 0, 40, r1, r2, r3)

        # 그리퍼 호출
        # # 집게 열기
        # open = self.send_gripper_goal('open')
        # # 집게 닫기
        # close = self.send_gripper_goal('close')

        self.callback_group = ReentrantCallbackGroup()

        self.bridge = CvBridge()
        # 실시간 이미지
        self.cv_image = None
        self.image_event = Event()

        # 자동 데이터 수집 서비스 서버 생성
        _ = self.create_service(
            DataCollect,
            'data_collect_service',
            self.handle_data_collect_service_request,
            callback_group=self.callback_group
        )

        # 이미지 토픽 서브스크라이버 생성
        _ = self.create_subscription(
            Image,  # 메시지 타입
            'gripper_images',  # 토픽 이름 (퍼블리셔와 일치)
            self.image_callback,  # 콜백 함수
            10  # QoS 프로파일
        )

        # 이미지 토픽 서브스크라이버 생성
        _ = self.create_subscription(
            Point,  # 메시지 타입
            'center_point',  # 토픽 이름 (퍼블리셔와 일치)
            self.box_data_callback,  # 콜백 함수
            10  # QoS 프로파일
        )

        self.start_job1()

    def image_callback(self, msg):
        # 메시지를 OpenCV 이미지로 변환
        try:
            self.cv_image = self.bridge.imgmsg_to_cv2(msg, desired_encoding='bgr8')
            self.image_event.set()  # 이미지 수신 이벤트 발생
            self.get_logger().info('Image received')

        except Exception as e:
            self.get_logger().error(f"Failed to process image: {e}")


    def box_data_callback(self, msg):
        # geometry_msgs.msg.Point(x=244.0, y=385.0, z=0.0)

        camera_x = 320  # 카메라 중앙 x
        camera_y = 240  # 카메라 중앙 y

        # 중심점 계산
        dx, dy, dz = self.calculate_camera_movement(camera_x, camera_y, msg)
        # TODO: 계산 된 값 매니풀레이터에 맞게 조정 필요

        self.set_center_pose(dx, dy, dz)

    def start_job1(self):
        box = Coordinate(x=150, y=45, z=130)
        self.send_joint_pose_goal(box.x, box.y, box.z, r1, r2, r3)

    def start_job2(self):
        pass

    def start_job3(self):
        pass

    def calculate_camera_movement(self, camera_x, camera_y, msg):
        # msg로 받은 중심점 좌표
        center_x = msg.x
        center_y = msg.y
        center_z = msg.z  # z 값도 필요한 경우

        # 카메라의 중심 (camera_x, camera_y)와 주어진 중심점 (center_x, center_y) 간의 차이 계산
        dx = center_x - camera_x  # x축 이동량
        dy = center_y - camera_y  # y축 이동량

        # z 값은 필요에 따라 추가적으로 설정할 수 있습니다
        dz = center_z  # 카메라의 깊이 정보가 필요하다면, 적절한 값으로 설정

        return dx, dy, dz

    def set_center_pose(self, x, y, z):
        print(f"x: {x}, y: {y}, z: {z} 만큼 움직여 중앙 정렬 중...")
        self.send_joint_pose_goal(x, y, z, r1, r2, r3)
        time.sleep(2)

    def handle_data_collect_service_request(self, request, response):
        """
        # Request
        bool start
        ---
        # Response
        bool success

        """
        self.get_logger().info(f"자동 데이터 수집 시작: {request.start}")

        # 자동 학습 기능
        if self.auto_data_collection():
            response.success = True
        else:
            response.success = False

        return response


    def auto_data_collection(self):
        coordinates = [
            Coordinate(x=150, y=45, z=130),
            Coordinate(x=160, y=50, z=180),
            Coordinate(x=205, y=45, z=130),
            Coordinate(x=210, y=-55, z=130),
            Coordinate(x=155, y=-40, z=130)
        ]

        index = 1

        # cap = cv2.VideoCapture('/dev/video0')
        # cap.set(cv2.CAP_PROP_FRAME_WIDTH, 1280)  # 가로 해상도 설정
        # cap.set(cv2.CAP_PROP_FRAME_HEIGHT, 720)  # 세로 해상도 설정

        time.sleep(2)

        for _ in range(30):
            for box in coordinates:
                self.send_joint_pose_goal(box.x, box.y, box.z, r1, r2, r3)
                time.sleep(6)

                self.get_image_data(index=index)
                time.sleep(3)
                index += 1

        return True

    def get_image_data(self, index):
        # 이미지 저장을 위한 디렉토리 생성
        save_directory = "img_capture"
        os.makedirs(save_directory, exist_ok=True)

        # 이미지 대기
        if not self.image_event.wait(timeout=5):  # 최대 5초 대기
            self.get_logger().error("Timeout waiting for image")
            return

        # 이미지 저장
        if self.cv_image is not None:
            file_name = f"{save_directory}/box_{index}.jpg"
            cv2.imwrite(file_name, self.cv_image)
            self.get_logger().info(f"Image saved: {file_name}")
        else:
            self.get_logger().error("No image data received")

        # 이벤트 초기화
        self.image_event.clear()

    def send_gripper_goal(self, action):
        position = -0.015
        if action == 'open':
            position = 0.025

        goal = GripperCommand.Goal()
        goal.command.position = position
        goal.command.max_effort = -1.0

        if not self.gripper_action_client.wait_for_server(timeout_sec=1.0):
            self.get_logger().error("Gripper action server not available!")
            return None
        print("보냄")
        return self.gripper_action_client.send_goal_async(goal)


    def send_joint_pose_goal(self, x, y, z, r1, r2, r3):
        print(f"x: {x}, y: {y}, z: {z} 로 이동")
        J0, J1, J2, J3, Sxy, sr1, sr2, sr3, St, Rt = self.solv_robot_arm2(x, y, z, r1, r2, r3)

        current_time = self.get_clock().now()
        self.trajectory_msg.header = Header()
        #		self.trajectory_msg.header.stamp = current_time.to_msg()
        self.trajectory_msg.header.frame_id = ''
        self.trajectory_msg.joint_names = ['joint1', 'joint2', 'joint3', 'joint4']

        point = JointTrajectoryPoint()
        # point.positions = [0.003, math.pi / 4.0, -0.489, 2.041]
        #		point.positions = [0.0] * 4
        point.positions = [Sxy, sr1 + th1_offset, sr2 + th2_offset, sr3]
        point.velocities = [0.0] * 4
        point.time_from_start.sec = 3
        point.time_from_start.nanosec = 0

        self.trajectory_msg.points = [point]
        self.joint_pub.publish(self.trajectory_msg)


    # author : karl.kwon (mrthinks@gmail.com)
    # r1 : distance J0 to J1
    # r2 : distance J1 to J2
    # r3 : distance J0 to J2
    def solv2(self, r1, r2, r3):
        d1 = (r3 ** 2 - r2 ** 2 + r1 ** 2) / (2 * r3)
        d2 = (r3 ** 2 + r2 ** 2 - r1 ** 2) / (2 * r3)

        s1 = math.acos(d1 / r1)
        s2 = math.acos(d2 / r2)

        return s1, s2

    # author : karl.kwon (mrthinks@gmail.com)
    # x, y, z : relational position from J0 (joint 0)
    # r1 : distance J0 to J1
    # r2 : distance J1 to J2
    # r3 : distance J2 to J3
    # sr1 : angle between z-axis to J0->J1
    # sr2 : angle between J0->J1 to J1->J2
    # sr3 : angle between J1->J2 to J2->J3 (maybe always parallel)
    def solv_robot_arm2(self, x, y, z, r1, r2, r3):
        Rt = math.sqrt(x ** 2 + y ** 2 + z ** 2)
        Rxy = math.sqrt(x ** 2 + y ** 2)
        St = math.asin(z / Rt)
        #   Sxy = math.acos(x / Rxy)
        Sxy = math.atan2(y, x)

        s1, s2 = self.solv2(r1, r2, Rt)

        sr1 = math.pi / 2 - (s1 + St)
        sr2 = s1 + s2
        sr2_ = sr1 + sr2
        sr3 = math.pi - sr2_

        J0 = (0, 0, 0)
        J1 = (J0[0] + r1 * math.sin(sr1) * math.cos(Sxy),
              J0[1] + r1 * math.sin(sr1) * math.sin(Sxy),
              J0[2] + r1 * math.cos(sr1))
        J2 = (J1[0] + r2 * math.sin(sr1 + sr2) * math.cos(Sxy),
              J1[1] + r2 * math.sin(sr1 + sr2) * math.sin(Sxy),
              J1[2] + r2 * math.cos(sr1 + sr2))
        J3 = (J2[0] + r3 * math.sin(sr1 + sr2 + sr3) * math.cos(Sxy),
              J2[1] + r3 * math.sin(sr1 + sr2 + sr3) * math.sin(Sxy),
              J2[2] + r3 * math.cos(sr1 + sr2 + sr3))

        return J0, J1, J2, J3, Sxy, sr1, sr2, sr3, St, Rt


def manipulation_controller(node=None):
    if node is None:
        print("node가 없습니다.")
        return
    key_value = getkey.getkey()

    if key_value == '1':
        node.trajectory_msg.points[0].positions = [0.0] * 4
        node.joint_pub.publish(node.trajectory_msg)
        print('joint1 +')
    elif key_value == 'y':
        node.trajectory_msg.points[0].positions[0] += joint_angle_delta
        node.joint_pub.publish(node.trajectory_msg)
        print('joint1 +')
    elif key_value == 'h':
        node.trajectory_msg.points[0].positions[0] -= joint_angle_delta
        node.joint_pub.publish(node.trajectory_msg)
        print('joint1 -')
    elif key_value == 'u':
        node.trajectory_msg.points[0].positions[1] += joint_angle_delta
        node.joint_pub.publish(node.trajectory_msg)
        print('joint2 +')
    elif key_value == 'j':
        node.trajectory_msg.points[0].positions[1] -= joint_angle_delta
        node.joint_pub.publish(node.trajectory_msg)
        print('joint2 -')
    elif key_value == 'i':
        node.trajectory_msg.points[0].positions[2] += joint_angle_delta
        node.joint_pub.publish(node.trajectory_msg)
        print('joint3 +')
    elif key_value == 'k':
        node.trajectory_msg.points[0].positions[2] -= joint_angle_delta
        node.joint_pub.publish(node.trajectory_msg)
        print('joint3 -')
    elif key_value == 'o':
        node.trajectory_msg.points[0].positions[3] += joint_angle_delta
        node.joint_pub.publish(node.trajectory_msg)
        print('joint4 +')
    elif key_value == 'l':
        node.trajectory_msg.points[0].positions[3] -= joint_angle_delta
        node.joint_pub.publish(node.trajectory_msg)
        print('joint4 -')
    elif key_value == 'q':
        return


def main():
    try:
        rclpy.init()
    except Exception as e:
        print(f"Failed to initialize rclpy: {e}")
        return

    try:
        node = ManipulationNode()
    except Exception as e:
        print(f"Failed to create ManipulationNode: {e}")
        return

    try:
        executor = MultiThreadedExecutor()
        executor.add_node(node)
        executor.spin()

    except Exception as e:
        print(f"Exception in execution: {e}")
    finally:
        print("Shutting down...")
        # 종료 처리
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()