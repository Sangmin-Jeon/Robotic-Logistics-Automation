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
from b4_fulfillment_interfaces.srv import DoGrip
from b4_fulfillment_interfaces.srv import ConvStart

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

        # 이미지 토픽 서브스크라이버
        _ = self.create_subscription(
            Image,  # 메시지 타입
            'gripper_images',
            self.image_callback,
            10,
            callback_group = self.callback_group
        )

        # 박스 중심점 토픽 서브스크라이버
        _ = self.create_subscription(
            Point,  # 메시지 타입
            'center_point',
            self.box_data_callback,
            10,
            callback_group=self.callback_group
        )

        # 시작 트리거 서비스
        _ = self.create_service(
            DoGrip,
            'do_grip_service',
            self.do_grip_callback,
            callback_group=self.callback_group
        )

        self.move_conv_client = self.create_client(
            ConvStart,
            'move_conv_service',
            callback_group=self.callback_group
        )


        # 메니풀레이션의 상태 좌표 저장
        self.manipulation_pose = None
        self.is_center_pose = False
        self.move_finished_first_pose = False
        # self.move_finished_first_pose = self.start_job1()

        self.capture_image = None

    def do_grip_callback(self, request, response):
        self.get_logger().info(f"manipulation going up: {request.grip}")

        if request.grip:
            self.move_finished_first_pose = self.start_job1()
            response.success = True
        else:
            response.success = False

        return response



    def image_callback(self, msg):
        # 메시지를 OpenCV 이미지로 변환
        try:
            self.cv_image = self.bridge.imgmsg_to_cv2(msg, desired_encoding='bgr8')
            self.image_event.set()  # 이미지 수신 이벤트 발생
            if self.is_center_pose and self.capture_image is None:
                self.capture_image = self.cv_image
                file_path = "/home/rokey10/capture_image.jpg"  # 저장할 파일 경로와 이름
                cv2.imwrite(file_path, self.capture_image)
                self.get_logger().info(f"이미지가 {file_path}에 저장되었습니다.")

            # self.get_logger().info('Image received')

        except Exception as e:
            self.get_logger().error(f"Failed to process image: {e}")


    def box_data_callback(self, msg):
        # geometry_msgs.msg.Point(x=244.0, y=385.0, z=0.0)

        if self.move_finished_first_pose and not self.is_center_pose:
            camera_x = 320  # 카메라 중앙 x
            camera_y = 240  # 카메라 중앙 y
            # 중심점 맞추는 계산 시작
            dx, dy, dz = self.calculate_camera_movement(camera_x, camera_y, msg)
            self.set_center_pose(dx, dy, dz)

    def start_job1(self):
        self.manipulation_pose = Coordinate(x=150, y=45, z=130)

        self.send_joint_pose_goal(
            self.manipulation_pose.x,
            self.manipulation_pose.y,
            self.manipulation_pose.z,
            r1, r2, r3
        )

        self.send_gripper_goal('open')

        time.sleep(2.5)

        return True

    def start_job2(self):
        pass

    def start_job3(self):
        pass

    def calculate_camera_movement(self, camera_x, camera_y, msg):
        # msg로 받은 중심점 좌표
        center_x = msg.x
        center_y = msg.y
        center_z = 130  # z 값도 필요한 경우

        ratio = 0.003

        self.get_logger().info(f"카메라 중앙: ({camera_x}, {camera_y}) , 박스 중앙: ({center_x}, {center_y})")

        # 카메라의 중심과 객체 중심 간의 차이만 계산
        dx = (camera_x - center_x) * ratio  # x축 이동량
        dy = (center_y - camera_y) * ratio  # y축 이동량
        dz = center_z  # z 값 고정

        # 객체의 중심점이 hit 박스 안에 들어오면
        if self.cal_center_in_hit_box(camera_x, camera_y, center_x, center_y) and not self.is_center_pose:
            self.is_center_pose = True
            self.get_box()
            return self.manipulation_pose.x, self.manipulation_pose.y, self.manipulation_pose.z

        # 새로운 목표 위치를 갱신
        self.manipulation_pose = Coordinate(
            self.manipulation_pose.x + dx,
            self.manipulation_pose.y + dy,
            dz
        )

        self.get_logger().info(f"dx: {dx}, dy: {dy}, dz: {dz} 만큼 계산됨 ")

        time.sleep(3)

        return self.manipulation_pose.x, self.manipulation_pose.y, self.manipulation_pose.z

    def set_center_pose(self, x, y, z):
        if not self.is_center_pose:
            self.get_logger().info(f"x: {x}, y: {y}, z: {z} 만큼 움직여 중앙 정렬 중...")
            self.send_joint_pose_goal(x, y, z, r1, r2, r3)
            time.sleep(2)


    def cal_center_in_hit_box(self, camera_x, camera_y, center_x, center_y):
        # 박스 크기 및 중심 좌표 정의
        x_min = 245
        x_max = 395
        y_min = 85
        y_max = 260
        # 박스 중앙: (333.0, 255.0)
        self.get_logger().info(f"Hit 박스 안에 들어옴: {x_min <= center_x <= x_max and y_min <= center_y <= y_max}")
        return x_min <= center_x <= x_max and y_min <= center_y <= y_max

    def get_box(self):
        if self.is_center_pose:
            self.get_logger().info(f"박스 잡으러 내려가는 중")
            time.sleep(2.5)
            self.send_joint_pose_goal(
                self.manipulation_pose.x + 40,
                self.manipulation_pose.y + 20,
                130,
                r1, r2, r3
            )

            time.sleep(2.5)

            self.send_joint_pose_goal(
                self.manipulation_pose.x + 60,
                self.manipulation_pose.y + 15,
                50,
                r1, r2, r3
            )

            time.sleep(3)

            self.get_logger().info(f"박스 잡았다.")

            _ = self.send_gripper_goal('close')

            time.sleep(2.5)

            self.send_joint_pose_goal(
                self.manipulation_pose.x,
                self.manipulation_pose.y,
                130,
                r1, r2, r3
            )

            time.sleep(2.5)

            self.get_logger().info(f"컨베이어로 이동 중 ...")

            self.send_joint_pose_convayer_goal(
                self.manipulation_pose.x + 15,
                self.manipulation_pose.y,
                150,
                r1, r2, r3
            )

            time.sleep(4)

            _ = self.send_gripper_goal('open')

            time.sleep(3)

            self.send_joint_pose_convayer_goal(
                    self.manipulation_pose.x,
                    self.manipulation_pose.y,
                    160,
                    r1, r2, r3
                    )

            time.sleep(3)

            self.send_joint_pose_init_goal(self.manipulation_pose.x, self.manipulation_pose.y, 160, r1, r2, r3)

            time.sleep(3)

            self.finished_put_box_in_conveyer()


    def finished_put_box_in_conveyer(self):
        while not self.move_conv_client.wait_for_service(timeout_sec=1.0):
            self.get_logger().info('서비스 대기 중...')

        request = ConvStart.Request()
        request.convstart = True
        future = self.move_conv_client.call_async(request)
        future.add_done_callback(self.move_conv_callback)

    def move_conv_callback(self, future):
        try:
            self.get_logger().info(f"컨베이어 작동 중: {future}")
        except Exception as e:
            self.get_logger().error(f"서비스 호출 실패: {e}")


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


    # 자동 데이터 수집 기능
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
        point.positions = [
            Sxy,
            sr1 + th1_offset,
            sr2 + th2_offset,
            sr3
        ]
        point.velocities = [0.0] * 4
        point.time_from_start.sec = 3
        point.time_from_start.nanosec = 0

        self.trajectory_msg.points = [point]
        self.joint_pub.publish(self.trajectory_msg)

    def send_joint_pose_convayer_goal(self, x, y, z, r1, r2, r3):
        custom_offset = -math.radians(40)
        print(f"x: {x}, y: {y}, z: {z} 로 이동")
        J0, J1, J2, J3, Sxy, sr1, sr2, sr3, St, Rt = self.solv_robot_arm2(x, y, z, r1, r2, r3)

        # 시계방향 90도 회전 (Sxy 값에 pi/2 추가)
        Sxy -= (math.pi * (10 / 9) / 2)

        current_time = self.get_clock().now()
        self.trajectory_msg.header = Header()
        self.trajectory_msg.header.frame_id = ''
        self.trajectory_msg.joint_names = ['joint1', 'joint2', 'joint3', 'joint4']

        point = JointTrajectoryPoint()
        point.positions = [
            Sxy,
            sr1 + th1_offset + math.radians(20),
            sr2 + th2_offset,
            sr3 + custom_offset
        ]
        point.velocities = [0.0] * 4
        point.time_from_start.sec = 3
        point.time_from_start.nanosec = 0

        self.trajectory_msg.points = [point]
        self.joint_pub.publish(self.trajectory_msg)



    def send_joint_pose_init_goal(self, x, y, z, r1, r2, r3):
        custom_offset = -math.radians(10)
        print(f"x: {x}, y: {y}, z: {z} 로 이동")
        J0, J1, J2, J3, Sxy, sr1, sr2, sr3, St, Rt = self.solv_robot_arm2(x, y, z, r1, r2, r3)

        current_time = self.get_clock().now()
        self.trajectory_msg.header = Header()
        self.trajectory_msg.header.frame_id = ''
        self.trajectory_msg.joint_names = ['joint1', 'joint2', 'joint3', 'joint4']

        point = JointTrajectoryPoint()
        point.positions = [
            Sxy,
            sr1 + th1_offset,
            sr2 + th2_offset,
            sr3 + custom_offset
        ]
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

