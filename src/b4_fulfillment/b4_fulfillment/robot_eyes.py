import json
import csv
import time
from ultralytics import YOLO
import cv2
import math
import os
import shutil
import sys
import numpy as np
import rclpy  # ROS2 파이썬 클라이언트 라이브러리
from rclpy.node import Node
from sensor_msgs.msg import Image
from std_msgs.msg import String, Float64MultiArray
from cv_bridge import CvBridge  # OpenCV 이미지를 ROS 메시지로 변환
from geometry_msgs.msg import Point  # 좌표 메시지 타입 추가


# ROS2 노드 클래스 정의
class RobotEyes(Node):
    def __init__(self, pt_file):
        super().__init__('RobotEyes')
        self.pt_file = pt_file

        # YOLO 모델 로드, .pt 파일을 사용하여 모델을 초기화
        self.model = YOLO(self.pt_file)
        self.get_logger().info(f'Loaded model from {self.pt_file}')

        self.cap = cv2.VideoCapture('/dev/video0')
        if not self.cap.isOpened():
            self.get_logger().error('Could not open video device')  # 카메라가 열리지 않을 경우 오류 메시지

        self.video_publisher = self.create_publisher(Image, 'gripper_images', 10)  # 비디오 피드 퍼블리셔 추가
        self.center_pub = self.create_publisher(Point, 'center_point', 10) # 중심점 퍼블리셔

        self.bridge = CvBridge()  # OpenCV 이미지를 ROS 이미지로 변환하기 위한 브릿지 설정

        self.frame_sent = False  # 프레임 전송 여부 확인


    # 프레임을 처리하는 함수
    def process_frame(self):
        if self.frame_sent:
            return

        # 카메라에서 프레임을 읽음
        ret, frame = self.cap.read()
        if not ret:
            self.get_logger().error('Failed to capture frame')
            return
        # YOLO 모델로 예측 수행
        results = self.model(frame, stream=True)
        object_count = 0  # 탐지된 물체의 수 초기화
        classNames = ['blue', 'red', 'purple']
        csv_output = []  # 결과를 저장할 CSV 데이터
        confidences = []  # 탐지된 물체의 확률 저장

        # 탐지된 결과 순회
        for r in results:
            for i, box in enumerate(r.boxes):
                # 바운딩 박스 좌표 및 정보 추출
                x1, y1, x2, y2 = map(int, box.xyxy[0])
                confidence = math.ceil((box.conf[0] * 100)) / 100
                cls = int(box.cls[0])
                confidences.append(confidence)

                # 바운딩 박스 그리기 및 텍스트 표시
                cv2.rectangle(frame, (x1, y1), (x2, y2), (0, 0, 255), 2)
                if box.cls is not None and len(box.cls) > 0:
                    cls = int(box.cls[0])
                    if 0 <= cls < len(classNames):  # 클래스 ID가 유효한 경우
                        cv2.putText(frame, f'{classNames[cls]}: {confidence}', (x1, y1 - 10), cv2.FONT_HERSHEY_SIMPLEX,
                                    0.5, (255, 0, 0), 2)
                    else:
                        self.get_logger().warn(f"Invalid class ID {cls}. Check classNames or YOLO model.")
                else:
                    self.get_logger().warn("box.cls is empty or None.")

                # 물체의 중심점 계산 및 표시
                center_x, center_y = (x1 + x2) // 2, (y1 + y2) // 2
                cv2.circle(frame, (center_x, center_y), 5, (0, 255, 255), -1)  # 중심점 표시

                # # 좌표 퍼블리시 (Point 메시지 사용)
                # self.publish_point_coordinates(center_x, center_y)

                # 이전 위치 저장 및 경계선 넘는지 확인
                object_id = f"obj_{i}"
                # current_center = (center_x, center_y)
                cur_point = Point()
                cur_point.x = float(center_x)
                cur_point.y = float(center_y)
                cur_point.z = 0.0
                self.center_pub.publish(cur_point)

                # 현재 위치 저장
                csv_output.append([x1, y1, x2, y2, confidence, cls])
                object_count += 1  # 탐지된 물체 수 증가

        img_msg = self.bridge.cv2_to_imgmsg(frame, encoding='bgr8')
        self.video_publisher.publish(img_msg)

        # 'q' 키를 눌러 프로그램 종료 시 데이터 저장 및 자원 해제
        if cv2.waitKey(1) & 0xFF == ord('q'):
            self.cap.release()  # 카메라 장치 해제

    # 노드 종료 시 자원 해제
    def destroy_node(self):
        self.cap.release()
        super().destroy_node()


# 메인 함수
def main(args=None):
    # ROS2 초기화
    rclpy.init(args=args)

    # .pt 파일 인자 확인
    if len(sys.argv) < 2:
        print("Error: No model (.pt) file provided.")
        return

    # YOLO 모델 파일 경로를 가져와서 노드 생성
    pt_file = sys.argv[1]
    node = RobotEyes(pt_file)

    try:
        # 노드가 실행 중일 때 계속 프레임을 처리
        while rclpy.ok():
            node.process_frame()
    finally:
        # 노드 종료 및 rclpy 종료
        node.destroy_node()
        rclpy.shutdown()


# 프로그램 시작점
if __name__ == '__main__':
    main()