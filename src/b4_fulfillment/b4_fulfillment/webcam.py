import json
import cv2
import numpy as np
from cv_bridge import CvBridge
from b4_fulfillment_interfaces.srv import Coordinate
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image
from rclpy.executors import MultiThreadedExecutor
from rclpy.callback_groups import ReentrantCallbackGroup
import threading
import numpy as np

class WebcamNode(Node):
    # def __init__(self, pt_file):
    def __init__(self):
        # 노드 초기화
        super().__init__('WebcamNode')
        # self.pt_file = pt_file
        self.callback_group = ReentrantCallbackGroup()

        # YOLO 모델 로드
        # self.model = YOLO(self.pt_file)
        # self.get_logger().info(f'Loaded model from {self.pt_file}')

        # 웹캠 설정
        self.cap = cv2.VideoCapture(0)  # 기본 웹캠 사용
        if not self.cap.isOpened():
            self.get_logger().error('Could not open video device')

        self.bridge = CvBridge()  # OpenCV 이미지를 ROS 이미지로 변환하기 위한 브릿지 설정

        # 이미지 퍼블리셔 생성
        self.video_publisher = self.create_publisher(
            Image,  # 메시지 타입
            'world_video',  # 토픽 이름
            qos_profile=10,  # QoS 프로파일
            callback_group = self.callback_group
        )

        # 좌표 보내는 서비스 클라이언트
        self.coordinate_srv_client = self.create_client(
            Coordinate,
            'coordinate_service',
            callback_group=self.callback_group
        )

        # OpenCV 창 열기
        # cv2.namedWindow('YOLO Detection')

        # while not self.coordinate_srv_client.wait_for_service(timeout_sec=1.0):
        #     self.get_logger().info('coordinate_service가 열리지 않았습니다...')
        #
        # self.coordinate_transformer()

    def process_frame(self):
        # 카메라에서 프레임을 읽음
        ret, frame = self.cap.read()
        if not ret:
            self.get_logger().error('Failed to capture frame')
            return

        # 이미지 크기 출력 (height, width, channels)
        height, width, channels = frame.shape
        print(f"Frame size: {width}x{height}, Channels: {channels}")

        # (주석 처리된 부분) YOLO 모델로 예측 수행
        # results = self.model(frame, stream=True)

        # (주석 처리된 부분) 결과 프레임에 예측 정보를 표시 (예: 객체 탐지 결과)
        # frame = results.render()[0]  # 첫 번째 (단일 프레임) 결과를 가져오기

        # 이미지를 ROS 메시지로 변환하여 퍼블리시
        img_msg = self.bridge.cv2_to_imgmsg(frame, encoding='bgr8')
        self.video_publisher.publish(img_msg)

        # 화면에 결과 프레임 출력
        cv2.imshow('Camera Feed', frame)

        # 'q' 키를 눌러 프로그램 종료 시 데이터 저장 및 자원 해제
        if cv2.waitKey(1) & 0xFF == ord('q'):
            self.cap.release()  # 카메라 장치 해제
            cv2.destroyAllWindows()  # OpenCV 창 닫기

    # 좌표 변환
    def coordinate_transformer(self):
        # JSON 데이터 준비
        json_data = {
            "location1": {
                "x": 0.0,
                "y": 0.0,
                "z": 0.0
            },
            "location2": {
                "x": 0.0,
                "y": 0.0,
                "z": 0.0
            },
            "location3": {
                "x": 0.0,
                "y": 0.0,
                "z": 0.0
            }
        }

        # 서비스 요청 객체 생성
        request = Coordinate.Request()
        # JSON 데이터를 직렬화하여 요청에 포함
        request.json_data = json.dumps(json_data)  # JSON 직렬화

        # 비동기적으로 서비스 호출
        future = self.coordinate_srv_client.call_async(request)

        # 응답을 처리할 콜백 함수 등록
        future.add_done_callback(self.coordinate_transformer_callback)

    def coordinate_transformer_callback(self, future):
        try:
            # 서비스 호출 결과 확인
            response = future.result()
            if response.success:
                self.get_logger().info(f"Success: {response.message}")
            else:
                self.get_logger().error(f"Failed: {response.message}")
        except Exception as e:
            self.get_logger().error(f"Service call failed: {e}")


# 메인 함수
def main(args=None):
    # ROS 2 initialization
    rclpy.init(args=args)

    # Create executor and node
    executor = MultiThreadedExecutor()
    node = WebcamNode()
    executor.add_node(node)

    try:
        # Run the executor in a separate thread
        ros_thread = threading.Thread(target=executor.spin, daemon=True)
        ros_thread.start()

        # Frame processing loop
        while rclpy.ok():
            node.process_frame()

    except KeyboardInterrupt:
        print("Interrupted by user. Shutting down...")
    finally:
        # Shutdown the node and executor
        executor.destroy_node()
        executor.shutdown()
        rclpy.shutdown()

if __name__ == '__main__':
    main()
