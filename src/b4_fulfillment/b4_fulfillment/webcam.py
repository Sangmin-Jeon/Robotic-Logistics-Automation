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

desired_aruco_dictionary = "DICT_5X5_100"

# The different ArUco dictionaries built into the OpenCV library.
ARUCO_DICT = {
    "DICT_4X4_50": cv2.aruco.DICT_4X4_50,
    "DICT_4X4_100": cv2.aruco.DICT_4X4_100,
    "DICT_4X4_250": cv2.aruco.DICT_4X4_250,
    "DICT_4X4_1000": cv2.aruco.DICT_4X4_1000,
    "DICT_5X5_50": cv2.aruco.DICT_5X5_50,
    "DICT_5X5_100": cv2.aruco.DICT_5X5_100,
    "DICT_5X5_250": cv2.aruco.DICT_5X5_250,
    "DICT_5X5_1000": cv2.aruco.DICT_5X5_1000,
    "DICT_6X6_50": cv2.aruco.DICT_6X6_50,
    "DICT_6X6_100": cv2.aruco.DICT_6X6_100,
    "DICT_6X6_250": cv2.aruco.DICT_6X6_250,
    "DICT_6X6_1000": cv2.aruco.DICT_6X6_1000,
    "DICT_7X7_50": cv2.aruco.DICT_7X7_50,
    "DICT_7X7_100": cv2.aruco.DICT_7X7_100,
    "DICT_7X7_250": cv2.aruco.DICT_7X7_250,
    "DICT_7X7_1000": cv2.aruco.DICT_7X7_1000,
    "DICT_ARUCO_ORIGINAL": cv2.aruco.DICT_ARUCO_ORIGINAL
}

# 카메라 매트릭스 및 왜곡 계수
mtx = np.array([[1.38377618e+03, 0.00000000e+00, 7.12946953e+02],
                [0.00000000e+00, 1.38806900e+03, 4.39115328e+02],
                [0.00000000e+00, 0.00000000e+00, 1.00000000e+00]])
dist = np.array([[0.05875061, 0.14238942, 0.00755354, -0.00307189, -0.29956771]])

# ArUco 마커 크기
aruco_size = 10.8  # cm (10.8cm x 10.8cm)

# ArUco 마커 ID 정의
aruco_22_id = 22
aruco_24_id = 24
aruco_20_id = 20

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
        # Check that we have a valid ArUco marker
        if ARUCO_DICT.get(desired_aruco_dictionary, None) is None:
            print("[INFO] ArUCo tag of '{}' is not supported".format(args["type"]))
            sys.exit(0)

        # Load the ArUco dictionary
        print("[INFO] detecting '{}' markers...".format(
            desired_aruco_dictionary))
        # this_aruco_dictionary = cv2.aruco.Dictionary_get(ARUCO_DICT[desired_aruco_dictionary])
        # this_aruco_parameters = cv2.aruco.DetectorParameters_create()

        # this_aruco_dictionary = cv2.aruco.getPredefinedDictionary(ARUCO_DICT[desired_aruco_dictionary])
        # this_aruco_parameters = cv2.aruco.DetectorParameters_create()

        dictionary = cv2.aruco.getPredefinedDictionary(cv2.aruco.DICT_5X5_100)
        parameters = cv2.aruco.DetectorParameters()
        self.detector = cv2.aruco.ArucoDetector(dictionary, parameters)

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

        while not self.coordinate_srv_client.wait_for_service(timeout_sec=1.0):
            self.get_logger().info('coordinate_service가 열리지 않았습니다...')
        self.coordinate_transformer()

    def process_frame(self):
        # 카메라에서 프레임을 읽음
        ret, frame = self.cap.read()
        if not ret:
            self.get_logger().error('Failed to capture frame')
            return

        # 이미지 크기 출력 (height, width, channels)
        # height, width, channels = frame.shape
        # print(f"Frame size: {width}x{height}, Channels: {channels}")

        h, w = frame.shape[:2]
        new_camera_mtx, roi = cv2.getOptimalNewCameraMatrix(mtx, dist, (w, h), 1, (w, h))
        undistorted_image = cv2.undistort(frame, mtx, dist, None, new_camera_mtx)

        # ArUco 사전 설정 (DICT_5X5_100)
        aruco_dict = cv2.aruco.getPredefinedDictionary(cv2.aruco.DICT_5X5_100)
        aruco_params = cv2.aruco.DetectorParameters()

        # ArUco 마커 감지
        corners, ids, _ = cv2.aruco.detectMarkers(undistorted_image, aruco_dict, parameters=aruco_params)

        if ids is not None and len(ids) >= 3:
            # 모든 마커의 중심 좌표를 계산하여 저장
            marker_positions = {}
            rvecs, tvecs, _ = cv2.aruco.estimatePoseSingleMarkers(corners, aruco_size, mtx, dist)
            for i, tvec in zip(ids, tvecs):
                marker_positions[i[0]] = tvec[0]
                print(f"ArUco 마커 ID {i[0]}: 3D 위치 {tvec[0]}")

            # 마커 22, 24, 20의 상대적 위치를 기반으로 계산
            if aruco_22_id in marker_positions and aruco_24_id in marker_positions and aruco_20_id in marker_positions:
                p22 = marker_positions[aruco_22_id]
                p24 = marker_positions[aruco_24_id]
                p20 = marker_positions[aruco_20_id]

                # 22번과 24번의 중심점을 연결하여 x축을 정의하고 원점을 설정 (x축과 20번 마커가 만나는 지점)
                x_axis_vec = p24 - p22
                x_axis_unit_vec = x_axis_vec / np.linalg.norm(x_axis_vec)

                # 마커 20이 24에 대해 수직이 되도록 보정 (24번을 기준으로 수직 벡터 계산)
                p24_to_p20_vec = p20 - p24
                z_axis_vec = np.cross(x_axis_unit_vec, p24_to_p20_vec)
                z_axis_unit_vec = z_axis_vec / np.linalg.norm(z_axis_vec)
                y_axis_vec = np.cross(z_axis_unit_vec, x_axis_unit_vec)
                y_axis_unit_vec = y_axis_vec / np.linalg.norm(y_axis_vec)

                # 새로운 원점을 p24로 설정하고, 상대 좌표로 변환
                origin = p24
                p22_relative = np.dot((p22 - origin), [x_axis_unit_vec, y_axis_unit_vec, z_axis_unit_vec])
                p24_relative = np.dot((p24 - origin), [x_axis_unit_vec, y_axis_unit_vec, z_axis_unit_vec])
                p20_relative = np.dot((p20 - origin), [x_axis_unit_vec, y_axis_unit_vec, z_axis_unit_vec])

                # 마커 좌표 출력
                print(f"마커 22번의 상대 좌표: {p22_relative}")
                print(f"마커 24번의 상대 좌표: {p24_relative}")
                print(f"마커 20번의 상대 좌표: {p20_relative}")

                # 마커 24와 20이 수직임을 고려한 거리 계산
                d22_24 = np.linalg.norm(p24_relative - p22_relative)
                d24_20 = np.linalg.norm(p20_relative - p24_relative)
                d20_22 = np.linalg.norm(p20_relative - p22_relative)

                print(f"22번과 24번 사이의 거리: {d22_24:.2f} cm")
                print(f"24번과 20번 사이의 거리 (수직): {d24_20:.2f} cm")
                print(f"20번과 22번 사이의 거리: {d20_22:.2f} cm")

                # 원점과 마커를 시각적으로 표시
                origin_2d = tuple(origin[:2].astype(int))
                p22_2d = tuple(p22[:2].astype(int))
                p24_2d = tuple(p24[:2].astype(int))
                p20_2d = tuple(p20[:2].astype(int))

                # 원점 표시
                cv2.circle(undistorted_image, origin_2d, 5, (0, 0, 255), -1)  # 빨간색 점으로 원점 표시

                # 각 마커 연결 선 그리기
                cv2.line(undistorted_image, p22_2d, p24_2d, (255, 0, 0), 2)  # 파란색 선으로 22번과 24번 연결
                cv2.line(undistorted_image, p24_2d, p20_2d, (0, 255, 0), 2)  # 초록색 선으로 24번과 20번 연결
                cv2.line(undistorted_image, p20_2d, p22_2d, (255, 255, 0), 2)  # 노란색 선으로 20번과 22번 연결

            else:
                print("필요한 모든 마커가 감지되지 않았습니다.")
        else:
            print("ArUco 마커가 충분히 감지되지 않았습니다.")


                # (주석 처리된 부분) YOLO 모델로 예측 수행
                # results = self.model(frame, stream=True)

                # (주석 처리된 부분) 결과 프레임에 예측 정보를 표시 (예: 객체 탐지 결과)
                # frame = results.render()[0]  # 첫 번째 (단일 프레임) 결과를 가져오기

        # 이미지를 ROS 메시지로 변환하여 퍼블리시
        img_msg = self.bridge.cv2_to_imgmsg(undistorted_image, encoding='bgr8')
        self.video_publisher.publish(img_msg)

        # 화면에 결과 프레임 출력
        cv2.imshow('Camera Feed', undistorted_image)

        # 'q' 키를 눌러 프로그램 종료 시 데이터 저장 및 자원 해제
        if cv2.waitKey(1) & 0xFF == ord('q'):
            self.cap.release()  # 카메라 장치 해제
            cv2.destroyAllWindows()  # OpenCV 창 닫기



    # 좌표 변환
    def coordinate_transformer(self):
        # JSON 데이터 준비
        json_data = {
            "init_pose": {
                "x": 0.0,
                "y": 0.0,
                "z": 0.0
            },
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
