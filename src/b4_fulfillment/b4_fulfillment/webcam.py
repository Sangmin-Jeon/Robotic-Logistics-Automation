import json
import cv2
import numpy as np
import cv2.aruco as aruco
from scipy.spatial.transform import Rotation as R
from collections import namedtuple
from cv_bridge import CvBridge
from b4_fulfillment_interfaces.srv import Coordinate
from b4_fulfillment_interfaces.msg import RobotCoord
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image
from rclpy.executors import MultiThreadedExecutor
from rclpy.callback_groups import ReentrantCallbackGroup
import threading
import numpy as np
import sys

desired_aruco_dictionary = "DICT_5X5_100"
Aruco_Coordinate = namedtuple("Aruco_Coordinate", ["id", "x", "y", "z"])

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
mtx = np.array([[1.36562832e+03, 0.00000000e+00, 7.06645482e+02],
                [0.00000000e+00, 1.36250581e+03, 4.12043801e+02],
                [0.00000000e+00, 0.00000000e+00, 1.00000000e+00]])

dist = np.array([[0.02759679, 0.2556816, -0.00123938, -0.00305147, -0.91805274]])

# ArUco 마커 크기
aruco_size = 10.8  # cm (10.8cm x 10.8cm)

# ArUco 마커 ID 정의
aruco_22_id = 22
aruco_24_id = 24
aruco_33_id = 33
aruco_34_id = 34


class WebcamNode(Node):
    # def __init__(self, pt_file):
    def __init__(self):
        # 노드 초기화
        super().__init__('WebcamNode')
        self.service_called = False  # service_called 초기화
        # self.pt_file = pt_file
        self.callback_group = ReentrantCallbackGroup()

        # YOLO 모델 로드
        # self.model = YOLO(self.pt_file)
        # self.get_logger().info(f'Loaded model from {self.pt_file}')
        # Check that we have a valid ArUco marker
        if ARUCO_DICT.get(desired_aruco_dictionary, None) is None:
            print("[INFO] ArUCo tag of '{}' is not supported".format(sys.args["type"]))
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
        self.cap = cv2.VideoCapture(18)  # 기본 웹캠 사용
        if not self.cap.isOpened():
            self.get_logger().error('Could not open video device')

        self.cap.set(cv2.CAP_PROP_FOURCC, cv2.VideoWriter_fourcc(*'MJPG'))
        self.cap.set(cv2.CAP_PROP_FRAME_WIDTH, 1280)
        self.cap.set(cv2.CAP_PROP_FRAME_HEIGHT, 720)

        # 변경된 해상도 가져오기
        width = self.cap.get(cv2.CAP_PROP_FRAME_WIDTH)
        height = self.cap.get(cv2.CAP_PROP_FRAME_HEIGHT)
        print(f"해상도: {width}, {height}")

        self.bridge = CvBridge()  # OpenCV 이미지를 ROS 이미지로 변환하기 위한 브릿지 설정

        # 이미지 퍼블리셔 생성
        self.video_publisher = self.create_publisher(
            Image,  # 메시지 타입
            'world_video',  # 토픽 이름
            qos_profile=10,  # QoS 프로파일
            callback_group=self.callback_group
        )

        self.robot_xyz = self.create_publisher(
            RobotCoord,  # 메시지 타입
            'robot_coord',  # 토픽 이름
            qos_profile=10,  # QoS 프로파일
            callback_group=self.callback_group
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
        # self.coordinate_transformer()

    def process_frame(self):
        # 카메라에서 프레임을 읽음
        ret, frame = self.cap.read()
        if not ret:
            self.get_logger().error('Failed to capture frame')
            return

        # 카메라 왜곡 보정
        h, w = frame.shape[:2]
        new_camera_mtx, roi = cv2.getOptimalNewCameraMatrix(mtx, dist, (w, h), 1, (w, h))
        undistorted_image = cv2.undistort(frame, mtx, dist, None, new_camera_mtx)

        # ArUco 딕셔너리와 탐지 파라미터 설정
        aruco_dict = aruco.getPredefinedDictionary(aruco.DICT_5X5_100)
        aruco_params = aruco.DetectorParameters()

        # ArUco 마커 탐지
        corners, ids, rejected = aruco.detectMarkers(undistorted_image, aruco_dict, parameters=aruco_params)
        list = []
        # 마커의 중심점과 rvec, tvec 계산하여 출력
        if ids is not None:
            centers = []
            rvecs, tvecs, _ = aruco.estimatePoseSingleMarkers(corners, aruco_size, mtx, dist)
            marker_positions = {}

            for corner, marker_id, rvec, tvec in zip(corners, ids.flatten(), rvecs, tvecs):
                # 각 마커의 코너 좌표를 가져와 중심점 계산
                corner_points = corner[0]
                center_x = int(np.mean(corner_points[:, 0]))
                center_y = int(np.mean(corner_points[:, 1]))
                centers.append((marker_id, (center_x, center_y), rvec, tvec))

                # 중심점 표시
                cv2.circle(undistorted_image, (center_x, center_y), 5, (0, 255, 0), -1)
                # 좌표축 그리기
                cv2.drawFrameAxes(undistorted_image, mtx, dist, rvec, tvec, 10)

                # rvec 및 tvec 출력
                # print(f"Marker ID: {marker_id}, rvec: {rvec.flatten()}, tvec: {tvec.flatten()}")

                # 마커의 실제 위치 저장
                marker_positions[marker_id] = (rvec, tvec.flatten())

            # 22번 마커를 중심으로 다른 마커들의 실제 위치 계산
        if aruco_33_id in marker_positions:
            rvec_33, tvec_33 = marker_positions[aruco_33_id]
            rotation_33 = R.from_rotvec(rvec_33.flatten())
            rotation_matrix_33 = rotation_33.as_matrix()

            # 중복 방지를 위한 집합
            unique_ids = set()

            for marker_id, (rvec, tvec) in marker_positions.items():
                if marker_id != aruco_33_id and marker_id not in unique_ids:
                    unique_ids.add(marker_id)  # ID 추가로 중복 방지

                    relative_position = tvec - tvec_33
                    actual_position = np.dot(rotation_matrix_33.T, relative_position)

                    A_Coord = Aruco_Coordinate(
                        id=marker_id,
                        x=actual_position[0],
                        y=actual_position[1],
                        z=actual_position[2]
                    )

                    # 34번 마커는 계속 퍼블리싱
                    if marker_id == 34:
                        msg = RobotCoord()
                        msg.robot_coord = str(A_Coord)
                        self.robot_xyz.publish(msg)

                    # 리스트에 추가
                    list.append(A_Coord)

            # 리스트를 서비스 요청으로 전송 (한 번만 실행)
            if list and not self.service_called:
                list_data = [
                    {
                        "id": int(coord.id),  # numpy.int32를 Python int로 변환
                        "x": float(coord.x),  # numpy.float64를 Python float로 변환
                        "y": float(coord.y),
                        "z": float(coord.z)
                    }
                    for coord in list
                ]

                request = Coordinate.Request()
                request.json_data = json.dumps(list_data)  # JSON 직렬화
                print(f"Sending data to service: {list_data}")

                future = self.coordinate_srv_client.call_async(request)
                future.add_done_callback(self.coordinate_transformer_callback)

                self.service_called = True

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

    def cal_robot_init_pose(self):
        pass

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
        node.destroy_node()  # 노드에서 destroy_node 호출
        executor.shutdown()
        rclpy.shutdown()


if __name__ == '__main__':
    main()