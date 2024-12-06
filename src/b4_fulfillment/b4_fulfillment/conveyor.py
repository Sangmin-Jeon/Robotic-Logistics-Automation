import rclpy
from rclpy.node import Node
from rclpy.action import ActionServer
from b4_fulfillment_interfaces.action import Conveyor  # 액션 인터페이스 파일 import
from b4_fulfillment_interfaces.msg import ConvGoButton, ConvStopButton
import serial
import time
import asyncio


class ConveyorServer(Node):
    def __init__(self):
        super().__init__('conveyor_action_server')

        # 액션 서버 생성
        self._action_server = ActionServer(self, Conveyor, 'conveyor_action', self.execute_callback)
        self.convgo_subscription = self.create_subscription(ConvGoButton, 'convgo_topic', self.convgo_button_callback,
                                                            10)
        self.convstop_subscription = self.create_subscription(ConvStopButton, 'convstop_topic',
                                                              self.convstop_button_callback, 10)

        self.serial_port = None
        self.stop_requested = False

    def convgo_button_callback(self, msg):
        # 로봇 상태 업데이트 메시지 수신 시 처리
        self.get_logger().info(f'Received Conveyor Button: {msg.convgobutton}')

        if msg.convgobutton:
            try:
                if self.serial_port is None or not self.serial_port.is_open:
                    self.serial_port = serial.Serial('/dev/ttyACM0', 115200, timeout=1)  # 시리얼 포트 이름과 보드레이트 설정
                    time.sleep(0.5)  # 아두이노 초기화를 위해 잠시 대기

                self.serial_port.write(bytes('500\n', 'utf-8'))
                self.get_logger().info('Sent 500 to Arduino')
            except serial.SerialException as e:
                self.get_logger().error(f'Failed to send 500 to Arduino: {e}')

    def convstop_button_callback(self, msg):
        # 로봇 상태 업데이트 메시지 수신 시 처리
        self.get_logger().info(f'Received Conveyor Button: {msg.convstopbutton}')

        if msg.convstopbutton:
            self.stop_requested = True
            try:
                if self.serial_port is None or not self.serial_port.is_open:
                    self.serial_port = serial.Serial('/dev/ttyACM0', 115200, timeout=1)  # 시리얼 포트 이름과 보드레이트 설정
                    time.sleep(0.5)  # 아두이노 초기화를 위해 잠시 대기

                self.serial_port.write(bytes('1\n', 'utf-8'))
                self.get_logger().info('Sent stop to Arduino')
            except serial.SerialException as e:
                self.get_logger().error(f'Failed to send 0 to Arduino: {e}')

    async def execute_callback(self, goal_handle):
        # 시리얼 포트 설정
        try:
            if self.serial_port is None or not self.serial_port.is_open:
                self.serial_port = serial.Serial('/dev/ttyACM0', 115200, timeout=1)  # 시리얼 포트 이름과 보드레이트 설정
                await asyncio.sleep(0.5)  # 아두이노 초기화를 위해 잠시 대기
        except serial.SerialException as e:
            self.get_logger().error(f'Failed to connect to Arduino: {e}')
            goal_handle.abort()
            return Conveyor.Result(cfinished=False)

        self.get_logger().info(f'Received goal: {goal_handle.request.goal}')

        # 시리얼 통신으로 goal 값 전송
        goal_value = goal_handle.request.goal
        try:
            goal = f'{goal_value}\n'
            self.serial_port.write(bytes(goal, 'utf-8'))
            self.get_logger().info(f'Sent goal to Arduino: {goal_value}')
        except serial.SerialException as e:
            self.get_logger().error(f'Failed to send data to Arduino: {e}')
            goal_handle.abort()
            return Conveyor.Result(cfinished=False)

        # 피드백 및 결과 처리
        feedback_msg = Conveyor.Feedback()
        result = Conveyor.Result()

        last_message_time = time.time()
        while True:
            # 비동기적으로 정지 요청 확인
            if self.stop_requested:
                self.get_logger().warn('Stop requested. Aborting goal.')
                goal_handle.abort()
                return Conveyor.Result(cfinished=False)

            # 시리얼로부터 피드백 수신
            if self.serial_port.in_waiting > 0:
                data = self.serial_port.read()
                response = str(data.decode('utf-8')).strip()  # Properly decode and strip newline characters
                self.get_logger().info(f'Received valid response from Arduino: {response}')

                if response == '1':
                    feedback_msg.cfeedback = response
                    goal_handle.publish_feedback(feedback_msg)

                last_message_time = time.time()  # 메시지 수신 시간 갱신

            # 1초 이상 메시지 갱신이 없으면 시리얼 포트 연결 해제
            if time.time() - last_message_time > 2.0:
                self.get_logger().error('No message update for 1 second. Closing serial connection.')
                goal_handle.abort()
                return Conveyor.Result(cfinished=True)

            await asyncio.sleep(0.1)  # 루프에서 잠시 대기하여 CPU 사용률 감소

    def destroy_node(self):
        super().destroy_node()
        if self.serial_port and self.serial_port.is_open:
            self.serial_port.close()  # 시리얼 포트 닫기


def main(args=None):
    rclpy.init(args=args)

    arduino_action_server = ConveyorServer()

    try:
        rclpy.spin(arduino_action_server)
    except KeyboardInterrupt:
        arduino_action_server.get_logger().info('Action server stopping...')
    finally:
        arduino_action_server.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()