import rclpy
from rclpy.node import Node
from rclpy.executors import MultiThreadedExecutor
from b4_fulfillment_interfaces.msg import Button, RobotStatus
from b4_fulfillment_interfaces.srv import Coord

class Robot(Node):
    def __init__(self):
        super().__init__('robot')
        self.subscription = self.create_subscription(Button, 'button_topic', self.listener_callback, 10)

        self.publisher_ = self.create_publisher(RobotStatus, 'robot_status_topic', 10)

        self.srv = self.create_service(Coord, 'to_coord', self.handle_coord_service)

        timer_period = 3.0  # 타이머 주기 (초)
        self.timer = self.create_timer(timer_period, self.timer_callback)
    
        self.status_list = [
                "박스로 가는 중",
                "박스를 옮기는 중",
                "바구니 가지러 가는 중",
                "배달 중"
            ]
        
        self.status_index = 0

    def listener_callback(self, msg):
        self.get_logger().info(f'Received button message: "{msg.button}"')
    
    def timer_callback(self):
        msg = RobotStatus()
        msg.robot_status = self.status_list[self.status_index]
        self.publisher_.publish(msg)
        self.get_logger().info(f'Published robot status: "{msg.robot_status}"')
        self.status_index = (self.status_index + 1) % len(self.status_list)

    def handle_coord_service(self, request, response):
        self.get_logger().info(f'Received coordinate: "{request.coord}"')
        response.is_received = True
        return response

def main(args=None):
    rclpy.init(args=args)
    node = Robot()
    executor = MultiThreadedExecutor()
    executor.add_node(node)
    try:
        executor.spin() 
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()
