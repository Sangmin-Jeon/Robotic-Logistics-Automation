import rclpy
from rclpy.node import Node
from rclpy.executors import MultiThreadedExecutor
from rclpy.callback_groups import ReentrantCallbackGroup
from geometry_msgs.msg import Twist
from b4_fulfillment_interfaces.msg import Button, RobotStatus, RobotMoveStop


class Robot(Node):
    def __init__(self):
        super().__init__('robot')
        self.callback_group = ReentrantCallbackGroup()

        self.subscription = self.create_subscription(Button, 'button_topic', self.listener_callback, 10,
                                                     callback_group=self.callback_group)

        self.publisher_ = self.create_publisher(RobotStatus, 'robot_status_topic', 10,
                                                callback_group=self.callback_group)

        self.cmd_vel_publisher = self.create_publisher(Twist, 'cmd_vel', 10, callback_group=self.callback_group)

        self.robotmove_sub = self.create_subscription(RobotMoveStop, 'move_topic', self.robotmove_callback, 10,
                                                      callback_group=self.callback_group)

    def listener_callback(self, msg):
        self.get_logger().info(f'Received button message: "{msg.button}"')

    def robotmove_callback(self, msg):
        self.get_logger().info(f'Received move command: "{msg.move}"')
        twist = Twist()

        if msg.move == 'move':
            twist.linear.x = -0.03  # Forward speed
            twist.linear.y = 0.0
            twist.angular.z = 0.0
            self.get_logger().info('Robot moving forward.')
        elif msg.move == 'stop':
            twist.linear.x = 0.0
            twist.linear.y = 0.0
            twist.angular.z = 0.0
            self.get_logger().info('Robot stopped.')
        elif msg.move == 'back':
            twist.linear.x = 0.03  # Backward speed
            twist.linear.y = 0.0
            twist.angular.z = 0.0
            self.get_logger().info('Robot moving backward.')
        else:
            self.get_logger().warning(f'Unknown command: "{msg.move}"')

        self.cmd_vel_publisher.publish(twist)


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