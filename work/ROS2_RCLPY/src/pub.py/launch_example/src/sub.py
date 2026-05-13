#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from std_msgs.msg import String

def main(args=None):
    rclpy.init(args=args)
    node = Node('sub_node')

    # 메시지를 받았을 때 실행될 콜백 함수
    def listener_callback(msg):
        node.get_logger().info(f'Received message: "{msg.data}"')

    # 구독자 생성
    subscription = node.create_subscription(
        String,
        'hello_topic',
        listener_callback,
        10
    )

    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()