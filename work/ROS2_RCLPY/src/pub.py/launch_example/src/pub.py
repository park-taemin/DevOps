#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from std_msgs.msg import String

def main(args=None):
    rclpy.init(args=args)
    node = Node('pub_node')
    
    # 퍼블리셔 생성 (토픽명: hello_topic, QoS: 10)
    publisher = node.create_publisher(String, 'hello_topic', 10)
    
    msg = String()
    msg.data = 'Hello world! (Python)'

    def timer_callback():
        node.get_logger().info(f'Publishing: "{msg.data}"')
        publisher.publish(msg)

    # 1초마다 실행되는 타이머 생성
    timer = node.create_timer(1.0, timer_callback)

    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()