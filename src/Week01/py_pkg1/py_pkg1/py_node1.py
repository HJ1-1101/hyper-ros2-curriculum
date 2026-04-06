#!/usr/bin/env python3
import rclpy
from rclpy.node import Node

class MyCustomNode(Node):
    def __init__(self):
        super().__init__('py_node1')
        self.get_logger().info('py_node1 has been started in ROS 2 Jazzy!')
        
        # Timer triggering every 1.0 seconds
        self.timer = self.create_timer(1.0, self.timer_callback)

    def timer_callback(self):
        self.get_logger().info('Jazzy py_node1 is running...')

def main(args=None):
    rclpy.init(args=args)
    node = MyCustomNode()
    
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()