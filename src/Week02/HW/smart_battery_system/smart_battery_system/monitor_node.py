import rclpy
from rclpy.node import Node
from my_robot_interfaces.msg import BatteryStatus
from my_robot_interfaces.srv import SetLedColor

class MonitorNode(Node):
    def __init__(self):
        super().__init__('monitor_node')
        self.sub = self.create_subscription(BatteryStatus, 'battery_info', self.listener_callback, 10)
        self.client = self.create_client(SetLedColor, 'set_led')

    def listener_callback(self, msg):
        # Only call the service if battery is low AND LED is still Green
        if msg.percentage <= 20.0 and msg.current_led_color == "GREEN":
            self.get_logger().error(f'DANGER! Battery is {msg.percentage}%. Switching LED to RED!')
            self.call_set_led_service("RED")

    def call_set_led_service(self, color):
        while not self.client.wait_for_service(timeout_sec=1.0):
            self.get_logger().info('Waiting for Battery Node service...')
        
        req = SetLedColor.Request()
        req.color = color
        self.client.call_async(req)

def main():
    rclpy.init()
    node = MonitorNode()
    rclpy.spin(node)
    rclpy.shutdown()