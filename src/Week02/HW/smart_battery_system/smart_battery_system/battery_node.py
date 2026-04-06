import rclpy
from rclpy.node import Node
from my_robot_interfaces.msg import BatteryStatus
from my_robot_interfaces.srv import SetLedColor

class BatteryNode(Node):
    def __init__(self):
        super().__init__('battery_node')
        self.publisher_ = self.create_publisher(BatteryStatus, 'battery_info', 10)
        self.srv = self.create_service(SetLedColor, 'set_led', self.set_led_callback)
        self.timer = self.create_timer(1.0, self.timer_callback)
        
        self.percentage = 100.0
        self.led_color = "GREEN"  # Start with Green

    def timer_callback(self):
        msg = BatteryStatus()
        msg.percentage = self.percentage
        msg.current_led_color = self.led_color
        msg.status = "NORMAL" if self.percentage > 20 else "LOW"
        
        self.publisher_.publish(msg)
        self.get_logger().info(f'Battery: {self.percentage}% | LED: {self.led_color}')
        
        if self.percentage > 0:
            self.percentage -= 5.0  # Dropping faster for testing

    def set_led_callback(self, request, response):
        self.get_logger().warn(f'Changing LED from {self.led_color} to {request.color}')
        self.led_color = request.color
        response.success = True
        return response

def main():
    rclpy.init()
    node = BatteryNode()
    rclpy.spin(node)
    rclpy.shutdown()