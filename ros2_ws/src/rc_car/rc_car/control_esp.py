import rclpy
from rclpy.node import Node
from std_msgs.msg import Int32 
import serial

class control_esp(Node):
    def __init__(self):
        super().__init__('control_esp')
        self.subscription = self.create_subscription(Int32, 'ps5/R2', self.listener_callback, 10)
        self.serial_port = serial.Serial('/dev/ttyUSB0', 115200, timeout=2)

    def listener_callback(self, msg):
        mapped_value = self.map_value(msg.data, 0, 255, 90, 180)
        self.get_logger().info(f'Receiving: {msg.data}, Sending: {mapped_value}')

        self.serial_port.write(str(mapped_value).encode())

    def map_value(self, value, in_min, in_max, out_min, out_max):
        return (value - in_min) * (out_max - out_min) // (in_max - in_min) + out_min

def main(args=None):
    rclpy.init(args=args)
    control_esp = control_esp()
    rclpy.spin(control_esp)

    control_esp.serial_port.close()
    control_esp.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
