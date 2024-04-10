import rclpy
from rclpy.node import Node
from std_msgs.msg import Int32 
import serial
import struct
import time

class control_esp(Node):
    def __init__(self):
        super().__init__('control_esp')
        self.pub_throttle = self.create_publisher(Int32, 'throttle', 10)
        self.pub_steer = self.create_publisher(Int32, 'steer', 10)

        self.sub_R2 = self.create_subscription(Int32, 'ps5/R2', self.R2_update, 10)
        self.sub_L2 = self.create_subscription(Int32, 'ps5/L2', self.L2_update, 10)
        self.sub_L_Joy = self.create_subscription(Int32, 'ps5/L_Joy', self.L_Joy_update, 10)
        self.serial_port = serial.Serial('/dev/ttyUSB0', 115200, timeout=2)

        timer_period = 1/100  # 100Hz
        self.timer = self.create_timer(timer_period, self.timer_callback)

        self.R2 = 0
        self.L2 = 0
        self.L_Joy = 128

    def R2_update(self, msg):
        self.R2 = msg.data

    def L2_update(self, msg):
        self.L2 = msg.data

    def L_Joy_update(self, msg):
        self.L_Joy = msg.data

    def timer_callback(self):
        if self.R2 > self.L2:
            throttle = self.map_value(self.R2, 0, 255, 90, 180)
        else if self.L2 > self.R2:
            throttle = self.map_value(self.L2, 0, 255, 90, 0)
        else:
            throttle = 90
        
        steer = self.map_value(self.L_Joy, 0, 255, 130, 64) # 97 is mid, 64 is right, 130 is left

        throttle_msg = Int32()
        steer_msg = Int32()

        throttle_msg.data = throttle
        steer_msg.data = steer

        self.pub_throttle.publish(throttle_msg)
        self.pub_steer.publish(steer_msg)

        data_to_send = struct.pack('<HH', self.throttle, self.steer)

        self.serial_port.write(data_to_send)

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

# 1000, 2000