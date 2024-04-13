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
        self.pub_throttle_sensitivity = self.create_publisher(Int32, 'throttle_sensitivity', 10)
        self.pub_steer_sensitivity = self.create_publisher(Int32, 'steer_sensitivity', 10)
        
        self.sub_L_Pad_LR = self.create_subscription(Int32, 'ps5/L_Pad_LR', self.L_Pad_LR_update, 10)
        self.sub_L_Pad_UD = self.create_subscription(Int32, 'ps5/L_Pad_UD', self.L_Pad_UD_update, 10)
        self.sub_R2 = self.create_subscription(Int32, 'ps5/R2', self.R2_update, 10)
        self.sub_L2 = self.create_subscription(Int32, 'ps5/L2', self.L2_update, 10)
        self.sub_L_Joy = self.create_subscription(Int32, 'ps5/L_Joy', self.L_Joy_update, 10)
        self.serial_port = serial.Serial('/dev/ttyUSB0', 115200, timeout=2)

        timer_period = 1/100  # 100Hz
        self.timer = self.create_timer(timer_period, self.timer_callback)

        self.R2 = 0
        self.L2 = 0
        self.L_Joy = 128
        self.throttle_sensitivity = 500
        self.steer_sensitivity = 500

        self.max_sensitivity = 1000
        self.min_sensitivity = 100

    def R2_update(self, msg):
        self.R2 = msg.data

    def L2_update(self, msg):
        self.L2 = msg.data

    def L_Joy_update(self, msg):
        self.L_Joy = msg.data
    
    def L_Pad_LR_update(self, msg):
        self.steer_sensitivity = max(self.min_sensitivity, self.steer_sensitivity + msg.data)
        self.steer_sensitivity = min(self.max_sensitivity, self.steer_sensitivity + msg.data)

    def L_Pad_UD_update(self, msg):
        self.throttle_sensitivity = max(self.min_sensitivity, self.throttle_sensitivity - msg.data)
        self.throttle_sensitivity = min(self.max_sensitivity, self.throttle_sensitivity - msg.data)

    def timer_callback(self):
        if self.R2 > self.L2:
            throttle = self.map_value(self.R2, 0, 255, 90, self.map_value((self.throttle_sensitivity/self.max_sensitivity), 0, 1, 90, 180))
        elif self.L2 > self.R2:
            throttle = self.map_value(self.L2, 0, 255, 90, self.map_value((self.throttle_sensitivity/self.max_sensitivity), 0, 1, 90, 0))
        else:
            throttle = 90
        
        max_steer_left = self.map_value((self.steer_sensitivity/self.max_sensitivity), 0, 1, 97, 130)
        max_steer_right = self.map_value((self.steer_sensitivity/self.max_sensitivity), 0, 1, 97, 64)
        steer = self.map_value(self.L_Joy, 0, 255, max_steer_left, max_steer_right) # 97 is mid, 64 is right, 130 is left

        throttle_msg = Int32()
        steer_msg = Int32()
        throttle_sensitivity_msg = Int32()
        steer_sensititvity_msg = Int32()

        throttle_msg.data = int(throttle)
        steer_msg.data = int(steer)
        steer_sensititvity_msg.data = int(self.steer_sensitivity)
        throttle_sensitivity_msg.data = int(self.throttle_sensitivity)

        self.pub_throttle.publish(throttle_msg)
        self.pub_steer.publish(steer_msg)
        self.pub_steer_sensitivity.publish(steer_sensititvity_msg)
        self.pub_throttle_sensitivity.publish(throttle_sensitivity_msg)

        data_to_send = struct.pack('<HH', int(throttle), int(steer))

        self.serial_port.write(data_to_send)

    def map_value(self, value, in_min, in_max, out_min, out_max):
        return (value - in_min) * (out_max - out_min) // (in_max - in_min) + out_min

def main(args=None):
    rclpy.init(args=args)
    control_esp_node = control_esp()
    rclpy.spin(control_esp_node)

    control_esp_node.serial_port.close()
    control_esp_node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()

# 1000, 2000