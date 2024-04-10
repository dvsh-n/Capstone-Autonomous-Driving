import rclpy
from rclpy.node import Node
from std_msgs.msg import Int32
from evdev import InputDevice, categorize, ecodes   
import threading    

class ps5_controller(Node):

    def __init__(self):
        super().__init__('ps5_controller')
        self.pub_R2 = self.create_publisher(Int32, 'ps5/R2', 10)
        self.pub_L2 = self.create_publisher(Int32, 'ps5/L2', 10)
        self.pub_L_Joy = self.create_publisher(Int32, 'ps5/L_Joy', 10)
        self.pub_L_Pad_LR = self.create_publisher(Int32, 'ps5/L_Pad_LR', 10)
        self.pub_L_Pad_UD = self.create_publisher(Int32, 'ps5/L_Pad_UD', 10)
        timer_period = 1/100  # 100Hz
        self.timer = self.create_timer(timer_period, self.timer_callback)
        self.gamepad = InputDevice('/dev/input/event2') # 18 for laptop, 4 for pi
        self.button_presses = {                          
            308: 'square',
            304: 'x',
            305: 'circle',
            307: 'triangle',
            310: 'L1',
            311: 'R1',
            312: 'L2',                        
            313: 'R2',
            314: 'share',                          
            315: 'pause',                          
            317: 'L3',                             
            318: 'R3',
            316: 'playstation',
            306: 'touchpad'
        }
        self.button_values = {                           
            0: 'up',
            1: 'down'
        }
        self.absolutes = {                               # ecodes.EV_ABS
            0: 'left joystick left/right',          # 0 = left, 255 = right
            1: 'left joystick up/down',             # 0 = up, 255 = down
            2: 'right joystick left/right',         # 0 = left, 255 = right
            3: 'L2 analog',                         # 0 = no press, 255 = full press
            4: 'R2 analog',                         # 0 = no press, 255 = full press
            5: 'right joystick up/down',            # 0 = up, 255 = down
            16: 'leftpad left/right',               # -1 = left, 0 = stop pressing, 1 = right
            17: 'leftpad up/down',                  # -1 = up, 0 = stop pressing, 1 = down
        }
        self.R2_val = 0
        self.L2_val = 0
        self.L_Pad_UD = 0
        self.L_Pad_LR = 0
        self.L_Joy_val = 128

        self.gamepad_thread = threading.Thread(target=self.ps5_handle)
        self.gamepad_thread.start()

    def timer_callback(self):
        R2_msg = Int32()
        L2_msg = Int32()
        L_Joy_msg = Int32()
        L_Pad_LR_msg = Int32()
        L_Pad_UD_msg = Int32()

        R2_msg.data = self.R2_val
        L2_msg.data = self.L2_val
        L_Joy_msg.data = self.L_Joy_val
        L_Pad_LR_msg.data = self.L_Pad_LR
        L_Pad_UD_msg.data = self.L_Pad_UD

        self.pub_R2.publish(R2_msg)
        self.pub_L2.publish(L2_msg)
        self.pub_L_Joy.publish(L_Joy_msg)
        self.pub_L_Pad_LR.publish(L_Pad_LR_msg)
        self.pub_L_Pad_UD.publish(L_Pad_UD_msg)

    def ps5_handle(self):
        for event in self.gamepad.read_loop():
            if event.type == ecodes.EV_ABS and event.code in self.absolutes:                    
                value = event.value

                match event.code:
                    case 0: self.L_Joy_val = value # Left Joy left-right
                    case 2: self.L2_val = value # L2
                    case 5: self.R2_val = value # R2
                    case 16: self.L_Pad_LR = value # Leftpad -1 = left, 0  = released, 1 = right
                    case 17: self.L_Pad_UD = value # Leftpad -1 = up, 0  = released, 1 = down

def main(args=None):
    rclpy.init(args=args)
    ps5_node = ps5_controller()
    try:
        rclpy.spin(ps5_node)
    except KeyboardInterrupt:
        pass
    finally:
        ps5_node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()