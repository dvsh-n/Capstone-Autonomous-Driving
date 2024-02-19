import rclpy
from rclpy.node import Node
from std_msgs.msg import Float64MultiArray
from evdev import InputDevice, categorize, ecodes       

class ps5_controller(Node):

    def __init__(self):
        super().__init__('ps5_controller')
        self.publisher_ = self.create_publisher(Float64MultiArray, 'ps5/analog_data', 10)
        timer_period = 1/30  # 30Hz
        self.gamepad = InputDevice('/dev/input/event4') 
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
        self.data = [0,0,0,0,0,0] 

    def timer_callback(self):
        msg = Float64MultiArray()
        msg.data = self.data
        self.publisher_.publish(msg)
    
    def run(self):
        for event in self.gamepad.read_loop():
            if event.type == ecodes.EV_ABS and event.code in self.absolutes:                    
                action, value = self.absolutes[event.code], event.value

                match event.code:
                    case 0: self.data[0] = value # Left Joy left-right
                    case 1: self.data[1] = value # Left Joy up-down
                    case 2: self.data[4] = value # L2
                    case 3: self.data[2] = value # Right Joy left-right
                    case 4: self.data[3] = value # Right Joy up-down
                    case 5: self.data[5] = value # R2

def main(args=None):
    rclpy.init(args=args)
    ps5_controller = ps5_controller()
    try:
        ps5_controller.run()
    except KeyboardInterrupt:
        pass
    finally:
        ps5_controller.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()