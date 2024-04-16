# if you don't need to be wireless, check out the library pydualsense

import time
from evdev import InputDevice, categorize, ecodes                   # pip install evdev
gamepad = InputDevice('/dev/input/event18')      # "cd /dev/input" then "ls -al" to see your connections
# event 2 for pi, event 18 for PC
button_presses = {                          # ecodes.EV_KEY
    308: 'square',
    304: 'x',
    305: 'circle',
    307: 'triangle',
    310: 'L1',
    311: 'R1',
    312: 'L2',                              # this shows up when the button clicks before the analog signals are reported
    313: 'R2',
    314: 'share',                           # 3 vertical lines, top left side of touchpad
    315: 'pause',                           # 3 horizontal lines, top right of touchpad
    317: 'L3',                              # left joystick press down vertically
    318: 'R3',
    316: 'playstation',
    306: 'touchpad'
}
buttons_state = [0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0]

button_values = {                           # ecodes.EV_KEY button press values
    0: 'up',
    1: 'down'
}

absolutes = {                               # ecodes.EV_ABS
    0: 'left joystick left/right',          # 0 = left, 255 = right
    1: 'left joystick up/down',             # 0 = up, 255 = down
    2: 'right joystick left/right',         # 0 = left, 255 = right
    3: 'L2 analog',                         # 0 = no press, 255 = full press
    4: 'R2 analog',                         # 0 = no press, 255 = full press
    5: 'right joystick up/down',            # 0 = up, 255 = down
    16: 'leftpad left/right',               # -1 = left, 0 = stop pressing, 1 = right
    17: 'leftpad up/down',                  # -1 = up, 0 = stop pressing, 1 = down
}

CENTER = 128
BLIND = 6                                   # there's a lot of drift at 128, so don't report changes within (128 - this value)
                 
left_joystick, right_joystick, L2_R2 = [CENTER, CENTER], [CENTER, CENTER], [0, 0]

if __name__ == '__main__':

    for event in gamepad.read_loop():

        if event.type == ecodes.EV_KEY and event.code in button_presses:       # any button press other than leftpad
            button, direction = button_presses[event.code], button_values[event.value]
            print(f'{button} {direction}')

        if event.type == ecodes.EV_ABS and event.code in absolutes:                     # leftpad, joystick motion, or L2/R2 triggers
            action, value = absolutes[event.code], event.value

            match event.code:
                case 0: left_joystick[0] = value
                case 1: left_joystick[1] = value
                case 2: L2_R2[0] = value
                case 3: right_joystick[0] = value
                case 4: right_joystick[1] = value
                case 5: L2_R2[1] = value
                case 16: 
                    if value == 0:
                        print("released")
                    elif value == 1:
                        print("right")
                    elif value == -1:
                        print("left")
                case 17: 
                    if value == 0:
                        print("released")
                    elif value == 1:
                        print("down")
                    elif value == -1:
                        print("up")

            if event.value > (CENTER - BLIND) and event.value < (CENTER + BLIND):   # skip printing the jittery center for the joysticks
                continue                       
            else: 
                print(f'{left_joystick}, {right_joystick}')

