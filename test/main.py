#  Control a 5V PWM fan speed with the lgpio library
#  Uses lgpio library, compatible with kernel 5.11
#  Author: William 'jawn-smith' Wilson

import lgpio
import time

# Configuration
FAN = 18 # pin used to drive PWM fan
FREQ = 10000

h = lgpio.gpiochip_open(0)
print("Start")

try:
    while True:

        lgpio.tx_pwm(h, FAN, FREQ, 0)
        # # Turn the fan off
        # lgpio.tx_pwm(h, FAN, FREQ, 0)
        # print("Back")
        # time.sleep(5)

        # # Turn the fan to max speed
        # lgpio.tx_pwm(h, FAN, FREQ, 100)
        # print("Forward")
        # time.sleep(5)

except KeyboardInterrupt:
    # Turn the fan to medium speed
    lgpio.tx_pwm(h, FAN, FREQ, 50)
    lgpio.gpiochip_close(h)