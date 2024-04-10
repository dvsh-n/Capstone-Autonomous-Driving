import serial
import struct
import time

# Serial port configuration
serial_port = '/dev/ttyUSB0'  # Adjust this to your serial port
baud_rate = 115200  # Adjust this to match the baud rate of your device

# Mapping function
def map_value(value, in_min, in_max, out_min, out_max):
    # Ensure the input is within the provided bounds
    value = max(min(value, in_max), in_min)
    # Perform the mapping
    return (value - in_min) * (out_max - out_min) // (in_max - in_min) + out_min

# Open the serial port
ser = serial.Serial(serial_port, baud_rate)
time.sleep(2)  # Wait for the connection to establish

try:
    while True:
        for i in range(128, 255):

            data_to_send = i
            data_to_send = map_value(data_to_send, 0, 255, 0, 180)
            data_to_send = struct.pack('<H', data_to_send)
            ser.write(data_to_send)
            time.sleep(0.1)

            print(f"Sent mapped value: {i}")

finally:
    # Make sure to close the port
    ser.close()

print("Serial port closed.")
