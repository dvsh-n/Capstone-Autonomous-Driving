#include <ESP32Servo.h>

Servo ESC_throttle;     // create servo object to control the ESC_throttle
Servo Steer;

uint16_t throttle_val; 
uint16_t steer_val;

void setup() {
  Serial.begin(115200);
  ESC_throttle.attach(13,1000,2000); // (pin, min pulse width, max pulse width in microseconds) 
  ESC_throttle.write(90); 
  Steer.attach(15);
  Steer.write(97);
  delay(1000);
}

void loop() {
  if (Serial.available() >= 4) { // Check if at least 2 bytes are available (for a 16-bit integer)
    uint8_t buf[4];
    Serial.readBytes(buf, 4);
    throttle_val = buf[0] | (buf[1] << 8); // Read the incoming string until a newline character
    steer_val = buf[2] | (buf[3] << 8);
    ESC_throttle.write(throttle_val);    // Send the signal to the ESC_throttle
    Steer.write(steer_val)
  }
  
}




