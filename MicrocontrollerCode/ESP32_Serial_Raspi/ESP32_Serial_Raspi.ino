#include <ESP32Servo.h>

Servo ESC;     // create servo object to control the ESC

uint16_t throttle_val; 

void setup() {
  Serial.begin(115200);
  ESC.attach(13,1000,2000); // (pin, min pulse width, max pulse width in microseconds) 
  ESC.write(90); 
  delay(1000);
}

void loop() {
  if (Serial.available() >= 2) { // Check if at least 2 bytes are available (for a 16-bit integer)
    uint8_t buf[2];
    Serial.readBytes(buf, 2);
    throttle_val = buf[0] | (buf[1] << 8); // Read the incoming string until a newline character
    ESC.write(throttle_val);    // Send the signal to the ESC
  }
  
}




