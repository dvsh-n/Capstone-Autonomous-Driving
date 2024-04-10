#include <ESP32Servo.h>

Servo ESC;     // create servo object to control the ESC

int potValue;  // value from the analog pin

void setup() {
  Serial.begin(115200);
  ESC.attach(13); // (pin, min pulse width, max pulse width in microseconds) 
  ESC.write(180); 
}

void loop() {
  potValue = analogRead(15);   // reads the value of the potentiometer (value between 0 and 1023)
  potValue = map(potValue, 0, 4095, 0, 180);   // scale it to use it with the servo library (value between 0 and 180)
  Serial.println(potValue);
  ESC.write(potValue);    // Send the signal to the ESC
}

// 97 is middle point
// 130 is maximum
// 62 is minimum