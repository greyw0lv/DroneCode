#include "Arduino.h"
#include <Servo.h>

Servo ESC1;     // create servo object to control the ESC
Servo ESC2; 
Servo ESC3; 
Servo ESC4; 
int potValue;  // value from the analog pin

void setup() {
  // Attach the ESC on pin 9
  Serial.begin(9600);
  ESC1.attach(5,1000,2000); // (pin, min pulse width, max pulse width in microseconds) 
  ESC2.attach(6,1000,2000);
  ESC3.attach(10,1000,2000);
  ESC4.attach(11,1000, 2000);
}

void loop() {
  //potValue = analogRead(A0);   // reads the value of the potentiometer (value between 0 and 1023)
  //potValue = map(potValue, 0, 1023, 0, 180);   // scale it to use it with the servo library (value between 0 and 180)
  ESC1.write(160);    // Send the signal to the ESC
  ESC2.write(120);
  ESC3.write(100);
  ESC4.write(90);
  //Serial.println(potValue);
}