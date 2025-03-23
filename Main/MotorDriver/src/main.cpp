#include "Arduino.h"
#include <SoftwareSerial.h>
#include <Servo.h>

Servo ESC1;     // create servo object to control the ESC
Servo ESC2;
Servo ESC3;
Servo ESC4;

SoftwareSerial espSerial(8, 9);

int potValue;  // value from the analog pin

void setup() {
  // Attach the ESC on pin 9
  Serial.begin(9600);
  espSerial.begin(9600);

  // (pin, min pulse width, max pulse width in microseconds) 

  ESC1.attach(11,200,2000); // not working, check wiring
  ESC2.attach(10,200,2000); // working
  ESC3.attach(5,200,2000); // working
  ESC4.attach(6,200,2000);// working
  Serial.println("Ending Setup");
}

void loop() {
  Serial.println("Starting loop");
  // Step 1: Receive 4 floats from ESP32
  byte buffer[16]; // 4 floats = 16 bytes
  if (espSerial.readBytes(buffer, 16) == 16) {
    float* receivedFloats = (float*)buffer;

    // Step 2: Print received floats
    Serial.print("Received from ESP32: ");
    for (int i = 0; i < 4; i++) {
      Serial.print(receivedFloats[i]);
      Serial.print("\t");
    }
    Serial.println();
  }
  Serial.println("Ending loop");
  delay(500);
  /*
  potValue = analogRead(A0);   // reads the value of the potentiometer (value between 0 and 1023)
  potValue = map(potValue, 0, 1023, 0, 180);   // scale it to use it with the servo library (value between 0 and 180)
  ESC1.write(potValue);    // Send the signal to the ESC
  ESC2.write(potValue); 
  ESC3.write(potValue); 
  ESC4.write(potValue); 
  Serial.println(potValue);
  //Serial.print(ESC.attached(6));
*/
}