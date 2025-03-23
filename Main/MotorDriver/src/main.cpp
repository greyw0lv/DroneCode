#include "Arduino.h"
#include <SoftwareSerial.h>
#include <Servo.h>

Servo ESC0;    //Front Right 1, 0
Servo ESC1;    //Front Left 2, 1
Servo ESC3;   //Back Right 3, 3
Servo ESC2;    //Back left 4, 2
float ESC[4] = {0};

SoftwareSerial espSerial(8, 9);

int potValue;  // value from the analog pin

void setup() {
  // Attach the ESC on pin 9
  Serial.begin(9600);
  espSerial.begin(9600);

  // (pin, min pulse width, max pulse width in microseconds) 

  ESC0.attach(11,200,2000); // not working, check wiring
  ESC1.attach(10,200,2000); // workings
  ESC3.attach(5,200,2000); // working
  ESC2.attach(6,200,2000);// working
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
      ESC[i] = receivedFloats[i];
      Serial.print(receivedFloats[i]);
      Serial.print("\t");
    }
    Serial.println();
  }
  Serial.println("Ending loop");
  delay(500);
  
  
  ESC0.write(ESC[0]);    // Send the signal to the ESC
  ESC1.write(ESC[1]); 
  ESC2.write(ESC[2]); 
  ESC3.write(ESC[3]); 
  
  for (int i = 0; i < 4; i++)
  {
    Serial.print(ESC[i]);
    Serial.print('\t');
  }
  Serial.println();
  

  //Serial.print(ESC.attached(6));

}