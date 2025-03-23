#include "Arduino.h"
#include <SoftwareSerial.h>
#include <Servo.h>

#include <SharpIR.h>
#include <Adafruit_NeoPixel.h>


#define IRPin A0//PSD
#define IRPin A1//Photocell
#define PIN 6//PSD pin
#define model 100500//PSD
#define NUMPIXELS 16 //Neo lights

Adafruit_NeoPixel pixels(NUMPIXELS, PIN, NEO_GRB + NEO_KHZ800);
#define DELAYVAL 500

int distance_cm;
int light;

SharpIR mySensor = SharpIR(IRPin, model);


Servo ESC0;    //Front Right 1, 0
Servo ESC1;    //Front Left 2, 1
Servo ESC3;   //Back Right 3, 3
Servo ESC2;    //Back left 4, 2
float ESC[4] = {0};

SoftwareSerial espSerial(8, 9);

int potValue;  // value from the analog pin

void NeoPixel(int r, int g, int b){
  // pixel
  pixels.clear(); // Set all pixel colors to 'off'
  int a = pixels.Color(r,g,b); //set colour

  pixels.setPixelColor(0,a);
  pixels.setPixelColor(1,a);
  pixels.setPixelColor(2,a);
  pixels.setPixelColor(3,a);
  pixels.setPixelColor(4,a);
  pixels.setPixelColor(5,a);
  pixels.setPixelColor(6,a);
  pixels.setPixelColor(7,a);

  pixels.show();
}

void setup() {
  // Attach the ESC on pin 9
  Serial.begin(9600);
  espSerial.begin(9600);

  // (pin, min pulse width, max pulse width in microseconds) 

  ESC0.attach(11,200,2000); // not working, check wiring
  ESC1.attach(10,200,2000); // workings
  ESC3.attach(5,200,2000); // working
  ESC2.attach(6,200,2000);// working
  

  pixels.begin();
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
//Random Sensors GO!!!!
  NeoPixel(150,150,10);
  
  distance_cm = mySensor.distance();
  Serial.print("Mean Dist (cm): ");
  Serial.println(distance_cm);
  light = mySensor.light();
  Serial.print("lightlvl: ");
  Serial.println(light);

  //Back to motor stuff.
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
  
  delay(500);
  //Serial.print(ESC.attached(6));

}