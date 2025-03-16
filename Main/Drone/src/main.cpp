/*
  Rui Santos & Sara Santos - Random Nerd Tutorials
  Complete project details at https://RandomNerdTutorials.com/esp-now-two-way-communication-esp32/
  Permission is hereby granted, free of charge, to any person obtaining a copy of this software and associated documentation files.
  The above copyright notice and this permission notice shall be included in all copies or substantial portions of the Software.
*/
#include "Arduino.h"
#include <esp_now.h>
#include <WiFi.h>
#include <Streaming.h>
#include <ESP32Servo.h>


// REPLACE WITH THE MAC Address of your receiver 
uint8_t broadcastAddress[] = {0x88, 0x13, 0xbf, 0xc8, 0x57, 0x9c};

// Define variables to store Remote readings to be sent

// Define variables to store incoming readings

// Variable to store if sending data was successful
String success;

//Structure example to send data

Servo neServo;

//Must match the receiver structure
struct Vec4{
  float NW,NE,SE,SW;
};

const float BASELINE = 0.5;
Vec4 motorPower;

struct Vec2{
  float x,y;
};

struct struct_message {
  Vec2 Joy;
  double Pot;
};

// Create a struct_message to hold incoming sensor readings
struct_message incomingReadings;

esp_now_peer_info_t peerInfo;

// Callback when data is sent
void OnDataSent(const uint8_t *mac_addr, esp_now_send_status_t status) {
  Serial.print("\r\nLast Packet Send Status:\t");
  Serial.println(status == ESP_NOW_SEND_SUCCESS ? "Delivery Success" : "Delivery Fail");
  if (status ==0){
    success = "Delivery Success :)";
  }
  else{
    success = "Delivery Fail :(";
  }
}

// Callback when data is received
void OnDataRecv(const uint8_t * mac, const uint8_t *incomingData, int len) {
  memcpy(&incomingReadings, incomingData, sizeof(incomingReadings));
  Serial.print("Bytes received: ");
  Serial.println(len);
}
 
float radiansFromPosition(Vec2 v){
  float theta = atan2(v.x, v.y) /(2*3.1415);
  return theta;
}

void Motor(float magnitude, float angle){
  //angle from -0.5radian to +0.5 radian
  motorPower.NE = BASELINE;
  motorPower.NW = BASELINE;
  motorPower.SE = BASELINE;
  motorPower.SW = BASELINE;

  if (angle < -0.25) {
    Serial.println("SouthEast");
    motorPower.SE = motorPower.SE + ((1 - motorPower.SE)*magnitude);
  } else if(angle < 0.0) {
    Serial.println("SouthWest");
    motorPower.SW = motorPower.SW + ((1 - motorPower.SW)*magnitude);
  } else if(angle < 0.25){
    Serial.println("NorthWest");
    motorPower.NW = motorPower.NW + ((1 - motorPower.NW)*magnitude);
  } else {
    Serial.println("NorthEast");
    motorPower.NE = motorPower.NE + ((1 - motorPower.NE)*magnitude);
  }
  
  analogWrite (13,map(motorPower.SE, 0 , 1, 0, 255));
  analogWrite (12,map(motorPower.SW, 0 , 1, 0, 255));
  analogWrite (14,map(motorPower.NW, 0 , 1, 0, 255));
  analogWrite (27,map(motorPower.NE, 0 , 1, 0, 255));
  Serial << "NW:" << motorPower.NW << "\tNE:" << motorPower.NE << "\nSW:" << motorPower.SW << "\tSE" << motorPower.SE << "\n" << endl;
}

void setup() {
  // Init Serial Monitor
  Serial.begin(115200);
  ESP32PWM::allocateTimer(0);

  // Set device as a Wi-Fi Station
  WiFi.mode(WIFI_STA);

  // Init ESP-NOW
  if (esp_now_init() != ESP_OK) {
    Serial.println("Error initializing ESP-NOW");
    return;
  }

  // Once ESPNow is successfully Init, we will register for Send CB to
  // get the status of Trasnmitted packet
  esp_now_register_send_cb(OnDataSent);
  
  // Register peer
  memcpy(peerInfo.peer_addr, broadcastAddress, 6);
  peerInfo.channel = 0;  
  peerInfo.encrypt = false;
  
  // Add peer        
  if (esp_now_add_peer(&peerInfo) != ESP_OK){
    Serial.println("Failed to add peer");
    return;
  }
  // Register for a callback function that will be called when data is received
  esp_now_register_recv_cb(esp_now_recv_cb_t(OnDataRecv));
}
 
void loop() {
  //Serial.print(incomingReadings.Joy.x);
  //Serial.print("\t");
  //Serial.print(incomingReadings.Joy.y);
  //Serial.print("\t");
  //Serial.println(incomingReadings.Pot);  
  
  if (!neServo.attached()) {
		neServo.setPeriodHertz(50); // standard 50 hz servo
		neServo.attach(33, 1000, 2000); // Attach the servo after it has been detatched
	}
	//neServo.write(0);
  //delay(1000);
  analogWrite(33, 150);
  neServo.write(150);

  //Convert X and Y into angle
    //Disable Joystick if Centered
    
  float theta = radiansFromPosition(incomingReadings.Joy);
  Serial.println(theta);
  //Scale by Potentiometer

  ///@loOK HERE
  incomingReadings.Pot = 1.0;//DELETE THIS  

  if (abs(incomingReadings.Joy.x) > 100 || abs(incomingReadings.Joy.y) > 100){
    Motor(incomingReadings.Pot,theta);
  }
  //Given Angle and Magnitude drive motors
  delay(500);
}
