/*
  Rui Santos & Sara Santos - Random Nerd Tutorials
  Complete project details at https://RandomNerdTutorials.com/esp-now-two-way-communication-esp32/
  Permission is hereby granted, free of charge, to any person obtaining a copy of this software and associated documentation files.
  The above copyright notice and this permission notice shall be included in all copies or substantial portions of the Software.
*/
#include "Arduino.h"
#include <esp_now.h>
#include <WiFi.h> //dunno if this one needs to be here.
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


const float BASELINE = 0.5;
float motorPower[4] = {BASELINE,BASELINE,BASELINE,BASELINE};


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

//-------KEY------//
//   NW NE | 1 0  //
//   SW SE | 2 3  //
//----------------//
void droneHover(){
  Serial.println("Hovering");
}
void droneForward(bool isForward){
  Serial.println("Forwarding");
}
void droneStrafe(bool isLeft){
  Serial.println("Lefting");
}

void updateDrone(){

}

void setup() {
  // Init Serial Monitor
  Serial.begin(115200);
  Serial2.begin(9600,SERIAL_8N1,16,17);
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
  //Get Serial Data
  byte buffer[8]; // Buffer to store incoming bytes
  if (Serial2.readBytes(buffer, 8) == 8) { // Wait for 8 bytes
    float* received = (float*)buffer; // Cast bytes to float array
    //int* temp = (int*)buffer;
    //Serial.print(temp[0]);
    //Serial.print("\t");
    //Serial.println(temp[1]);
    Serial.printf("Received: %.2f, %.2f,\n", 
                  received[0], received[1]);
  }


  //Get User Intent
  //float theta = radiansFromPosition(incomingReadings.Joy);
  //Serial.println(theta);
  //incomingReadings.Pot = 1.0;//DELETE THIS  

  //Get Signals
  //float curRoll= analogRead(32);
  //float curYaw = analogRead(33);

  //Updated Intent
  //updateDrone(); //Moved to UNO
  float ESC1 = 2.0;
  float ESC2 = 5.0;
  float ESC3 = 7.0;
  float ESC4 = 9.0;


  float results[4] = {ESC1, ESC2, ESC3, ESC4};
  Serial2.write((byte*)results, sizeof(results)); // Send raw bytes



  delay(500);
}
