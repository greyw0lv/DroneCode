#include "Arduino.h"
#include <esp_now.h>
#include <WiFi.h>
#include <Streaming.h>

#include <Wire.h>

#define SCREEN_WIDTH 128  // OLED display width, in pixels
#define SCREEN_HEIGHT 64  // OLED display height, in pixels

/*
I think theres something wrong with joystick, sensitivity seems strange.A0
TODO: potentiometer, as float
*/


int PIN_X = 32;
int PIN_Y = 33;
int PIN_P = 36;
int PIN_Z = 34;
int PIN_R = 35;

int xPos, yPos = 0;



// REPLACE WITH THE MAC Address of your receiver 
//88:13:bf:c8:57:9c
uint8_t broadcastAddress[] = {0x88, 0x13, 0xbf, 0xc8, 0x57, 0x9c};

// Variable to store if sending data was successful
String success;

//Structure example to send data
//Must match the receiver structure
struct Vec4{
  float x,y,z,r;
};

struct struct_message {
    Vec4 Joy;
    double Pot;
};

// Create a struct_message called RemoteReadings to hold sensor readings
struct_message RemoteReadings;

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

//sens is weird when float, unsure why.
float getJoy(int PinOut){
  Serial.print("--getting Joy:");
  Serial.println(PinOut);
  const int deadzone = 200; // Deadzone threshold (adjust as needed)
  const int center = 1824;  // 1824? Center position for analog joystick
  float position = 0.0;
  int Axis = analogRead(PinOut);// Raw value from gpio
  
  if (abs(Axis - center) > deadzone){
    position = map(Axis,0+660, 4095-660, -1.0, 1.0);
  }
  return position;
}

double getPot(int PinOut){
  Serial.print("--getting Pot:");
  Serial.println(PinOut);
  const int deadzone = 0; // Deadzone threshold (adjust as needed)
  //const int center = 0;  // 1824? Center position for analog joystick
  double power = 0.0;
  int Axis = analogRead(PinOut);// Raw value from gpio
  
  if (abs(Axis) > deadzone){
    power = map(Axis*100000.0,0, 4095, 0.00, 1.00)/100000.0;
  }
  return power;
}

void getReadings(){
  Serial.println("--getting Readings--");
  // Set values to send
  RemoteReadings.Joy.x = getJoy(PIN_X);
  RemoteReadings.Joy.y = getJoy(PIN_Y);
  RemoteReadings.Joy.z = getJoy(PIN_Z);
  RemoteReadings.Joy.r = getJoy(PIN_R);
  RemoteReadings.Pot = getPot(PIN_P);
}


void setup() {
  // Init Serial Monitor
  Serial.begin(115200);
  pinMode(PIN_P, INPUT);
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

}
 
void loop() {
  getReadings();

  Serial << "JoystickRAW\t X:" << analogRead(PIN_X) << "\tY:" << analogRead(PIN_Y) << endl;
  Serial << "JoystickClean\t X:" << RemoteReadings.Joy.x << "\tY:" << RemoteReadings.Joy.y << endl;
  Serial << "JoystickRAW\t Z:" << analogRead(PIN_Z) << "\tR:" << analogRead(PIN_R) << endl;
  Serial << "JoystickClean\t Z:" << RemoteReadings.Joy.z << "\tR:" << RemoteReadings.Joy.z << endl;
  Serial << "Pot:" << RemoteReadings.Pot << "\n" << endl; 
  // Send message via ESP-NOW
  esp_err_t result = esp_now_send(broadcastAddress, (uint8_t *) &RemoteReadings, sizeof(RemoteReadings));

  if (result == ESP_OK) {
    Serial.println("Sent with success");
  }
  else {
    Serial.println("Error sending the data");
  }
  delay(500);
}
