
#include "Arduino.h"
#include <esp_now.h>
#include <WiFi.h> //dunno if this one needs to be here.
#include <Streaming.h>



// REPLACE WITH THE MAC Address of your receiver 
//88:13:bf:c8:54:64
uint8_t broadcastAddress[] = {0x88, 0x13, 0xbf, 0xc8, 0x54, 0x64};

// Define variables to store Remote readings to be sent

// Define variables to store incoming readings

// Variable to store if sending data was successful
String success;

//Structure example to send data


//Must match the receiver structure


const float BASELINE = 0.5;
float motorPower[4] = {BASELINE,BASELINE,BASELINE,BASELINE};

float curRoll;
float curYaw;
float ESC[4] = {0};

struct Vec4{
  float x,y,z,r;
};

struct struct_message {
  Vec4 Joy;
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
 
float radiansFromPosition(Vec4 v){
  float theta = atan2(v.x, v.y) /(2*3.1415);
  return theta;
}

float mapfloat(float x, float in_min, float in_max, float out_min, float out_max){
  return (x - in_min) * (out_max - out_min) / (in_max - in_min) + out_min;
}
//-------KEY------//
//   NW NE | 1 0  //
//   SW SE | 2 3  //
//----------------//
void droneForward(){
  Serial.println("Forwarding");
  
  float normYAW = mapfloat(curYaw,-90,90,-1,1);
  //Front
  for (int i = 0; i < 2; i++)
  {
    ESC[i] = ESC[i] + (2*(normYAW - (incomingReadings.Joy.x)));//10% duty +- 2%, ofset by joystick by 10deg
  }
  //Back
  for (int i = 2; i < 4; i++)
  {
    ESC[i] = ESC[i] - (2*(normYAW - (incomingReadings.Joy.x)));//10% duty
  }

}
void droneStrafe(){
  Serial.println("Lefting");
  float normRoll = mapfloat(curRoll,-90,90,-1,1);
  //Side
  for (int i = 0; i < 4; i++)
  {
    if (i > 0 && i < 3)
    {//Left //1,2
      ESC[i] = ESC[i] + (2*(normRoll - (incomingReadings.Joy.y)));//10% duty
    } else
    {//Right // 0,3
      ESC[i] = ESC[i] - (2*(normRoll - (incomingReadings.Joy.y)));//10% duty
    }

  }
}
void droneTwist(){
  Serial.println("Twisting");
  //Twist
  for (int i = 0; i < 4; i++)
  {
    if (i % 2)
    {//0,2
      ESC[i] = ESC[i] + ((incomingReadings.Joy.r/50));//10% duty
    } else
    {//1,3
      ESC[i] = ESC[i] - ((incomingReadings.Joy.r/50));//10% duty
    }

  }
}

void droneAmplify(){
  for (int i = 0; i < 4; i++)
  {
    ESC[i] = ESC[i] *2* incomingReadings.Pot;
  }
  
}
void droneReset(){
  for (int i = 0; i < 4; i++)
  {
    ESC[i] = 50;//10% duty
  }
}

void updateMotor(){

  droneReset();
  droneForward();
  droneStrafe();
  droneTwist();
  droneAmplify();
  


}

void setup() {
  // Init Serial Monitor
  Serial.begin(115200);
  Serial2.begin(9600,SERIAL_8N1,16,17);

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

  //Get Signals
  //Done
  //Get Serial Data || NANO
  byte buffer[8]; // Buffer to store incoming bytes
  if (Serial2.readBytes(buffer, 8) == 8) { // Wait for 8 bytes
    float* received = (float*)buffer; // Cast bytes to float array
    //int* temp = (int*)buffer;
    //Serial.print(temp[0]);
    //Serial.print("\t");
    //Serial.println(temp[1]);
    curRoll= received[0];
    curYaw = received[1];
    Serial.printf("Received: %.2f, %.2f,\n", 
                  received[0], received[1]);
  }
  
  //Get User Intent || ESP REMOTE
  //float theta = radiansFromPosition(incomingReadings.Joy);
  //Serial.println(theta);
  //incomingReadings.Pot = 1.0;//DELETE THIS  
  
  //Updated Intent
  updateMotor();
  
  Serial.printf("Received: %.2f, %.2f,%.2f, %.2f\n", 
    ESC[0], ESC[1],ESC[2],ESC[3]);

  float results[4] = {ESC[0],ESC[1],ESC[2],ESC[3]};
  Serial2.write((byte*)results, sizeof(results)); // Send raw bytes



  delay(500);
}
