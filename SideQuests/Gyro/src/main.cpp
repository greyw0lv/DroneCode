#include <Arduino.h>
#include "Arduino_BMI270_BMM150.h"
#include <Streaming.h>
#include <1euroFilter.h>

//stuff for 1 euro

static OneEuroFilter ax;
static OneEuroFilter ay;
static OneEuroFilter az;// not enabled yet, setup has to be called later


#define FREQUENCY   100   // [Hz] 
#define MINCUTOFF   3.0   // [Hz] needs to be tuned according to your application
#define BETA        0.1   // needs to be tuned according to your application
unsigned long start_time;
//end of 1 euro

struct threeAxis{
  float x,y,z = 0;
};
threeAxis Accel;

threeAxis filtered_Accel;



void setup() {
  Serial.begin(9600);

  ax.begin(FREQUENCY, MINCUTOFF, BETA);
  ay.begin(FREQUENCY, MINCUTOFF, BETA);
  az.begin(FREQUENCY, MINCUTOFF, BETA);
  
  start_time = micros();
  while (!Serial);
  Serial.println("Started");
  //IMU setup
  if (!IMU.begin()) {
    Serial.println("Failed to initialize IMU!");
    while (1);
  }
  //Accelerometer
  Serial.print("Accelerometer sample rate = ");
  Serial.print(IMU.accelerationSampleRate());
  Serial.println(" Hz");
  Serial.println();
  Serial.println("Acceleration in G's");
  Serial.println("X\tY\tZ");
  //end of IMU


}

void loop() {
  float elapsed_time = 1E-6 * (micros() - start_time); // in seconds

  if (IMU.accelerationAvailable()) {
    IMU.readAcceleration(Accel.x, Accel.y, Accel.z);
    filtered_Accel.x = ax.filter(Accel.x, elapsed_time);
    filtered_Accel.y = ay.filter(Accel.y, elapsed_time);
    filtered_Accel.z = az.filter(Accel.z, elapsed_time);
  }


  Serial << "Accel:" <<"\tX:"<< Accel.x <<"\tY:"<< Accel.y <<"\tZ:"<< Accel.z <<endl;

  
  float curRoll = map(filtered_Accel.x*100000.0,-1.0f,1.0f,-90.0f,90.0f)/100000.0;//dont fucky with the map
  float curYaw = map(filtered_Accel.y*100000.0,-1.0f,1.0f,-90.0f,90.0f)/100000.0;//dont fucky with the map

  Serial << "Roll:" << curRoll << "Yaw:" << curYaw << endl;
  

  delay(50);
}