#include <Arduino.h>
#include "Arduino_BMI270_BMM150.h"
#include "SimpleKalmanFilter.h"
#include "SensorFusion.h"

#include <Streaming.h>

SimpleKalmanFilter gyroKalmanFilterx(1, 1, 0.01);
SimpleKalmanFilter gyroKalmanFiltery(1, 1, 0.01);
SimpleKalmanFilter gyroKalmanFilterz(1, 1, 0.01);
SimpleKalmanFilter accelKalmanFilterx(2, 1, 0.02);
SimpleKalmanFilter accelKalmanFiltery(2, 1, 0.02);
SimpleKalmanFilter accelKalmanFilterz(2, 1, 0.02);
SimpleKalmanFilter magnetoKalmanFilterx(0.7, 1, 0.01);
SimpleKalmanFilter magnetoKalmanFiltery(0.7, 1, 0.01);
SimpleKalmanFilter magnetoKalmanFilterz(0.7, 1, 0.01);

struct threeAxis{
  float x,y,z;
};

threeAxis Gyro;
threeAxis Accel;
threeAxis Magneto;

threeAxis Estimated_Gyro;
threeAxis Estimated_Accel;
threeAxis Estimated_Magneto;

float pitch, roll, yaw;
float deltat;

SF fusion; //creates fusion object

auto gyroOffset = [](threeAxis v) {
  v.x = v.x - -1.0;
  v.y = v.y -  0.2;
  v.z = v.z - -0.4;
  return (v);
};

auto accelOffset = [](threeAxis v) {
  v.x = v.x -  0;
  v.y = v.y -  0;
  v.z = v.z +  1.0;
  return (v);
};

auto accelDeadzone = [](threeAxis v) {
  if (abs(v.x) < 0.02){v.x = 0;};
  if (abs(v.y) < 0.02){v.y = 0;};
  if (abs(v.z) < 0.03){v.z = 0;};
  return (v);
};

auto gyroDeadzone = [](threeAxis v) {
  if (abs(v.x) < 0.13){v.x = 0;};
  if (abs(v.y) < 0.13){v.y = 0;};
  if (abs(v.z) < 0.13){v.z = 0;};
  return (v);
};

void setup() {
  Serial.begin(9600);
  while (!Serial);
  Serial.println("Started");
  //IMU setup
  if (!IMU.begin()) {
    Serial.println("Failed to initialize IMU!");
    while (1);
  }
  //Gyro
  Serial.print("Gyroscope sample rate = ");
  Serial.print(IMU.gyroscopeSampleRate());
  Serial.println(" Hz");
  Serial.println();
  Serial.println("Gyroscope in degrees/second");
  Serial.println("X\tY\tZ");
  //MagnetoMeter
  Serial.print("Magnetic field sample rate = ");
  Serial.print(IMU.magneticFieldSampleRate());
  Serial.println(" Hz");
  Serial.println();
  Serial.println("Magnetic Field in uT");
  Serial.println("X\tY\tZ");
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

  if (IMU.gyroscopeAvailable()) {
    IMU.readGyroscope(Gyro.x, Gyro.y, Gyro.z);
    Estimated_Gyro.x = gyroKalmanFilterx.updateEstimate(Gyro.x);
    Estimated_Gyro.y = gyroKalmanFiltery.updateEstimate(Gyro.y);
    Estimated_Gyro.z = gyroKalmanFilterz.updateEstimate(Gyro.z);

  }
  if (IMU.accelerationAvailable()) {
    IMU.readAcceleration(Accel.x, Accel.y, Accel.z);
    Estimated_Accel.x = accelKalmanFilterx.updateEstimate(Accel.x);
    Estimated_Accel.y = accelKalmanFiltery.updateEstimate(Accel.y);
    Estimated_Accel.z = accelKalmanFilterz.updateEstimate(Accel.z);

  }
  if (IMU.magneticFieldAvailable()) {
    IMU.readMagneticField(Magneto.x, Magneto.y, Magneto.z);
    Estimated_Magneto.x = magnetoKalmanFilterx.updateEstimate(Magneto.x);
    Estimated_Magneto.y = magnetoKalmanFiltery.updateEstimate(Magneto.y);
    Estimated_Magneto.z = magnetoKalmanFilterz.updateEstimate(Magneto.z);

  }
  Estimated_Gyro   = gyroOffset(Estimated_Gyro);
  Estimated_Accel  = accelOffset(Estimated_Accel); //doesnt seem to have 
  //Estimated_Magneto= (Estimated_Magneto);

  Estimated_Gyro   = gyroDeadzone(Estimated_Gyro);
  Estimated_Accel  = accelDeadzone(Estimated_Accel);
  //Estimated_Magneto= deadzone(Estimated_Magneto);

  
  //Serial << "Gyro: " <<"\tX:"<< Estimated_Gyro.x <<"\tY:"<< Estimated_Gyro.y <<"\tZ:"<< Estimated_Gyro.z <<endl;
  Serial << "Accel:" <<"\tX:"<< Estimated_Accel.x <<"\tY:"<< Estimated_Accel.y <<"\tZ:"<< Estimated_Accel.z <<endl;
  //Serial << "Magne:" <<"\tX:"<< Estimated_Magneto.x <<"\tY:"<< Estimated_Magneto.y <<"\tZ:"<< Estimated_Magneto.z <<endl<<endl;
  


  
  deltat = fusion.deltatUpdate();
  fusion.MahonyUpdate(Estimated_Gyro.x, Estimated_Gyro.y, Estimated_Gyro.z, Estimated_Accel.x, Estimated_Accel.y, Estimated_Accel.x, Estimated_Magneto.x, Estimated_Magneto.y, Estimated_Magneto.z, deltat);  //mahony is suggested if there isn't the mag
  //fusion.MadgwickUpdate(Estimated_Gyro.x, Estimated_Gyro.y, Estimated_Gyro.z, Estimated_Accel.x, Estimated_Accel.y, Estimated_Accel.x, Estimated_Magneto.x, Estimated_Magneto.y, Estimated_Magneto.z, deltat);  //else use the magwick

  roll = fusion.getRoll();
  pitch = fusion.getPitch();
  yaw = fusion.getYaw();

  //Serial << "Pitch:\t" << pitch << "\t\tRoll:\t" << roll << "\t\tYaw:\t" << yaw << endl;
  delay(5);
}