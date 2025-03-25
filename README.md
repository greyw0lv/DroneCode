# DroneCode
 
# Main directory contains 4 projects.

## Drone
The code for the ESP32 mounted physically on the DRONE. Contains control logic, and I/O

## Gyro
Code used to read the Nano's onboard IMU and export the data across serial TX pin.

## MotorDriver
Code used in the Uno, primary role is reading serial and sending out PWM signals. Additonally extra sensors were added to this board to satify sensors' course project.

## Remote
Code used in the ESP32 mounted on the REMOTE CONTROLLER breadboard. this reads the input from 2 XY axis joysticks, and 1 potentiometer, and sends the data as a struct to the drone ESP.
