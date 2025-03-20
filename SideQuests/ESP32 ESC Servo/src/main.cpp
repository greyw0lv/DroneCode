// Include the ESP32 Arduino Servo Library instead of the original Arduino Servo Library
#include <ESP32Servo.h>
#include <Streaming.h>
Servo ESC0; // create servo object to control a servo
Servo ESC1;  
Servo ESC2;
Servo ESC3;


/// Warn : should be pin 25, using pin 26 rn just to see if we need DAC or if digital out is enough.
int servoPin[] = {13, 12, 14, 26};      // GPIO pin used to connect the servo control (digital out)

int potPin = 34;        // GPIO pin used to connect the potentiometer (analog in)

int ADC_Max = 4096;     // This is the default ADC max value on the ESP32 (12 bit ADC width);
  
int val;    // variable to read the value from the analog pin

void setup()
{
  Serial.begin(115200);

  pinMode(servoPin[0], OUTPUT);
  pinMode(servoPin[1], OUTPUT);
  pinMode(servoPin[2], OUTPUT);
  pinMode(servoPin[3], OUTPUT);

  ESC0.attach(servoPin[0], 1000, 2000);   // attaches the servo in the servo object
  ESC1.attach(servoPin[1], 1000, 2000);   // attaches the servo in the servo object
  ESC2.attach(servoPin[2], 1000, 2000);   // attaches the servo in the servo object
  ESC3.attach(servoPin[3], 1000, 2000);   // attaches the servo in the servo object

}

void loop() {
  //val = analogRead(potPin);         
  //Serial.println(val);   // read the value of the potentiometer (value between 0 and 1023)
  //val = map(val, 0, ADC_Max, 0, 180);     // scale it to use it with the servo (value between 0 and 180)
  ESC1.write(10);                  // set the servo position according to the scaled value
  ESC0.write(10);
  ESC2.write(20);
  ESC3.write(20);
  Serial << ESC0.read() << "\t" <<  ESC1.read() << "\t" << ESC2.read() << "\t" <<  ESC3.read() << endl;

  //delay(500);                          // wait for the servo to get there
}
