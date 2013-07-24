/**
 * To calibrate use this program. Perform the following
 * 1. Connect ardiuino board to USB
 * 2. Throttle to full
 * 3. Connect battyry
 * 4. ESC says : 123 -> beep x2 
 * 5. Throttle to zero
 * 6. ESC says : beep x3 -> long beep
 * 7. Disconnect battery
 * 8: Disconnect USB
 * 9: Calibration ready (hopefully)
 **/


// --- Libraries ---
#if defined(ARDUINO) && ARDUINO >= 100
#include "Arduino.h"
#else
#include "WProgram.h"
#endif

#include <Servo.h>
#include "receiver.h"

Receiver receiver;

//ESCs - servo
//Index - 1. front, 2. right, 3. rear, 4. left
Servo esc0; //Pin 10
Servo esc1; //Pin 11
Servo esc2; //Pin 12
Servo esc3; //Pin 13

float looptime;
float recvec[6] = {
  0, 0, 0, 0, 0, 0}; //Receiver - Read values
int motorvalues[4] = {
  0, 0, 0, 0}; //Motor - Input values

void setup()  {

  Serial.begin(9600); //Initialize serial
  Serial.println("Initializing...");

   // --- Initialize ESCs ---
  //Set pins 10-13 for ESCs as outputs
  pinMode(10,OUTPUT);
  pinMode(11,OUTPUT);
  pinMode(12,OUTPUT);
  pinMode(13,OUTPUT);
  //Attach pins for ESCs (sets motor orientation)
  esc0.attach(11); //ESC 0 - Pin 11
  esc1.attach(10); //ESC 1 - Pin 10
  esc2.attach(13); //ESC 2 - Pin 13
  esc3.attach(12); //ESC 3 - Pin 12
  
  receiver.initialize();
  delay(1000);
}

void loop()  {
  looptime = millis();
  
  receiver.read(recvec); //Psi_b_d, Thrust, Theta, Phi, Cycle 1, Cycle 2
  if (recvec[1] > 0.5)
    for (int i = 0; i < 4; i++)
      motorvalues[i] = 2000;
  else
    for (int i = 0; i < 4; i++)
      motorvalues[i] = 1000;
  for (int i = 0; i < 4; i++)  {
    Serial.print(motorvalues[i]);
    Serial.print(',');
  }
  esc0.writeMicroseconds(motorvalues[0]); //Index 0
  esc1.writeMicroseconds(motorvalues[1]); //Index 1
  esc2.writeMicroseconds(motorvalues[2]); //Index 2
  esc3.writeMicroseconds(motorvalues[3]); //Index 3
    

  Serial.println(millis()-looptime);

}
