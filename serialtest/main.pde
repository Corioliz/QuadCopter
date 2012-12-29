//Quadcopter - 13.12.2011
//Code combining IMU, Receiver, ESCs and PID

// --- Libraries ---
#include "IMU.h"
#include "PID.h"
#include "receiver.h"

#include <Servo.h>

#if defined(ARDUINO) && ARDUINO >= 100
#include "Arduino.h"
#else
#include "WProgram.h"
#endif

#include "i2cmaster.h"

#include "gyroscope.h"
#include "accelerometer.h"
#include "magnetometer.h"

// --- Constructors --- --- --- --- ---

//IMU - wAcc , wMag
IMU imu( 0.35 , 0.85 );
//0.85 0.85

//Receiver
Receiver receiver;

//ESCs - servo
//Index - 1. front, 2. right, 3. rear, 4. left
Servo esc0; //Pin 10
Servo esc1; //Pin 11
Servo esc2; //Pin 12
Servo esc3; //Pin 13

//PID -      P, I, D,   ref, in, maxCtrl, minCtrl
PID pid_phi( 100, .5, 50.0,   0, 0,  2000, -2000 , 20, -20 ); //PID for Phi / roll ?
PID pid_theta( 0, 0.0, 0.0, 0, 0,  2000, -2000 , 200, -200); //PID for Theta / pitch ?
PID pid_psi( 0, 0.0, 0.0, 0, 0,  2000, -2000 , 200, -200 ); //PID for Psi d_body

//Calculation values
float vec[6] = {
  0, 0, 0, 0, 0, 0}; //Vector for angles (3) and ang.velocities (3) (copter coordinates)
float vec0[6] = {
  0, 0, 0, 0, 0, 0}; //Input vector for angles (3) and ang.velocities (3) (sensor coordinates)
float recvec[6] = {
  0, 0, 0, 0, 0, 0}; //Receiver - Read values
float pidref[3] = {
  0, 0, 0}; //Reference values PID - phi, theta, psi_b_d
int motorvalues[4] = {
  0, 0, 0, 0}; //Motor - Input values

float pidvalues[3] = {
  0, 0, 0}; //Auxiliary values for PID calculation
float pidthrust = 1; //Thrust level

//Ratios for Receiver values to PID Reference
float phi_ratio = 2*0.17453; //20 degrees as rads
float theta_ratio = 2*0.17453; //20 degrees as rads
float psi_ratio = 1; // ???
float thrust_ratio = 1000;

float looptime;

//Serial write
int serialReadVec[4] = {0, 0, 0, 0};

// --- --- Setup --- --- --- --- ---
void setup() {

  Serial.begin(9600); //Initialize serial

  Serial.println("Initializing...");

  // --- Initialize I2C ---
  i2c_init(); 

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

    Serial.println("ESC Pins attached");

  //  // --- Calibrate ESCs ---
  /*
  //Set highest values
  esc0.writeMicroseconds(2000);
  esc1.writeMicroseconds(2000);
  esc2.writeMicroseconds(2000);
  esc3.writeMicroseconds(2000);
  delay(2000); //Wait for 2 sec
  */
  //Set lowest values
  esc0.writeMicroseconds(1000);
  esc1.writeMicroseconds(1000);
  esc2.writeMicroseconds(1000);
  esc3.writeMicroseconds(1000);

  Serial.println("ESCs calibrated");

  // --- Initialize IMU - Acce, Gyro, Magneto ---
  imu.initialize(); 
  delay(1000); //Wait for 1 sec

  Serial.println("IMU initialized");

  // --- Receiver ---
  receiver.initialize();
  delay(1000); //Wait for 1 sec

  Serial.println("Receiver initialized");

  Serial.println("Initializing ready, waiting start command");
  
  // Start condition: Throttle low, right pot above half way
  char startFlag = 1;
  while (startFlag) {
    receiver.read(recvec);
    if (recvec[4] > 1900 && recvec[5] > 1900 && recvec[1] < 0.1) {
      startFlag = 0;
    }
    delay(100);
  }
  Serial.println("Start command received, arming engines");
  pid_phi.resetITerm();
  pid_theta.resetITerm();
  pid_psi.resetITerm();
}

// --- --- Loop --- --- --- --- ---
//Read measurement and receiver; calculate refvalues, pidvalues, motorvalues; set motorvalues
void loop() {
  looptime = millis();
  
  //Read serial for PID parameters
  if (readCsvToVector(serialReadVec)) {
    Serial.print("Serial Read: ");
    for (byte i = 0; i < 4; i++){
      Serial.print(serialReadVec[i]); Serial.print(",");
    }
    Serial.print('\n');
    pid_phi.setParameters( serialReadVec[0] , serialReadVec[1]*0.01f, serialReadVec[2], serialReadVec[3]*0.01f );
  }
    
  //Get measurement values
  imu.complementaryFilter(vec0); //Angles (3) and ang.velocities (3)
  //
  vec[0] = -vec0[1];
  vec[1] = -vec0[0];
  vec[2] = vec0[2];
  vec[3] = -vec0[4];
  vec[4] = vec0[3];
  vec[5] = vec0[5];

  //Print measurements
  //   for(int i=0; i < 3; i++){
  //   Serial.print(vec[i]);
  //   Serial.print(",");
  //   }

  //   Serial.print(vec[0]);
  //   Serial.print(",");
  //   Serial.print(vec[1]);
  //   Serial.print(",");
  //   Serial.print(vec[5]);
  //   Serial.print(",");

  //  //Read receiver values
  receiver.read(recvec); //Psi_b_d, Thrust, Theta, Phi, Cycle 1, Cycle 2
  // Thrust [0 1]
  // Psi_b_d [-1 1]
  // Theta [-1 1]
  // Phi [-1 1]
  // Cycle1/2 = [1000 2000]

  // Invert Psi_b_d
  recvec[0] = -recvec[0];
  /*for (int i = 0; i < 6; i++) {
    Serial.print(recvec[i]);
    Serial.print(',');
  }*/
 
  
  // Override receiver for testing
//  recvec[0] = 0;
//  recvec[1] = 0.5;
//  recvec[2] = 0;
//  recvec[3] = 0;
//  recvec[4] = 1950;
//  recvec[5] = 1950;
  
  //Calculate reference values and thrust value
  pidthrust = thrust_ratio * recvec[1] + 1000; //Value between 1000 and 2000
  pidref[0] = phi_ratio * recvec[3];
  pidref[1] = theta_ratio * recvec[2];
  pidref[2] = psi_ratio * recvec[0];

  //PID : calculate - ref , input
  pidvalues[0] = pid_phi.calculate( pidref[0] , vec[0] ); //Phi
  pidvalues[1] = pid_theta.calculate( pidref[1] , vec[1] ); //Theta
  pidvalues[2] = pid_psi.calculate( pidref[2] , vec[5] ); //Psi

  //Motors : calculate motor values
  // X style : Index : 0. left front, 1. right front, 2. right rear, 3. left rear
  motorvalues[0] = (int)( pidthrust + pidvalues[0] - pidvalues[1] - pidvalues[2] );
  motorvalues[1] = (int)( pidthrust - pidvalues[0] - pidvalues[1] + pidvalues[2] );
  motorvalues[2] = (int)( pidthrust - pidvalues[0] + pidvalues[1] - pidvalues[2] );
  motorvalues[3] = (int)( pidthrust + pidvalues[0] + pidvalues[1] + pidvalues[2] );

//  // + style : Index : 0. front, 1. right, 2. rear, 3. left
//  motorvalues[0] = (int)( pidthrust - pidvalues[1] - pidvalues[2] );
//  motorvalues[1] = (int)( pidthrust - pidvalues[0] + pidvalues[2] );
//  motorvalues[2] = (int)( pidthrust + pidvalues[1] - pidvalues[2] );
//  motorvalues[3] = (int)( pidthrust + pidvalues[0] + pidvalues[2] );

  // Check motor values
  for(int i=0; i < 4; i++){
    if(motorvalues[i] > 1850){
      motorvalues[i] = 1850; //Max value is 2000
    } 
    
    if( recvec[4] < 1900 || recvec[5] < 1900 || motorvalues [i] < 1000){
      motorvalues[i] = 1000; //Min value is 1000
    }
    Serial.print(motorvalues[i]);
    Serial.print(",");
  }
  //Serial.println(0);
    //Set motor values
  esc0.writeMicroseconds(motorvalues[0]); //Index 0
  esc1.writeMicroseconds(motorvalues[1]); //Index 1
  esc2.writeMicroseconds(motorvalues[2]); //Index 2
  esc3.writeMicroseconds(motorvalues[3]); //Index 3
  Serial.println(millis()-looptime);
}

// This is for reading from serial a comma separated list of integers
boolean readCsvToVector(int* pidVector) {
  byte len; 
  if (Serial.available() <= 0)
    return 0;
  delay(10);
  len = Serial.available();
  char stream[20];
  char number[5];
  // Read the message
  for (byte i = 0; i < len; ++i) {
    stream[i] = Serial.read();
  }
  byte i = 0, j = 0, k = 0;
  while ( i < len ) {
    while (stream[i] != ',' && i < len) {
      number[j] = stream[i];
      ++i;
      ++j;
    }
    ++i;
    number[j] = '\0';
    j = 0;
    pidVector[k++] = atoi(number);
  }
  return 1;
}

