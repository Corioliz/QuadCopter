
#include <gyroscope.h>

#include <accelerometer.h>

#include <magnetometer.h>

#include <i2cmaster.h>

float rate[3] = {0,0,0};
Gyroscope gyro;

float acc[3] = {0,0,0};
Accelerometer acce;

float magval[3] = {0,0,0};
Magnetometer magneto;

void setup() {
  i2c_init();
  delay(200);
  
  Serial.begin(9600);
  delay(200);
    
  Serial.println("Sensor acquisition test");
  Serial.println("Initializing...");
  gyro = Gyroscope();
  acce = Accelerometer();
  magneto = Magnetometer();
  
  gyro.initialize();
  delay(20);
  acce.initialize();
  delay(20);
  magneto.initialize();
  delay(20);
  Serial.println("Done!");
  Serial.println("Starting main loop in 1 sec");
  delay(1000);
}

void loop() {
  //Serial.println("Acce data");
  acce.getData(acc);
  delay(2);
  
  //Serial.println("Acce data");
  gyro.getSIData(rate);
  delay(2);
  
  //Serial.println("Magneto data");
  magneto.getData(magval);
  delay(2);
  
  Serial.print(acc[0]); Serial.print(",");
  Serial.print(acc[1]); Serial.print(",");
  Serial.print(acc[2]); Serial.print(",");
  Serial.print(rate[0]); Serial.print(",");
  Serial.print(rate[1]); Serial.print(",");
  Serial.print(rate[2]); Serial.print(",");
  Serial.print(magval[0]); Serial.print(",");
  Serial.print(magval[1]); Serial.print(",");
  Serial.print(magval[2]); Serial.println("");
  
}


