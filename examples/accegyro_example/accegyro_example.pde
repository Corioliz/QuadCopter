#include <gyroscope.h>

#include <i2cmaster.h>

#include <accelerometer.h>

float rate[3];
Gyroscope gyro;

float acc[3];
Accelerometer acce;

void setup() {
  i2c_init();
  Serial.begin(9600);
  Serial.println("Sensor acquisition test");
  Serial.println("Initializing...");
  gyro = Gyroscope();
  acce = Accelerometer();
  
  gyro.initialize();
  acce.initialize();
  Serial.println("Done!");
  Serial.println("Starting main loop in 1 sec");
  delay(1000);
}

void loop() {
  acce.getData(acc);
  gyro.getData(rate);
  
  Serial.print(acc[0]); Serial.print(",");
  Serial.print(acc[1]); Serial.print(",");
  Serial.print(acc[2]); Serial.print(",");
  Serial.print(rate[0]); Serial.print(",");
  Serial.print(rate[1]); Serial.print(",");
  Serial.print(rate[2]); Serial.println("");
}


