#include <i2cmaster.h>

#include <gyroscope.h>

float rate[3];
Gyroscope gyro;

void setup() {
  i2c_init();
  Serial.begin(9600);
  Serial.println("testi");
  gyro = Gyroscope();
  gyro.initialize();
}

void loop() {
  gyro.getData(rate);
  Serial.println(rate[0]);
  delay(25);
}


