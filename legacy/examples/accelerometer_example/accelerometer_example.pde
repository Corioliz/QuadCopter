#include <i2cmaster.h>
#include <accelerometer.h>


float acc[3];
Accelerometer acce;

void setup() {
  Serial.begin(9600);
  Serial.println("Testi");
  i2c_init();
  acce = Accelerometer();
  acce.initialize();
  

}

void loop() {
  acce.getData(acc);
  Serial.println(acc[0]);
  Serial.println(acc[1]);
  Serial.println(acc[2]);
  delay(250);
}
