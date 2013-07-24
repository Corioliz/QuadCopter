#include "IMU.h"

IMU::IMU(float wAcc, float wMag) {
	// wGyro and wAcc must sum up to 1
	IMU::wAcc = wAcc;
	IMU::wMag = wMag;
	IMU::wGyroAcc = 1-wAcc;
	IMU::wGyroMag = 1-wMag;
	
	// flag for getAttitude function
	firstRound = 1;
	
	// conversion constants
	radToDeg = 180/PI;
	degToRad = PI/180;
	
	// sensor objects
	accelerometer = Accelerometer();
	gyroscope = Gyroscope();
	//magneto = Magnetometer();
	
	//Magnet field inclination Cos and Sin for alpha = 73 degrees
	c_alpha = 0.29237;
	s_alpha = 0.95630;
}

void IMU::initialize() {
	accelerometer.initialize();
	gyroscope.initialize();
	lastMillis = millis();
}

void IMU::complementaryFilter(float* vec) {
	unsigned long currentMillis = millis();
	// Step 1: Read sensor values
	accelerometer.getData(acc);
	delay(2);
	gyroscope.getSIData(rates);
	delay(2);
	delay(2);
	
	normalize(acc);
	
	// Step 1.5: Calculate time interval (maybe this should actually be fixed?)
	dT = (currentMillis - lastMillis) * 0.001f;
	lastMillis = currentMillis;
	
	// Step 2a: Calculate pitch and roll angles from accelerometer readings
	float R = 1.0/sqrt(square(acc[0])+square(acc[1])+square(acc[2])); // Accelerometer vector length inverse
	float pitchAcc = asin( acc[1] * R );
	float rollAcc = atan2( acc[0] , acc[2] );

	// Step 3: filtering
	vec[0] = wGyroAcc * (vec[0] + rates[0]*dT) + wAcc * rollAcc; // roll (phi)
	vec[1] = wGyroAcc * (vec[1] + rates[1]*dT) + wAcc * pitchAcc; // pitch (theta)
	vec[2] = 0;
	vec[3] = rates[0];
	vec[4] = rates[1];
	vec[5] = rates[2];
	
}

void IMU::getAttitude(float* vec) {
	
}

void IMU::normalize(float* vec) {
	float length = sqrt(vec[0]*vec[0] + vec[1]*vec[1] + vec[2]*vec[2]);
	vec[0] /= length;
	vec[1] /= length;
	vec[2] /= length;
}

void IMU::getRates(float* Rates) {
	//Rates = rates;
	Rates[0] = rates[0];
	Rates[1] = rates[1];
	Rates[2] = rates[2];
}

void IMU::getAccs(float* Accs) {
	//Rates = rates;
	Accs[0] = acc[0];
	Accs[1] = acc[1];
	Accs[2] = acc[2];
}

void IMU::getMags(float* Mags) {
	//Rates = rates;
	Mags[0] = mag[0];
	Mags[1] = mag[1];
	Mags[2] = mag[2];
}

inline float IMU::square(float num) {
	return num*num;
}
	
