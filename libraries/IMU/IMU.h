#ifndef IMU_h
#define IMU_h

#if defined(ARDUINO) && ARDUINO >= 100
  #include "Arduino.h"
  #else
  #include "WProgram.h"
#endif

#include "i2cmaster.h"
#include "gyroscope.h"
#include "accelerometer.h"
#include "magnetometer.h"

class IMU
{
	private:
		Gyroscope gyroscope;
		Accelerometer accelerometer;
		Magnetometer magneto;
		float attitude[3];
		float attitudePrev[3];
		float attAcc[3];
		float attGyro[3];
		
		float c_alpha;
		float s_alpha;
		
		float acc[3];
		float rates[3];
		float mag[3];
		
		float wGyroAcc;
		float wGyroMag;
		float wAcc;
		float wMag;
		
		float Axz;
		float Ayz;
		
		float radToDeg;
		float degToRad;
		
		unsigned long lastMillis;
		float dT;
		char firstRound;
		
		void normalize(float* vec);
		float square(float num);
				
	public:
		void complementaryFilter(float* vec);
		IMU(float wAcc, float wMag);
		void initialize();
		void getAttitude(float* vec);
		void getRates(float* Rates);
		void getAccs(float* Accs);
		void getMags(float* Mags);
};

#endif
