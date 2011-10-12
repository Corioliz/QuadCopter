#ifndef IMU_h
#define IMU_h

#include "Wprogram.h"
#include "i2cmaster.h"
#include "gyroscope.h"
#include "accelerometer.h"

class IMU
{
	private:
		Gyroscope gyroscope;
		Accelerometer accelerometer;
		float attitude[3];
		float attitudePrev[3];
		float attAcc[3];
		float attGyro[3];
		
		float acc[3];
		float rates[3];
		
		float wGyro;
		float wAcc;
		
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
		IMU(float wGyro, float wAcc);
		void initialize();
		void getAttitude(float* vec);
		void getRates(float* Rates);
};

#endif
