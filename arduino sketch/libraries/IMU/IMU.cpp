#include "IMU.H"

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
	gyroscope = Gyroscope();
	accelerometer = Accelerometer();
	magneto = Magnetometer();
	
	//Magnet field inclination Cos and Sin for alpha = 73 degrees
	c_alpha = 0.29237;
	s_alpha = 0.95630;
}

void IMU::initialize() {
	gyroscope.initialize();
	accelerometer.initialize();
	magneto.initialize();
	lastMillis = millis();
}

void IMU::complementaryFilter(float* vec) {
	unsigned long currentMillis = millis();
	// Step 1: Read sensor values
	accelerometer.getData(acc);
	delay(2);
	//acc[2] *= -1; // Invert acc z-axis so the vector now points up
	gyroscope.getSIData(rates);
	delay(2);
	magneto.getData(mag);
	delay(2);
	//acc[2] *= -1;
	
	normalize(acc);
	
	//acc[0] = -acc[0];
	//acc[1] = -acc[1];
	
	//Magnetometer offsets
	mag[0] = -(mag[0] + 50); //Invert x-axis
	mag[1] = -(mag[1] + 20); //Invert y-axis
	mag[2] = mag[2] + 50;
	//Normalize magnetometer
	normalize(mag);
	
	// Step 1.5: Calculate time interval (maybe this should actually be fixed?)
	dT = (currentMillis - lastMillis) * 0.001f;
	lastMillis = currentMillis;
	// Step 2a: Calculate pitch and roll angles from accelerometer readings
	float R = 1.0/sqrt(square(acc[0])+square(acc[1])+square(acc[2])); // Accelerometer vector length inverse
	//float pitchAcc = acos(acc[1]*R); // Rotation around y-axis
	//float rollAcc = acos(acc[0]*R); // Rotation around x-axis
	
	float pitchAcc = asin( acc[1] * R );
	float rollAcc = atan2( acc[0] , acc[2] );
	//float rollAcc = atan( acc[0] / acc[2] );
	
	//YPR alternative
	//float rollAcc = atan2( -acc[2] , acc[1] );
	
	// Step 3: filtering
	vec[0] = wGyroAcc * (vec[0] + rates[0]*dT) + wAcc * rollAcc; // roll (phi)
	vec[1] = wGyroAcc * (vec[1] + rates[1]*dT) + wAcc * pitchAcc; // pitch (theta)
	
	float sT = sin(vec[1]);
	float cT = cos(vec[1]);
	float sP = sin(vec[0]);
	float cP = cos(vec[0]);
	
	float apuA = - cP * mag[1] + sP * mag[2] ;
	float apuB = cT * mag[0] + sT * sP * mag[1] + sT * cP * mag[2] ;
	
	float yawMag = atan2( apuA , apuB );
	//float yawMag = atan( apuA / apuB );
	
	//float cos_che = cos(yawMag) * apuB + sin(yawMag) * apuA ;
	//Serial.println(cos_che);
	
	//vec[0] = vec[0];
	
	/*
	float sP = sin(vec[0]);
	float cP = cos(vec[0]);
	
	float pitchAcc = acos( acc[1] / sP );
	
	vec[1] = wGyroAcc * (vec[1] + rates[1]*dT) + wAcc * pitchAcc; // pitch
	
	vec[1] = vec[1] + PI/2;
	
	float sT = sin(vec[1]);
	float cT = cos(vec[1]);
	
	float apuA = sP * sT * c_alpha;
	float apuB = cP * c_alpha;
	float apuC = sP * cT * s_alpha;
	float apuD = cP * cT * s_alpha;
	float apuE = cP * sT * c_alpha;
	float apuF = sP * c_alpha;
	
	float yawMag = asin( (apuA*(-mag[2] - apuD) - apuE*(-mag[1] - apuC) )/(apuB * apuE + apuA * apuF) );
	*/
	
	// Tilt compensation for compass
	/*
	float sT = sin(vec[0] - PI*0.5);
	float cT = cos(vec[0]  - PI*0.5);
	float sP = sin(vec[1]  - PI*0.5);
	float cP = cos(vec[1] - PI*0.5);
	//
	float sT = sin(vec[0]);
	float cT = cos(vec[0]);
	float sP = sin(-vec[1]);
	float cP = cos(-vec[1]);
	*/
	//float yawMag = atan2(mag[2]*sP+mag[0]*cP, -mag[1]*cT-mag[0]*sT*sP+mag[2]*sT*cP);
	//float yawMag = - atan2( mag[1] * cT * sP + mag[2] * sT , mag[0] * cP + mag[1] * sT * sP - mag[2] * cT * sP );
	
	//float X_H = - mag[1] * cP - mag[0] * sT * sP + mag[2] * cT * sP ;
	//float Y_H = - mag[0] * cT - mag[2]*sT ;
	
	//float yawMag = - atan2( Y_H , X_H );
	//float yawMag = - atan2( -mag[0] * cT + mag[2]*sT , - mag[1] * cT * sP - mag[0] * sT * sP - mag[2] * cT * sP );
	
	/*
	if ( X_H < 0 ) {
	
		yawMag = PI - yawMag;
		
	} else if ( X_H > 0 ) {
		
		if( Y_H < 0) {
			yawMag = - yawMag;
		} else {
			yawMag = 2*PI - yawMag;
		}
		
	} else {
		
		if( Y_H < 0) {
			yawMag = 0.5 * PI;
		} else {
			yawMag = 1.5 * PI;
		}
		
	}
	*/
	// Yaw update based only on gyro
	vec[2] = wGyroMag * (vec[2] + rates[2]*dT) + wMag * yawMag;
	
	//Gyro - angular velocities
	// vec[3] = rates[0] ;
	// vec[4] = rates[1] ;
	float lpCoeff = 0.5;
	vec[3] = (1-lpCoeff)*vec[3] + lpCoeff*rates[0];
	vec[4] = (1-lpCoeff)*vec[4] + lpCoeff*rates[1];
	vec[5] = (1-lpCoeff)*vec[5] + lpCoeff*rates[2];
	
}

void IMU::getAttitude(float* vec) {
	/*unsigned long currentMillis;
	unsigned long interval;
	float tmp;
	
	currentMillis = millis();	
	// Read sensor data
	accelerometer.getData(acc);
	acc[2] *= -1; // Invert accelerometer z-axis
	gyroscope.getData(rates);
	dT = (currentMillis - lastMillis) * 0.001f;
	lastMillis = currentMillis;	
	normalize(acc);
	
	if (firstRound) {
		// Initialize previous estimate 
		attitude[0] = acc[0];
		attitude[1] = acc[1];
		attitude[2] = acc[2];
		
		firstRound = 0;
	} else {
		if (abs(attitude[2]) < 0.01) {
			// Skip gyro because of numerical instability
			rates[0] = attitude[0];
			rates[1] = attitude[1];
			rates[2] = attitude[2];
		} else {
			// Process gyro data
			Axz = atan2(attitude[0],attitude[2]);
			Axz += rates[0] * degToRad * dT;
			Ayz = atan2(attitude[1],attitude[2]);
			Ayz += rates[1] * degToRad * dT;
			char gyroZsign = (cos(Axz) >= 0) ? 1 : -1;
			
			attGyro[0] = sin(Axz) / sqrt(1 + square(cos(Axz)) + square(tan(Ayz)));
			attGyro[1] = sin(Ayz) / sqrt(1 + square(cos(Ayz)) + square(tan(Axz)));
			attGyro[2] = gyroZsign * sqrt(1 - square(attGyro[0]) - square(attGyro[1]));
					
		}
		// Combine data
		attitude[0] = (acc[0] + attGyro[0]*wGyro)/(1+wGyro);
		attitude[1] = (acc[1] + attGyro[1]*wGyro)/(1+wGyro);
		attitude[2] = (acc[2] + attGyro[2]*wGyro)/(1+wGyro);
	
		normalize(attitude);
		
		vec[0] = attitude[0];
		vec[1] = attitude[1];
		vec[2] = attitude[2];	
		return;
	}
	
	/*	
	// Process gyroscope data
	float Axz = Axz_prev + rates[0] * dT;
	float Ayz = Ayz_prev + rates[1] * dT;
	
	float cotAxz = 1 / tan(Axz);
	float secAxz = 1 / cos(Axz);
	float cotAyz = 1 / tan(Ayz);
	float secAyz = 1 / cos(Ayz);
	
	attGyro[0] = 1 / sqrt(1 + cotAxz*cotAxz + secAyz*secAyz);
	attGyro[1] = 1 / sqrt(1 + cotAyz*cotAyz + secAxz*secAxz);
	
	attGyro[2] = sqrt(1 - attGyro[0]*attGyro[0] - attGyro[1]*attGyro[1]);
	
	if (attitudePrev[2] < 0)
		attGyro[2] = -1 * attGyro[2];
		
	// Combine data
	
	attitude[0] = (attAcc[0] + attGyro[0]*wGyro)/(1+wGyro);
	attitude[1] = (attAcc[1] + attGyro[1]*wGyro)/(1+wGyro);
	attitude[2] = (attAcc[2] + attGyro[2]*wGyro)/(1+wGyro);
	
	normalize(attitude);
	
	vec[0] = attitude[0];
	vec[1] = attitude[1];
	vec[2] = attitude[2];
	
	attitudePrev[0] = attitude[0];
	attitudePrev[1] = attitude[1];
	attitudePrev[2] = attitude[2];
	
	Axz_prev = Axz;
	Ayz_prev = Ayz;*/
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

float IMU::square(float num) {
	return num*num;
}
	
