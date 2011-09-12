#ifndef PID_h
#define PID_h

#include <stdlib.h>
#include "WProgram.h"
#include "pins_arduino.h"

class PID {
	public:
	
	PID(float p, float i, float d, float ref, float in, float maxCtrl, float minCtrl) {
		Kp = p;
		Ki = i;
		Kd = d;
		prevTime = millis();
		prevRef = ref;
		prevInput = in;
		
		minLimit = minCtrl;
		maxLimit = maxCtrl;
	}

	float calculate(float ref, float input) {
		// Calculate sampling time
		long dt = 0.001 * (millis() - prevTime); // Convert to seconds

		
		float error = ref - input;
		pTerm = Kp * (ref - input);
		dTerm = -Kd * (input - prevInput)/dt; // dError/dt = - dInput/dt
		iTerm += Ki * error * dt;
		
		// Calculate control
		float output = pTerm + iTerm + dTerm;
		
		// Anti-windup
		if (output > maxLimit) {
			iTerm -= output - maxLimit;
			output = maxLimit;
		} else {
			iTerm += minLimit - output;
			output = minLimit;
		}
		
		// Update state
		prevTime = millis();
		prevRef = ref;
		prevInput = input;
		
		return output;
	}
	
	private:
	long prevTime;
	float prevRef;
	float prevInput;
	float pTerm;
	float iTerm;
	float dTerm;
	float minLimit;
	float maxLimit;
	float Kp, Ki, Kd;
};
