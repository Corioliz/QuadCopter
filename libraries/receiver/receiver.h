#ifndef Receiver_h
#define Receiver_h

#include <stdlib.h>
#include "WProgram.h"
#include "pins_arduino.h"

#define PULSEONHIGHLIMIT 2075
#define PULSEONLOWLIMIT 950
#define PULSEOFFLOWLIMIT 12000
#define PULSEOFFHIGHLIMIT 24000
#define NUMCHANNELS 6

#define RX_YAW_MIN 1128
#define RX_YAW_MAX 1948
#define RX_YAW_AVG 1544

#define RX_PITCH_MIN 1184
#define RX_PITCH_MAX 1816
#define RX_PITCH_AVG 1504

#define RX_ROLL_MIN 1160
#define RX_ROLL_MAX 1944
#define RX_ROLL_AVG 1544

#define RX_THROTTLE_MIN 1140
#define RX_THROTTLE_MAX 1808

#define RX_CYCLE1_MIN 1016
#define RX_CYCLE1_MAX 2044

#define RX_CYCLE2_MIN 1016
#define RX_CYCLE2_MAX 2044

#define ROLL 3
#define PITCH 2
#define YAW 0
#define THROTTLE 1
#define CYCLE1 4
#define CYCLE2 5

volatile static byte previousState = 0x00;
// Struct for pulse data
typedef struct {
  unsigned char pulseEdge;
  unsigned long risingEdgeTime;
  unsigned long fallingEdgeTime;
  unsigned long lastWidth;
} 
pinPulseData;
volatile static pinPulseData pulseData[NUMCHANNELS];

class Receiver {

  public:
  
  // Conversion factors for rx values to angles
  

  
  void initialize() {
	
	DDRK = 0; // All port K pins as input
    PORTK = 0; // All port K pins set to "0"
  
    // PCMSK2 defines which interrupts PCINT16...23 trigger Pin Change Interrupt Vector2
    for (int i = 0; i < NUMCHANNELS; i++) {
      PCMSK2 |= (0x01 << i);
      pulseData[i].pulseEdge = 0;
    }
    PCICR |= 0x1 << 2; // Enable interrupt vector 2
	
	// Precalculation for calibration values
	yawFactor = 2.0 / (RX_YAW_MAX - RX_YAW_MIN);
	yawOffset = RX_YAW_AVG; // (RX_YAW_MAX + RX_YAW_MIN) / 2;
	
	pitchFactor = 2.0 / (RX_PITCH_MAX - RX_PITCH_MIN);
	pitchOffset = RX_PITCH_AVG; // (RX_PITCH_MAX + RX_PITCH_MIN) / 2;
	
	rollFactor = 2.0 / (RX_ROLL_MAX - RX_ROLL_MIN);
	rollOffset = RX_ROLL_AVG; // (RX_ROLL_MAX + RX_ROLL_MIN) / 2;
	
	throttleFactor = 1.0 / (RX_THROTTLE_MAX - RX_THROTTLE_MIN);
	throttleOffset = RX_THROTTLE_MIN; // (RX_THROTTLE_MAX + RX_THROTTLE_MIN) / 2;
  }
  
  void read(float* rx_values) {
	// Critical region (interrupts disabled because of read operation)
	// SREG state saved (has to be done this way because the
	// state is not known)
	byte prevSREG = SREG;
	cli(); // Disable interrupts while reading values
	
	//
	rx_values[YAW] = yawFactor * (pulseData[YAW].lastWidth - yawOffset);
	rx_values[PITCH] = pitchFactor * (pulseData[PITCH].lastWidth - pitchOffset);
	rx_values[ROLL] = rollFactor * (pulseData[ROLL].lastWidth - rollOffset);
	rx_values[THROTTLE] = throttleFactor * ( pulseData[THROTTLE].lastWidth - throttleOffset );
	rx_values[CYCLE1] = pulseData[CYCLE1].lastWidth;
	rx_values[CYCLE2] = pulseData[CYCLE2].lastWidth;
	//
	
	/*
	rx_values[YAW] = pulseData[YAW].lastWidth ;
	rx_values[PITCH] = pulseData[PITCH].lastWidth ;
	rx_values[ROLL] = pulseData[ROLL].lastWidth ;
	rx_values[THROTTLE] = pulseData[THROTTLE].lastWidth ;
	rx_values[CYCLE1] = pulseData[CYCLE1].lastWidth ;
	rx_values[CYCLE2] = pulseData[CYCLE2].lastWidth ;
	*/
	
	SREG = prevSREG; // Restore SREG state
  }
  void print() {
  // Critical region (interrupts disabled because of read operation)
  // SREG state saved (has to be done this way because the
  // state is not known)
  byte prevSREG = SREG;
  cli(); // Disable interrupts while reading values
  for (int i = 0; i < NUMCHANNELS; i++) {
    Serial.print(pulseData[i].lastWidth); Serial.print(",");
  }
  Serial.print(0); Serial.println("");  
  SREG = prevSREG; // Restore SREG state
  }
  
  
  private:
	float yawFactor;
	float yawOffset;
  
    float pitchFactor;
    float pitchOffset;
  
    float rollFactor;
    float rollOffset;
	
	float throttleFactor;
	float throttleOffset;
 
};



void myInterruptFcn() {
  byte currentState;
  byte currentPin;
  byte pin;
  byte mask;
  long currentTime;
  long time;

  // Read the pin states for the port K
  currentState = *portInputRegister(11); // 11 = PORTK
  // Mask consists of pins that have changed state
  mask = currentState ^ previousState;
  
  previousState = currentState;
  // if no interrupt pins have changed, do nothing
  if ( (mask &= PCMSK2) == 0) {
    return;
  }
  currentTime = micros();
  for (byte i = 0; i < 8; i++) {
    currentPin = 0x01 << i;
    if (currentPin & mask) {
      if (currentPin & previousState) { // rising edge
        time = currentTime - pulseData[i].fallingEdgeTime;
        pulseData[i].risingEdgeTime = currentTime;
        if ((time >= PULSEOFFLOWLIMIT) && (time <= PULSEOFFHIGHLIMIT))
          pulseData[i].pulseEdge = 1; // valid rising edge
        else
          pulseData[i].pulseEdge = 0; // invalid rising edge
      }
      else { // falling edge
        time = currentTime - pulseData[i].risingEdgeTime;
        pulseData[i].fallingEdgeTime = currentTime;
        if ((time >= PULSEONLOWLIMIT) && (time <= PULSEONHIGHLIMIT) && (pulseData[i].pulseEdge == 1)) {
          pulseData[i].lastWidth = time;
          pulseData[i].pulseEdge = 0;
        }
      }
    }
  }
}

// Attach interrupt function
SIGNAL(PCINT2_vect) {
  myInterruptFcn(); 
}

#endif
