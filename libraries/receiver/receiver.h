#ifndef Receiver_h
#define Receiver_h

#include <stdlib.h>
#include "WProgram.h"
#include "pins_arduino.h"

#define PULSEONHIGHLIMIT 2075
#define PULSEONLOWLIMIT 950
#define PULSEOFFLOWLIMIT 12000
#define PULSEOFFHIGHLIMIT 24000

class Receiver {

  public:
  void initialize() {
	DDRK = 0; // All port K pins as input
    PORTK = 0; // All port K pins set to "0"
  
    // PCMSK2 defines which interrupts PCINT16...23 trigger Pin Change Interrupt Vector2
    for (int i = 0; i < numChannels; i++) {
      PCMSK2 |= (0x01 << i);
      pulseData[i].pulseEdge = 0;
    }
    PCICR |= 0x1 << 2; // Enable interrupt vector 2
  }
  void read() {
  // Critical region (interrupts disabled because of read operation)
  // SREG state saved (has to be done this way because the
  // state is not known)
  byte prevSREG = SREG;
  cli(); // Disable interrupts while reading values
  for (int i = 0; i < numChannels; i++) {
    Serial.print(pulseData[i].lastWidth); Serial.print(",");
  }
  Serial.print(0); Serial.println("");  
  SREG = prevSREG; // Restore SREG state
  }
	
};

volatile static byte previousState = 0x00;
// Struct for pulse data
typedef struct {
  unsigned char pulseEdge;
  unsigned long risingEdgeTime;
  unsigned long fallingEdgeTime;
  unsigned long lastWidth;
} 
pinPulseData;
volatile static pinPulseData pulseData[numChannels];

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
