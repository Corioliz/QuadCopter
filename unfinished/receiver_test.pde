
/** EXPERIMENTAL ARV IMPLEMENTATION */
#include <stdlib.h>
#include "WProgram.h"
#include "pins_arduino.h"

#define PULSEONHIGHLIMIT 2075
#define PULSEONLOWLIMIT 950
#define PULSEOFFLOWLIMIT 12000
#define PULSEOFFHIGHLIMIT 24000

long prevTime;
long loopTimer;
long prevLoopTime;
volatile long pulseLength = 0;
volatile long cmd = 0;

const int numChannels = 6;
byte previousState = 0x00;

// Struct for pulse data
typedef struct {
  unsigned char pulseEdge;
  unsigned long risingEdgeTime;
  unsigned long fallingEdgeTime;
  unsigned long lastWidth;
} 
pinPulseData;
volatile static pinPulseData pulseData[numChannels];

// Attach interrupt function
SIGNAL(PCINT2_vect) {
  myInterruptFcn(); 
}

void setup() {
  // PORTK A8..A15 have special functionality
  // PCINT16...23 (pin change interrupts)
  DDRK = 0; // All port K pins as input
  PORTK = 0; // All port K pins set to "0"
  
  // PCMSK2 defines which interrupts PCINT16...23 trigger Pin Change Interrupt Vector2
  for (int i = 0; i < numChannels; i++) {
    PCMSK2 |= (0x01 << i);
    pulseData[i].pulseEdge = 0;
  }

  PCICR |= 0x1 << 2; // Enable interrupt vector 2

  Serial.begin(9600);
  prevLoopTime = micros();  
}

void loop() {
  if (micros()- prevLoopTime > 50000)
  {
    readReceiverValues();
    prevLoopTime = micros();
  }
}

void readReceiverValues() {
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

/** WORKING ARDUINO IMPLEMENTATION
 * int pin = 2; // Interrupt 0
 * volatile int cmd = 0;
 * long prevTime;
 * long time;
 * 
 * void setup()
 * {
 * pinMode(pin, INPUT);
 * attachInterrupt(0,intFcn,CHANGE);
 * Serial.begin(9600);
 * prevTime = micros();
 * }
 * 
 * void loop()
 * {
 * Serial.println(cmd,DEC);
 * delay(200); 
 * }
 * 
 * void intFcn()
 * {
 * cmd = micros() - prevTime;
 * prevTime = micros();
 }*/

