
#include "receiver.h"

Receiver rece;

void setup () {
	Serial.begin(9600);
	rece.initialize();
}

void loop () {
	rece.read();
}