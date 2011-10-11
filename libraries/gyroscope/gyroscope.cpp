#include "gyroscope.h"

Gyroscope::Gyroscope() {
	ADDR = 0x1E;
	ADDR_LSB_PIN = 31;
	RESET = 0x80;
	CTRL = 0x02;
	MODE_80 = 0x04;
	MODE_20 = 0x06;
	INT_DIS = 0x01;
	countsToDps = 0.75;
	XOffset = 0;
	YOffset = 0;
	ZOffset = 0;
	countsToRps = 0.75 * 0.0174533;
}

void Gyroscope::initialize() {
	digitalWrite(ADDR_LSB_PIN, LOW);
	cmr_write(CTRL, RESET);
	delay(2);
	
	cmr_write(CTRL, MODE_20 | INT_DIS);
	delay(2000);
	calibrate();
	
}
	
void Gyroscope::getData(float* rate) {
	cmr_read_rates();
	rate[0] = (parseRawData(rawX) - XOffset) * countsToDps;
	rate[1] = (parseRawData(rawY) - YOffset) * countsToDps;
	rate[2] = (parseRawData(rawZ) - ZOffset) * countsToDps;
	
}

void Gyroscope::getSIData(float* rate) {
	cmr_read_rates();
	rate[0] = (parseRawData(rawX) - XOffset) * countsToRps;
	rate[1] = (parseRawData(rawY) - YOffset) * countsToRps;
	rate[2] = (parseRawData(rawZ) - ZOffset) * countsToRps;
}

void Gyroscope::calibrate() {
	for (int i = 0; i < 100; i++) {
		cmr_read_rates();
		XOffset += parseRawData(rawX) * 0.01f;
		YOffset += parseRawData(rawY) * 0.01f;
		ZOffset += parseRawData(rawZ) * 0.01f;
	}
}
void Gyroscope::cmr_write(byte address, byte data) {
  i2c_start_wait( (ADDR << 1) | I2C_WRITE );
  i2c_write(address);
  i2c_write(data);
  i2c_stop();
}

void Gyroscope::cmr_read(byte address) {
  i2c_start_wait( (ADDR << 1) | I2C_WRITE );
  i2c_write(address);
  i2c_rep_start( (ADDR << 1) | I2C_READ );
  res = i2c_readNak();
  i2c_stop();
}

void Gyroscope::cmr_read_rates() {
	// Read order Z Y X
  i2c_start_wait( (ADDR << 1) | I2C_WRITE );
  i2c_write(0x11); // Z_MSB
  i2c_rep_start( (ADDR << 1) | I2C_READ );

  rawZ = (unsigned short) ((i2c_readAck() << 8) & 0xFF00);
  rawZ |= (unsigned short)(i2c_readAck() & 0x00FF);

  rawY = (unsigned short) ((i2c_readAck() << 8) & 0xFF00);
  rawY |= (unsigned short)(i2c_readAck() & 0x00FF);

  rawX = (unsigned short) ((i2c_readAck() << 8) & 0xFF00);
  rawX |= (unsigned short)(i2c_readNak() & 0x00FF);

  i2c_stop();
}

short Gyroscope::parseRawData(unsigned short rawReading) {
	short base = (rawReading >> 1) & 0x1FFF; // 12 bit data + sign bit
	if (base & 0x1000) { // Read sign bit and act accordingly
	// Negative number, take XOR for 12 least significant bits and add one
		return ( -1 * ( ((base & 0x0fff) ^ 0x0fff) + 0x0001));
	} 
	else {
		return (base & 0x0fff);
	}
}
