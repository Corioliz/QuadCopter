#include "accelerometer.h"

Accelerometer::Accelerometer() {
	ADDR_1 = 0x79;
	ADDR_2 = 0xF1;
	I2C_RD_SEL = 0x17;
	SCAMODE = 0x14;
	countsToG = 0.001f;
}

void Accelerometer::initialize() {
	sca_write(SCAMODE, 0x80);
	sca_read(SCAMODE);
}

void Accelerometer::getData(float* acc) {
	sca_readData();
	acc[0] = parseRawData(rawData[0], rawData[1]) * countsToG;
	acc[1] = parseRawData(rawData[2], rawData[3]) * countsToG;
	acc[2] = parseRawData(rawData[4], rawData[5]) * countsToG;
}

void Accelerometer::sca_write(byte address, byte data) {
	i2c_start_wait( (ADDR_1 << 1) | I2C_WRITE );
	i2c_write(ADDR_2);
	i2c_write(address);
	i2c_write(data);
	i2c_stop();
}

void Accelerometer::sca_read(byte address) {
	// Write the to-be-read register address to I2C_RD_SEL
	sca_write(I2C_RD_SEL, address);
	i2c_start_wait( (ADDR_1 << 1) | I2C_WRITE );
	i2c_write(ADDR_2);
	i2c_rep_start( (ADDR_1 << 1) | I2C_READ );
	res = i2c_readNak();
	i2c_stop();
}

void Accelerometer::sca_readData() {
	// Read order Y X Z
	// Cycle: Y_MSB -> Y_LSB -> X_MSB -> X_LSB -> Z_MSB -> Z_LSB 
	sca_write(I2C_RD_SEL, 0x07); // Y_LSB address

	i2c_start_wait( (ADDR_1 << 1) | I2C_WRITE );
	i2c_write(ADDR_2);
	i2c_rep_start( (ADDR_1 << 1) | I2C_READ );

	for (int k = 0; k < 5; ++k) { 
	rawData[k] = i2c_readAck();
	}
	rawData[5] = i2c_readNak();
	i2c_stop();
}

short Accelerometer::parseRawData(byte MSB, byte LSB) {
	short base = ((MSB & 0x7f) << 5) + (LSB >> 3);
	if (MSB & 0x80) {
	// Negative number, take XOR with 0xff and add one
	return -1 * ( (base ^ 0x0fff) + 0x01);
	} 
	else {
	return base;
	}
}
	 
