#include "accelerometer.h"

//VTI Technologies - SCA3000-EO2 Accelerometer
//I2C Communication
Accelerometer::Accelerometer() {
	ADDR_1 = 0x79; // 011110 and 2 first bits of device address
	ADDR_2 = 0xF1; // 8 last bits of device address
	I2C_RD_SEL = 0x17; //Register address for I2C read operation
	SCAMODE = 0x14; //Register address for Mode
	countsToG = 0.001f;
}

//Initialize Acceleromter
void Accelerometer::initialize() {
	sca_write(SCAMODE, 0x80); // Select operation mode / Output ring buffer enabled , others are initial values
	//sca_read(SCAMODE); //Read operation mode ?
}

//Get data as ? to acc vector
void Accelerometer::getData(float* acc) {
	sca_readData(); //Read raw data to rawData vector
	acc[0] = parseRawData(rawData[0], rawData[1]) * countsToG;
	acc[1] = parseRawData(rawData[2], rawData[3]) * countsToG;
	acc[2] = parseRawData(rawData[4], rawData[5]) * countsToG;
}
//Write 8bit data to 8bit address 
void Accelerometer::sca_write(byte address, byte data) {
	i2c_start_wait( (ADDR_1 << 1) | I2C_WRITE ); //Start / First part of device address and Write (0)
	i2c_write(ADDR_2); //Second part of device address
	i2c_write(address); //Register address
	i2c_write(data); //Write data
	i2c_stop(); //End condition
}
//Read data from 8bit address
void Accelerometer::sca_read(byte address) {
	// Write the to-be-read register address to I2C_RD_SEL
	sca_write(I2C_RD_SEL, address);
	i2c_start_wait( (ADDR_1 << 1) | I2C_WRITE ); //Start / First part of device address and Write (0)
	i2c_write(ADDR_2); //Second part of device address
	i2c_rep_start( (ADDR_1 << 1) | I2C_READ ); //Repeated Start / First part of device address and Read (1)
	res = i2c_readNak(); //Read result to res
	i2c_stop(); //End condition
}
//Read raw data to rawData vector
void Accelerometer::sca_readData() {
	// Read order Y X Z
	// Cycle: Y_MSB -> Y_LSB -> X_MSB -> X_LSB -> Z_MSB -> Z_LSB 
	sca_write(I2C_RD_SEL, 0x07); // Y_LSB address

	i2c_start_wait( (ADDR_1 << 1) | I2C_WRITE ); //Start / First part of device address and Write (0)
	i2c_write(ADDR_2); //Second part of device address
	i2c_rep_start( (ADDR_1 << 1) | I2C_READ ); //Repeated Start / First part of device address and Read (1)

	for (int k = 0; k < 5; ++k) { 
	rawData[k] = i2c_readAck(); //Read 
	}
	rawData[5] = i2c_readNak(); //Read last (Z_LSB)
	i2c_stop(); //End condition
}
//Convert raw data binary value to decimal
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
	 
