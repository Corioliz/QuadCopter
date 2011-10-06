#include "gyroscope.h"

//VTI Technologies - CMR3000-DOX Gyroscope
//I2C Communication
Gyroscope::Gyroscope() {
	ADDR = 0x1E; //Device address (MISO pin state = LOW)
	ADDR_LSB_PIN = 31; //Arduino Pin for MISO state
	
	CTRL = 0x02; //Register address for Control register
	//CTRL register modes
	RESET = 0x80; //Reset device
	MODE_80 = 0x04; //Measurement, BW = 80 Hz
	MODE_20 = 0x06; //Measurement, BW = 80 Hz
	INT_DIS = 0x01; //Interrupt disabled
	
	countsToDps = 1/1.33f*0.75; //Data to degrees per second
	XOffset = 0; //Offset for X axis
	YOffset = 0; //Offset for Y axis
	ZOffset = 0; //Offset for Z axis
	countsToRps = 1/1.33f * 0.75 * 0.0174533; //Data to radians per second
}

//Initialize Gyroscope measurement
void Gyroscope::initialize() {
	digitalWrite(ADDR_LSB_PIN, LOW); //MISO state is LOW
	cmr_write(CTRL, RESET); //Control = Reset
	delay(2);
	
	cmr_write(CTRL, MODE_20 | INT_DIS); //Control = Measurement 20Hz and Interrupt disabled
	delay(2000);
	calibrate(); //Calibrate X, Y and Z Offsets
	
}

//Get data in degrees per seconds to rate vector [x,y,z]
void Gyroscope::getData(float* rate) {
	cmr_read_rates(); //Read raw data to rawX, rawY and rawZ
	rate[0] = (parseRawData(rawX) - XOffset) * countsToDps;
	rate[1] = (parseRawData(rawY) - YOffset) * countsToDps;
	rate[2] = (parseRawData(rawZ) - ZOffset) * countsToDps;
	
}

//Get data in radians per seconds to rate vector [x,y,z]
void Gyroscope::getSIData(float* rate) {
	cmr_read_rates(); //Read raw data to rawX, rawY and rawZ
	rate[0] = (parseRawData(rawX) - XOffset) * countsToRps;
	rate[1] = (parseRawData(rawY) - YOffset) * countsToRps;
	rate[2] = (parseRawData(rawZ) - ZOffset) * countsToRps;
}

//Calibrate X, Y and Z offsets
void Gyroscope::calibrate() {
	//Calculate mean of 100 measurements in stable position
	for (int i = 0; i < 100; i++) {
		cmr_read_rates(); //Read raw data
		XOffset += parseRawData(rawX) * 0.01f;
		YOffset += parseRawData(rawY) * 0.01f;
		ZOffset += parseRawData(rawZ) * 0.01f;
	}
}

//Write 8bit data to register with 8bit address
void Gyroscope::cmr_write(byte address, byte data) {
  i2c_start_wait( (ADDR << 1) | I2C_WRITE ); // Start / Device address and Write (0)
  i2c_write(address); // 8bit register address 
  i2c_write(data); // 8 bit data
  i2c_stop(); // End condition
}

//Read 8bit data from register with 8bit address
void Gyroscope::cmr_read(byte address) {
  i2c_start_wait( (ADDR << 1) | I2C_WRITE ); // Start / Device address and Write (0)
  i2c_write(address); // 8bit register address 
  i2c_rep_start( (ADDR << 1) | I2C_READ ); // Repeated Start / Device address and Read (1)
  res = i2c_readNak(); // Read the result to res
  i2c_stop(); // End condition
}

//Read raw data to rawX, rawY and rawZ
void Gyroscope::cmr_read_rates() {
  // Read order Z_MSB, Z_LSB, Y_MSB, Y_LSB, X_MSB and X_LSB
  i2c_start_wait( (ADDR << 1) | I2C_WRITE ); // Start write
  i2c_write(0x11); // Z_MSB register address
  i2c_rep_start( (ADDR << 1) | I2C_READ ); // Start reading

  //Read and combine Z_MSB and Z_LSB
  rawZ = (unsigned short) ((i2c_readAck() << 8) & 0xFF00);
  rawZ |= (unsigned short)(i2c_readAck() & 0x00FF);
  //Read and combine Y_MSB and Y_LSB
  rawY = (unsigned short) ((i2c_readAck() << 8) & 0xFF00);
  rawY |= (unsigned short)(i2c_readAck() & 0x00FF);
  //Read and combine X_MSB and X_LSB
  rawX = (unsigned short) ((i2c_readAck() << 8) & 0xFF00);
  rawX |= (unsigned short)(i2c_readNak() & 0x00FF);

  i2c_stop(); //End condition
}

//Convert raw data binary value to decimal
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
