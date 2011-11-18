#include "magnetometer.h"

//Honeywell HMC5843 - Magnetometer
//I2C - Communication
Magnetometer::Magnetometer() {
	
	ADDR = 0x1E; //Device address 
	//Register addresses
	REG_CONFA = 0x00; //Configuration A
	REG_CONFB = 0x01; //Configuration B
	REG_MODE = 0x02; //Mode register
	
	REG_DATA_X_MSB = 0x03; //Data X MSB
	
	//Settings
	SET_CONFA = 0b00010000;
	SET_CONFB = 0b00100000;
	SET_MODE = 0b00000000;
	
	
	
}

//Initialize Magnetometer measurement
void Magnetometer::initialize() {
	hmc_write(REG_CONFA,SET_CONFA);
	delay(10);
	hmc_write(REG_CONFB,SET_CONFB);
	delay(10);
	hmc_write(REG_MODE,SET_MODE);
	delay(200);
	
}

void Magnetometer::getData(float* magnetodata) {
	hmc_read_rates();
	magnetodata[0] = parseRawData(magX);
	magnetodata[1] = parseRawData(magY);
	magnetodata[2] = parseRawData(magZ);
}

void Magnetometer::testDevice(){
	
	hmc_read(0x00);
	Serial.println(res,BIN);
	hmc_read(0x01);
	Serial.println(res,BIN);
	hmc_read(0x02);
	Serial.println(res,BIN);
	
	hmc_read(0x09);
	Serial.println(res,BIN);
	hmc_read(0x0A);
	Serial.println(res,BIN);
	hmc_read(0x0B);
	Serial.println(res,BIN);
	hmc_read(0x0C);
	Serial.println(res,BIN);
	
}

void Magnetometer::hmc_read_rates() {
	// Read order Z_MSB, Z_LSB, Y_MSB, Y_LSB, X_MSB and X_LSB
	i2c_start_wait( (ADDR << 1) | I2C_WRITE ); // Start write
	i2c_write(REG_DATA_X_MSB); // X_MSB register address
	i2c_rep_start( (ADDR << 1) | I2C_READ ); // Start reading
	
	//Read and combine X_MSB and X_LSB
	magX = (unsigned short) ((i2c_readAck() << 8) & 0xFF00) ;
	magX |= (unsigned short) (i2c_readAck() & 0x00FF);
	//Read and combine Y_MSB and Y_LSB
	magY = (unsigned short) ((i2c_readAck() << 8) & 0xFF00) ;
	magY |= (unsigned short) (i2c_readAck() & 0x00FF);
	//Read and combine Z_MSB and Z_LSB
	magZ = (unsigned short) ((i2c_readAck() << 8) & 0xFF00) ;
	magZ |= (unsigned short) (i2c_readNak() & 0x00FF);
	
	i2c_stop();

}

//Write 8bit data to register with 8 bit address
void Magnetometer::hmc_write(byte address, byte data) {
  i2c_start_wait( (ADDR << 1) | I2C_WRITE ); // Start / Device address and Write (0)
  i2c_write(address); // 8bit register address 
  i2c_write(data); // 8 bit data
  i2c_stop(); // End condition
}

void Magnetometer::hmc_read(byte address) {
  i2c_start_wait( (ADDR << 1) | I2C_WRITE ); // Start / Device address and Write (0)
  i2c_write(address); // 8bit register address 
  i2c_rep_start( (ADDR << 1) | I2C_READ ); // Repeated Start / Device address and Read (1)
  res = i2c_readNak(); // Read the result to res
  i2c_stop(); // End condition	
}

float Magnetometer::parseRawData(unsigned short rawReading) {
	short base = rawReading;
	if ( base & 0x8000) { //Read sign bit
		//Negative number
		return ( -1 * ( (~base) + 0x0001) ) ; //Convert bit (NOT) and add one
	} else {
		//Positive number
		return ( base ); //Leave as it is
	}
}
