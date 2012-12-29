#ifndef Accelerometer_h
#define Accelerometer_h

#include "Wprogram.h"
#include "i2cmaster.h"

//VTI Technologies - SCA3000-EO2 Accelerometer
//I2C Communication
class Accelerometer
{
	private:
		byte ADDR_1; // 011110 and 2 first bits of device address
		byte ADDR_2; // 8 last bits of device address
		byte I2C_RD_SEL; //Register address for I2C read operation
		byte SCAMODE; //Register address for Mode
		byte rawData[6]; //Raw data
		char res; //Result of read operation
		float countsToG;
		
		//Functions
		void sca_write(byte address, byte data); //Write data to register with address
		void sca_read(byte address); //Read register with address
		void sca_readData(); //Read raw data to rawData
		short parseRawData(byte MSB, byte LSB); //Convert raw data binary value to decimal
		
	public:
		Accelerometer(); //Constructor
		void initialize(); //Initialize accelerometer
		void getData(float* acc); //Get data as ? to acc vector
};

#endif
