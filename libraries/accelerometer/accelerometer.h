#ifndef Accelerometer_h
#define Accelerometer_h

#include "Wprogram.h"
#include "i2cmaster.h"

class Accelerometer
{
	private:
		byte ADDR_1;
		byte ADDR_2;
		byte I2C_RD_SEL;
		byte SCAMODE;
		byte rawData[6];
		char res;
		float countsToG;
		
		void sca_write(byte address, byte data);	
		void sca_read(byte address);		
		void sca_readData();
		short parseRawData(byte MSB, byte LSB);
		
	public:
		Accelerometer();
		void initialize();		
		void getData(float* acc);
};

#endif
