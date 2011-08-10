#ifndef Gyroscope_h
#define Gyroscope_h

#include "Wprogram.h"
#include "i2cmaster.h"

class Gyroscope
{
	private:
		byte ADDR;
		byte ADDR_LSB_PIN;
		byte CTRL;
		byte RESET;
		byte MODE_80;
		byte MODE_20;
		byte INT_DIS;
		unsigned short rawX, rawY, rawZ;
		float XOffset, YOffset, ZOffset;
		char res;
		float countsToDps;
		
		void cmr_write(byte address, byte data);
		void cmr_read(byte address);
		void cmr_read_rates();
		short parseRawData(unsigned short rawReading);
		
	public:
		Gyroscope();
		void initialize();		
		void getData(float* rate);
		void calibrate();
};

#endif
