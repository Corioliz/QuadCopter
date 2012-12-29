#ifndef Gyroscope_h
#define Gyroscope_h

#if defined(ARDUINO) && ARDUINO >= 100
  #include "Arduino.h"
  #else
  #include "WProgram.h"
#endif
#include "i2cmaster.h"

//VTI Technologies - CMR3000-DOX Gyroscope
//I2C - Communication
class Gyroscope
{
	private:
		byte ADDR; //Device address
		byte ADDR_LSB_PIN; //Arduino pin for MISO state
		byte CTRL; //Control register address
		byte RESET; //Control / Reset
		byte MODE_80; //Control / Measurement 80 Hz
		byte MODE_20; //Control / Measurement 20 Hz
		byte INT_DIS; //Control / Interrupt disabled
		unsigned short rawX, rawY, rawZ; //Raw data of X, Y and Z axis
		float XOffset, YOffset, ZOffset; //Offset for X, Y and Z axis
		char res; //Result from read operation
		float countsToDps; //Raw data to degrees per second
		float countsToRps; //Raw data to radians per second
		
		//Functions
		void cmr_write(byte address, byte data); //Write data to address
		void cmr_read(byte address); //Read address to res
		void cmr_read_rates(); //Read raw data (rawX, rawY and rawZ)
		float parseRawData(unsigned short rawReading); //Convert raw data binary value to decimal
		
	public:
		Gyroscope(); //Constructor
		//Functions
		void initialize(); //Initialize gyroscope
		void getData(float* rate); //Get data as degrees per second to rate vector
		void getSIData(float* rate); //Get data as radians per second to rate vector
		void calibrate(); //Calibrate offsets
};

#endif
