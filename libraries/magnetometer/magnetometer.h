#ifndef Magnetometer_h
#define Magnetometer_h

#include "Wprogram.h"
#include "i2cmaster.h"

//Honeywell HMC5843 - Magnetometer
//I2C - Communication
class Magnetometer
{
	private:
		byte ADDR; //Device address
		//Register addresses
		byte REG_CONFA; //Configuration A
		byte REG_CONFB; //Configuration B
		byte REG_MODE; //Mode register
		
		byte REG_DATA_X_MSB; //Data X MSB
		
		//Settings
		byte SET_CONFA;
		byte SET_CONFB;
		byte SET_MODE;
		
		unsigned short magX, magY, magZ;
		
		char res; //Result from Read
		
		//Functions
		void hmc_write(byte address, byte data);
		void hmc_read(byte address);
		void hmc_read_rates();
		float parseRawData(unsigned short rawReading);
		
	public:
		Magnetometer(); //Constructor
		//Functions
		void initialize(); //Initialize magnetometer
		void getData(float* magnetodata); //Get data
		void testDevice();
};

#endif