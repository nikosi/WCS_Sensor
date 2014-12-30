/* 
 * Nikolas Simon
 * University of Freiburg 
 * December 2014
 * 
 * Functions for initializing and reading 
 * BMP085 Barometer via I2C
 * (Based on the Arduino-Code)
 *
 */
 
 
 #include "BMP085.h"
 
 #include "nrf_gpio.h"
 #include "powermeter_board.h"
 #include "twi_master.h"
 #include "nrf_delay.h"
 
const unsigned char OSShift = 0;

//Calibration values
int16_t ac1, ac2, ac3, b1, b2, mb, mc, md, ac1_test, ac2_test,md_test;
uint16_t ac4, ac5, ac6;

void bmp085_read_ac1(void)
{
	 // Set control register
     uint8_t i2c_data[22] = {0};

		// First calibration register address (0xAA)
		i2c_data[0] = BMP085_RA_AC1_H;				
		
		// Write first calibration register address to slave device
    twi_master_transfer(BMP085_ADDRESS_WRITE, i2c_data, 1, TWI_DONT_ISSUE_STOP); 
		 
    // Read calibration registers 0xAA = 0xBF
    twi_master_transfer(BMP085_ADDRESS_READ, i2c_data, 2, TWI_ISSUE_STOP);
		 
    ac1_test =  (((int16_t)i2c_data[0]<<8) | i2c_data[1]);
}

void bmp085_read_md(void)
{
	 // Set control register
     uint8_t i2c_data[22] = {0};

		// First calibration register address (0xAA)
		i2c_data[0] = BMP085_RA_MD_H;				
		
		// Write first calibration register address to slave device
    twi_master_transfer(BMP085_ADDRESS_WRITE, i2c_data, 1, TWI_DONT_ISSUE_STOP); 
		 
    // Read calibration registers 0xAA = 0xBF
    twi_master_transfer(BMP085_ADDRESS_READ, i2c_data, 2, TWI_ISSUE_STOP);
		 
    md_test =  (((int16_t)i2c_data[0]<<8) | i2c_data[1]);
}

void bmp085_read_ac2(void)
{
	 // Set control register
     uint8_t i2c_data[22] = {0};

		// First calibration register address (0xAA)
		i2c_data[0] = BMP085_RA_AC2_H;				
		
		// Write first calibration register address to slave device
    twi_master_transfer(BMP085_ADDRESS_WRITE, i2c_data, 1, TWI_DONT_ISSUE_STOP); 
		 
    // Read calibration registers 0xAA = 0xBF
    twi_master_transfer(BMP085_ADDRESS_READ, i2c_data, 2, TWI_ISSUE_STOP);
		 
    ac2_test =  (((int16_t)i2c_data[0]<<8) | i2c_data[1]);
}

// Initializing the BMP 085 by reading 
void bmp085_init(void)
{
    // Set control register
     uint8_t i2c_data[22] = {0};

		// First calibration register address (0xAA)
		i2c_data[0] = BMP085_RA_AC1_H;				
		
		// Write first calibration register address to slave device
    twi_master_transfer(BMP085_ADDRESS_WRITE, i2c_data, 1, TWI_DONT_ISSUE_STOP); 
		 
    // Read calibration registers 0xAA = 0xBF
    twi_master_transfer(BMP085_ADDRESS_READ, i2c_data, 22, TWI_ISSUE_STOP);
		 
    ac1 =  (((int16_t)i2c_data[0]<<8) | i2c_data[1]);
    ac2 =  (((int16_t)i2c_data[2]<<8) | i2c_data[3]);
    ac3 =  (((int16_t)i2c_data[4]<<8) | i2c_data[5]);
    ac4 =  (((uint16_t)i2c_data[6]<<8) | i2c_data[7]);
    ac5 =  (((uint16_t)i2c_data[8]<<8) | i2c_data[9]);
    ac6 =  (((uint16_t)i2c_data[10]<<8) | i2c_data[11]);
		 
    b1 =  (((int16_t)i2c_data[12]<<8) | i2c_data[13]);
    b2 =  (((int16_t)i2c_data[14]<<8) | i2c_data[15]);
		 
    mb =  (((int16_t)i2c_data[16]<<8) | i2c_data[17]);
    mc =  (((int16_t)i2c_data[18]<<8) | i2c_data[19]);
    md =  (((int16_t)i2c_data[20]<<8) | i2c_data[21]);
}


// Read raw data of temperature
double bmp085_read_temperature(void)
{
	uint8_t i2c_data[2] = {0};
	int16_t raw_temperature = 0;
	
	// Set control register to temp measurement mode
	i2c_data[0] = BMP085_RA_CONTROL;				
	i2c_data[1] = 0x35; //BMP085_MODE_TEMPERATURE;
	twi_master_transfer(BMP085_ADDRESS_WRITE, i2c_data, 2, TWI_ISSUE_STOP); 
	
	// Write temperature register address
	i2c_data[0] = BMP085_RA_CONTROL;	
  twi_master_transfer(BMP085_ADDRESS_WRITE, i2c_data, 1, TWI_ISSUE_STOP); 

	twi_master_transfer(BMP085_ADDRESS_READ, i2c_data, 1, TWI_ISSUE_STOP);
	
	// Wait maximum conversion time
	nrf_delay_ms(5);
	
	// Write temperature register address
	i2c_data[0] = BMP085_RA_MSB;	
  twi_master_transfer(BMP085_ADDRESS_WRITE, i2c_data, 1, TWI_DONT_ISSUE_STOP); 

	// Read raw temperature data
	twi_master_transfer(BMP085_ADDRESS_READ, i2c_data, 2, TWI_ISSUE_STOP);
	
	raw_temperature = (int16_t) ((i2c_data[0]<<8) | i2c_data[1]);
	
	/*raw_temperature = 27898;
	ac1 =408;
	ac2 = -72;
	ac3 =-14383;
	ac4 =32741;
	ac5 =32757;
	ac6 =23153;
	b1 =6190;
	b2 =4;
	mb = -32768;
	mc =-8711;
	md =2868;*/
	
	 /*	Datasheet formula:
			UT = raw temperature
			X1 = (UT - AC6) * AC5 / 2^15
			X2 = MC * 2^11 / (X1 + MD)
			B5 = X1 + X2
			T = (B5 + 8) / 2^4	*/
			
	//int32_t ut = raw_temperature;
	
	//int32_t x1 = ((raw_temperature - ac6) * ac5) >> 15;
	double x1 = ((double)((raw_temperature - ac6) * ac5)) / 32768.0 ;
	double x2 = (((double)mc) * 2048.0) / (x1 + ((double)md));
	double b5 = x1 + x2;
	
	return ((b5 + 8.0) / 16.0);
	
	//return raw_temperature;
}


// Read raw data of pressure
int16_t bmp085_read_pressure(void)
{
	uint8_t i2c_data[3] = {0};
	int16_t raw_pressure = 0;
	
	// Set control register to temp measurement mode
	i2c_data[0] = BMP085_RA_CONTROL;				
	i2c_data[1] = BMP085_MODE_PRESSURE_0;
	twi_master_transfer(BMP085_ADDRESS_WRITE, i2c_data, 2, TWI_DONT_ISSUE_STOP); 
	
	// Wait maximum conversion time
	nrf_delay_ms(7);
	
	// Write temperature register address
	i2c_data[0] = BMP085_RA_MSB;				

	// Read raw temperature data
	twi_master_transfer(BMP085_ADDRESS_READ, i2c_data, 3, TWI_ISSUE_STOP);
	
	raw_pressure = (int16_t) ((i2c_data[0]<<8) | i2c_data[1]);
	
	return raw_pressure;
}

/*
{
	  long up = 0;
	
		i2c_data[0] = 0xF4;          												// Control register address
    i2c_data[1] = 0x34 + (OSShift<<6);		              // 1111 1000
    
    // Write voltage register address
    twi_master_transfer(BMP085_ADDRESS_WRITE, i2c_data, 2, TWI_DONT_ISSUE_STOP); 
    
		i2c_data[0] = 0xF6;          												// Control register address
		
		// Write voltage register address
    twi_master_transfer(BMP085_ADDRESS_WRITE, i2c_data, 1, TWI_DONT_ISSUE_STOP); 
		
    // Read voltage register
    twi_master_transfer(BMP085_ADDRESS_READ, i2c_data, 3, TWI_DONT_ISSUE_STOP); 
		
		up = (i2c_data[0] <<11) | (i2c_data[1]<<3) | (i2c_data[2]&0x07);
    //up = (((unsigned long) i2c_data[0] << 16) | ((unsigned long) i2c_data[1] << 8) | (unsigned long) i2c_data[2]) >> (8-OSShift);
    return up;
}*/


/*
long GetPressure(unsigned long up)
{
    long x1, x2, x3, b3, b6, p;
    unsigned long b4, b7;
    b6 = PressureCompensate - 4000;
    x1 = (b2 * (b6 * b6)>>12)>>11;
    x2 = (ac2 * b6)>>11;
    x3 = x1 + x2;
    b3 = (((((long)ac1)*4 + x3)<<OSS) + 2)>>2;

    // Calculate B4
    x1 = (ac3 * b6)>>13;
    x2 = (b1 * ((b6 * b6)>>12))>>16;
    x3 = ((x1 + x2) + 2)>>2;
    b4 = (ac4 * (unsigned long)(x3 + 32768))>>15;

    b7 = ((unsigned long)(up - b3) * (50000>>OSS));
    if (b7 < 0x80000000)
    p = (b7<<1)/b4;
    else
    p = (b7/b4)<<1;

    x1 = (p>>8) * (p>>8);
    x1 = (x1 * 3038)>>16;
    x2 = (-7357 * p)>>16;
    p += (x1 + x2 + 3791)>>4;

    long temp = p;
    return temp;
}


float GetAltitude(float pressure)
{
    float A = pressure/101325;
    float B = 1/5.25588;
    float C = pow(A,B);
    C = 1 - C;
    C = C /0.0000225577;
    return C;
}
*/
