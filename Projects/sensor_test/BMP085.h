/* Nikolas Simon
 * University of Freiburg 
 * December 2014
 * 
 * Functions for initializing and reading BMP085 barometer
 *
 */


#ifndef __BAROMETER_H__
#define __BAROMETER_H__

#include <stdbool.h>
#include <stdint.h>

#define BMP085_ADDRESS_WRITE    0xEE        
#define BMP085_ADDRESS_READ     0xEF        

void bmp085_init(void);
//void bmp085_test(void);
int16_t bmp085_read_pressure(void);
double bmp085_read_temperature(void);
void bmp085_read_ac1(void);
void bmp085_read_ac2(void);
void bmp085_read_md(void);

// Definition of different calibration register addresses
#define BMP085_RA_AC1_H 0xAA /* AC1_H */
#define BMP085_RA_AC1_L 0xAB /* AC1_L */
#define BMP085_RA_AC2_H 0xAC /* AC2_H */
#define BMP085_RA_AC2_L 0xAD /* AC2_L */
#define BMP085_RA_AC3_H 0xAE /* AC3_H */
#define BMP085_RA_AC3_L 0xAF /* AC3_L */
#define BMP085_RA_AC4_H 0xB0 /* AC4_H */
#define BMP085_RA_AC4_L 0xB1 /* AC4_L */
#define BMP085_RA_AC5_H 0xB2 /* AC5_H */
#define BMP085_RA_AC5_L 0xB3 /* AC5_L */
#define BMP085_RA_AC6_H 0xB4 /* AC6_H */
#define BMP085_RA_AC6_L 0xB5 /* AC6_L */
#define BMP085_RA_B1_H 0xB6 /* B1_H */
#define BMP085_RA_B1_L 0xB7 /* B1_L */
#define BMP085_RA_B2_H 0xB8 /* B2_H */
#define BMP085_RA_B2_L 0xB9 /* B2_L */
#define BMP085_RA_MB_H 0xBA /* MB_H */
#define BMP085_RA_MB_L 0xBB /* MB_L */
#define BMP085_RA_MC_H 0xBC /* MC_H */
#define BMP085_RA_MC_L 0xBD /* MC_L */
#define BMP085_RA_MD_H 0xBE /* MD_H */
#define BMP085_RA_MD_L 0xBF /* MD_L */

#define BMP085_RA_CONTROL 0xF4 /* CONTROL */

#define BMP085_RA_MSB 0xF6 /* MSB */
#define BMP085_RA_LSB 0xF7 /* LSB */
#define BMP085_RA_XLSB 0xF8 /* XLSB */

// Definition of different operating mode register addresses
#define BMP085_MODE_TEMPERATURE 0x2E
#define BMP085_MODE_PRESSURE_0 0x34
#define BMP085_MODE_PRESSURE_1 0x74
#define BMP085_MODE_PRESSURE_2 0xB4
#define BMP085_MODE_PRESSURE_3 0xF4

/*



uint16_t ltc2942_read_voltage(void);
const unsigned char OSS = 0;
long PressureCompensate;
float bmp085GetTemperature(unsigned int ut);
long bmp085GetPressure(unsigned long up);
float calcAltitude(float pressure);
unsigned int bmp085ReadUT(void);
unsigned long bmp085ReadUP(void);

int ac1;
int ac2;
int ac3;
unsigned int ac4;
unsigned int ac5;
unsigned int ac6;
int b1;
int b2;
int mb;
int mc;
int md;
char bmp085Read(unsigned char address);
int bmp085ReadInt(unsigned char address);
void writeRegister(int deviceAddress, byte address, byte val);
int readRegister(int deviceAddress, byte address);

*/

#endif
