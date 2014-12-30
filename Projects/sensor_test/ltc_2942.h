/* Nikolas Simon
 * University of Freiburg 
 * Ocotber 2014
 * 
 * Functions for initializing and reading 
 * LTC 2942 Battery Gas Gauge with voltage measurement
 *
 */


#include <stdbool.h>
#include <stdint.h>


#ifndef LTC_2942_H
#define LTC_2942_H

#define LTC_ADDRESS_WRITE       0xC8        // 1100100(0)
#define LTC_ADDRESS_READ        0xC9        // 1100100(1)

#define STATUS_REG              0x00
#define CONTROL_REG             0x01
#define ACC_CHARGE_MSB_REG      0x02
#define ACC_CHARGE_LSB_REG      0x03
#define CHARGE_THRE_H_MSB       0x04
#define CHARGE_THRE_H_LSB       0x05
#define CHARGE_THRE_L_MSB       0x06
#define CHARGE_THRE_L_LSB       0x07
#define VOLTAGE_MSB             0x08
#define VOLTAGE_LSB             0x09
#define VOLTAGE_THRE_H          0x0A
#define VOLTAGE_THRE_L          0x0B
#define TEMP_MSB                0x0C
#define TEMP_LSB                0x0D
#define TEMP_THRE_H             0x0E
#define TEMP_THRE_L             0x0F

void ltc2942_init(void);
uint16_t ltc2942_read_voltage(void);
uint16_t ltc2942_read_temp(void);
void ltc2942_test(void);


#endif //LTC_2942_H
