/* Copyright (c) 2014 Nordic Semiconductor. All Rights Reserved.
 *
 * The information contained herein is property of Nordic Semiconductor ASA.
 * Terms and conditions of usage are described in detail in NORDIC
 * SEMICONDUCTOR STANDARD SOFTWARE LICENSE AGREEMENT.
 *
 * Licensees are granted free, non-transferable use of the information. NO
 * WARRANTY of ANY KIND is provided. This heading must NOT be removed from
 * the file.
 *
 */

/** @file
* @defgroup spi_master_example_main main.c
* @{
* @ingroup spi_master_example
*
* @brief SPI Master Loopback Example Application main file.
*
* This file contains the source code for a sample application using SPI.
*
*/

#include "nrf_delay.h"
#include "nrf_gpio.h"
#include "common.h"
#include "app_error.h"
#include "app_util_platform.h"
#include "spi_master.h"
#include "simple_uart.h"
#include "twi_master.h"

#include "mpu_9250.h"
#include "ADS8320.h"
#include "ltc_2942.h"
#include "BMP085.h"

#include "powermeter_board.h"
#include "example_config.h"


uint8_t imu_data[18] = 
{
  0, 0, 0, 0, 0, 0,       // Acc xh, xl, yh, yl, zh, zl
  0, 0, 0, 0, 0, 0,       // Gyro xh, xl, yh, yl, zh, zl
  0, 0, 0, 0, 0, 0        // Mag xh, xl, yh, yl, zh, zl
};
uint8_t start_byte = 0xAA;
uint16_t adc_data = 0;
float voltage = 0;
float adc_voltage = 0;
double temperatur = 0;
uint16_t i2c_data = 0;
uint8_t mag_data = 0;
uint8_t id = false;
long baro_data = 0;
const float adc_mul = 0.0000503540039;
const float bat_vol_mul = 0.00009155413138;
const float bat_temp_mul = 0.009155413138;
uint8_t uart_tx_buffer[5] = {0};

#define DELAY_MS    100     /**< Timer Delay in milli-seconds. */


static uint8_t tx_data[TX_RX_MSG_LENGTH]; /**< SPI master TX buffer. */
static uint8_t rx_data[TX_RX_MSG_LENGTH]; /**< SPI master RX buffer. */


/**@brief Function for error handling, which is called when an error has occurred. 
 *
 * @param[in] error_code  Error code supplied to the handler.
 * @param[in] line_num    Line number where the handler is called.
 * @param[in] p_file_name Pointer to the file name. 
 */
void app_error_handler(uint32_t error_code, uint32_t line_num, const uint8_t * p_file_name)
{
    for (;;)
    {
        //No implementation needed.
				nrf_gpio_pin_set(LED_0);
				nrf_delay_ms(50);
				nrf_gpio_pin_clear(LED_0);
				nrf_delay_ms(250);
    }
}


/** @brief Function for main application entry.
 */
int main(void)
{
	
		// Set LED pins to output
	  nrf_gpio_cfg_output(LED_0);
		nrf_gpio_cfg_output(LED_1);
  
		// Set spi slave pins to output
		nrf_gpio_cfg_output(SPIM0_MPU_PIN);
		nrf_gpio_cfg_output(SPIM0_ADC1_PIN);
		nrf_gpio_cfg_output(SPIM0_ADC2_PIN);

		// Enable analog voltage supply
		nrf_gpio_cfg_output(ANALOG_VOLTAGE_ENABLE_PIN);
		nrf_gpio_pin_set(ANALOG_VOLTAGE_ENABLE_PIN);

		// Disable all spi slaves
		nrf_gpio_pin_set(SPIM0_MPU_PIN);
		nrf_gpio_pin_set(SPIM0_ADC1_PIN);
		nrf_gpio_pin_set(SPIM0_ADC2_PIN);

		// Configure simple uart connection
		simple_uart_config(RTS_PIN_NUMBER, TX_PIN_NUMBER, CTS_PIN_NUMBER, RX_PIN_NUMBER, HWFC);

		//twi_master_init();
		//ltc2942_init();
		//bmp085_init();
		
		// Init & check mpu9250
		mpu9250_init();	

		//id = mpu9250_whoami();
	
		// Check magnetic field sensor
		//mag_data = mpu9250_mag_whoami();

		// Main loop
    while(1)
    {
			
				//mpu9250_read_acc(&imu_data[0]);           // Output rate 4 kHz
				//mpu9250_read_gyro(&imu_data[6]);          // Output rate 8 kHz
				//mpu9250_read_mag(&imu_data[12]);				// Update rate 100 Hz
			 
				//id = mpu9250_whoami();
				//id = mpu9250_mag_whoami();
			
				// Read both ADCs
				adc_data = ads8320_read(ADC2);
				voltage = adc_data * adc_mul;
			
				// Read battery voltage and temperatur
				//i2c_data = ltc2942_read_temp();
				//temperatur = ((float) i2c_data * bat_temp_mul) - 273.15;
				//i2c_data = ltc2942_read_voltage();
				//voltage = (float) i2c_data * bat_vol_mul;
			
				// Read barometer via I2C
				//temperatur = bmp085_read_temperature();
				//i2c_data = bmp085_read_pressure();

				uart_tx_buffer[0] = (uint8_t)voltage + '0';
				voltage -= (uint8_t)voltage;
				voltage *= 10; 
				uart_tx_buffer[1] = '.';
				uart_tx_buffer[2] = (uint8_t)voltage + '0';
				voltage -= (uint8_t)voltage;
				voltage *= 10;
				uart_tx_buffer[3] = (uint8_t)voltage + '0';
				voltage -= (uint8_t)voltage;
				voltage *= 10;
				uart_tx_buffer[4] = (uint8_t)voltage + '0';


				// Send start byte + imu data via uart
				simple_uart_put(start_byte);
				simple_uart_putbuffer(uart_tx_buffer, 5);
				
				//simple_uart_putbuffer(imu_data, 19);

				//nrf_delay_ms(14);                           // 20.16 ms per loop
				//nrf_gpio_pin_set(LED_0);
				nrf_delay_ms(10);
				//nrf_gpio_pin_clear(LED_0);
				//nrf_delay_ms(250);
			
    }
}

/** @} */
