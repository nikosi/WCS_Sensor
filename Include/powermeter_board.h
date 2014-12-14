/* Nikolas Simon
 * University of Freiburg 
 * Ocotber 2014
 * 
 * Description of the powermeter V4.3 board
 *
 */

#ifndef POWERMETER_BOARD_H
#define POWERMETER_BOARD_H

#include "nrf_gpio.h"

/*
 * LED pin numbers
 */
#define LED_START      28
#define LED_0          28
#define LED_1          29
#define LED_2					 25		// Not connected
#define LED_STOP       29

#define SER_CONN_ASSERT_LED_PIN     LED_2


/*
 * SPI pin description - 3 slave selects for MPU, ADC1 and ADC2
 */
#define SPIM0_SCK_PIN       15u     /**< SPI clock GPIO pin number. */
#define SPIM0_MOSI_PIN      13u     /**< SPI Master Out Slave In GPIO pin number. */
#define SPIM0_MISO_PIN      14u     /**< SPI Master In Slave Out GPIO pin number. */
#define SPIM0_MPU_PIN       10u     /**< SPI Slave Select GPIO pin number. */
#define SPIM0_ADC1_PIN			12u			/**< SPI Slave Select GPIO pin number. */
#define SPIM0_ADC2_PIN			11u			/**< SPI Slave Select GPIO pin number. */


/** @def  TX_RX_MSG_LENGTH
 * number of bytes to transmit and receive. This amount of bytes will also be tested to see that
 * the received bytes from slave are the same as the transmitted bytes from the master */
#define TX_RX_MSG_LENGTH   20


/*
 * UART pin description - connected to FTDI-Chip for USB communication
 */
#define RX_PIN_NUMBER  20    // UART RX pin number.
#define TX_PIN_NUMBER  19    // UART TX pin number.
#define CTS_PIN_NUMBER 18    // UART Clear To Send pin number. Not used if HWFC is set to false. 
#define RTS_PIN_NUMBER 17    // UART Request To Send pin number. Not used if HWFC is set to false. 
#define HWFC           false // UART hardware flow control.


/*
 * I2C pin description - connected to gas fuel gauge for checking battery voltage and temp.
 */ 
#define TWI_MASTER_CONFIG_CLOCK_PIN_NUMBER 23u		// I2C clock pin number
#define TWI_MASTER_CONFIG_DATA_PIN_NUMBER 22u		// I2C data bus pin number


/*
 * Interrupt pin number
 */ 
#define MPU_INT_PIN_NUMBER 		6u		// MPU interrupt pin number
#define ALERT_INT_PIN_NUMBER	5u		// Alert interrupt pin number of battery control


/*
 * Enable signal of analog voltage converter
 */ 
#define ANALOG_VOLTAGE_ENABLE_PIN 4u		// Enable pin number


#define BUTTON_START   0
#define BUTTON_0       0
#define BUTTON_1       1
#define BUTTON_2       2
#define BUTTON_3       3
#define BUTTON_4       4
#define BUTTON_5       5
#define BUTTON_6       6
#define BUTTON_7       7
#define BUTTON_STOP    7
#define BUTTON_PULL    NRF_GPIO_PIN_NOPULL


/* serialization APPLICATION board
#define SER_APP_RX_PIN              16    // UART RX pin number.
#define SER_APP_TX_PIN              17    // UART TX pin number.
#define SER_APP_CTS_PIN             18    // UART Clear To Send pin number.
#define SER_APP_RTS_PIN             19    // UART Request To Send pin number.

#if 0
#define SER_APP_SPIM0_SCK_PIN       20     // SPI clock GPIO pin number.
#define SER_APP_SPIM0_MOSI_PIN      17     // SPI Master Out Slave In GPIO pin number
#define SER_APP_SPIM0_MISO_PIN      16     // SPI Master In Slave Out GPIO pin number
#define SER_APP_SPIM0_SS_PIN        21     // SPI Slave Select GPIO pin number
#define SER_APP_SPIM0_RDY_PIN       19     // SPI READY GPIO pin number
#define SER_APP_SPIM0_REQ_PIN       18     // SPI REQUEST GPIO pin number
#else
#define SER_APP_SPIM0_SCK_PIN       23     // SPI clock GPIO pin number.
#define SER_APP_SPIM0_MOSI_PIN      20     // SPI Master Out Slave In GPIO pin number
#define SER_APP_SPIM0_MISO_PIN      22     // SPI Master In Slave Out GPIO pin number
#define SER_APP_SPIM0_SS_PIN        21     // SPI Slave Select GPIO pin number
#define SER_APP_SPIM0_RDY_PIN       29     // SPI READY GPIO pin number
#define SER_APP_SPIM0_REQ_PIN       28     // SPI REQUEST GPIO pin number

#endif

// serialization CONNECTIVITY board
#if 0
#define SER_CON_RX_PIN              17    // UART RX pin number.
#define SER_CON_TX_PIN              16    // UART TX pin number.
#define SER_CON_CTS_PIN             19    // UART Clear To Send pin number. Not used if HWFC is set to false.
#define SER_CON_RTS_PIN             18    // UART Request To Send pin number. Not used if HWFC is set to false.
#else
#define SER_CON_RX_PIN              16    // UART RX pin number.
#define SER_CON_TX_PIN              17    // UART TX pin number.
#define SER_CON_CTS_PIN             18    // UART Clear To Send pin number. Not used if HWFC is set to false.
#define SER_CON_RTS_PIN             19    // UART Request To Send pin number. Not used if HWFC is set to false.
#endif

#if 0
#define SER_CON_SPIS_SCK_PIN        20    // SPI SCK signal.
#define SER_CON_SPIS_MISO_PIN       16    // SPI MISO signal.
#define SER_CON_SPIS_MOSI_PIN       17    // SPI MOSI signal.
#define SER_CON_SPIS_CSN_PIN        21    // SPI CSN signal.
#define SER_CON_SPIS_RDY_PIN        19     // SPI READY GPIO pin number.
#define SER_CON_SPIS_REQ_PIN        18     // SPI REQUEST GPIO pin number.
#else
#define SER_CON_SPIS_SCK_PIN        23    // SPI SCK signal.
#define SER_CON_SPIS_MOSI_PIN       22    // SPI MOSI signal.
#define SER_CON_SPIS_MISO_PIN       20    // SPI MISO signal.
#define SER_CON_SPIS_CSN_PIN        21    // SPI CSN signal.
#define SER_CON_SPIS_RDY_PIN        29     // SPI READY GPIO pin number.
#define SER_CON_SPIS_REQ_PIN        28     // SPI REQUEST GPIO pin number.
#endif*/



#endif //POWERMETER_BOARD_H
