/* 
 * Nikolas Simon
 * University of Freiburg 
 * November 2014
 * 
 * Function for reading MPU 9250 sensor via SPI @ 1MHz
 *
 */
 
 
 #include "ADS8320.h"
 #include "nrf_gpio.h"
 #include "powermeter_board.h"
 #include "spi_master.h"
 #include "nrf51_bitfields.h"
 #include "app_util_platform.h"
 

static uint8_t tx_data[TX_RX_MSG_LENGTH]; /*!< SPI TX buffer */
static uint8_t rx_data[TX_RX_MSG_LENGTH]; /*!< SPI RX buffer */

volatile bool spi_ads_done = false;

 
static void spi_ads_init(spi_master_hw_instance_t spi_master_instance, spi_master_event_handler_t spi_master_event_handler, const bool lsb, uint8_t slave_select)
{
    uint32_t err_code = NRF_SUCCESS;

    //Configure SPI master.
    spi_master_config_t spi_config = SPI_MASTER_INIT_DEFAULT;

		spi_config.SPI_Pin_SCK = SPIM0_SCK_PIN;
		spi_config.SPI_Pin_MISO = SPIM0_MISO_PIN;
		spi_config.SPI_Pin_MOSI = SPIM0_MOSI_PIN;
		spi_config.SPI_Pin_SS = slave_select;
   
    spi_config.SPI_CONFIG_ORDER = (lsb ? SPI_CONFIG_ORDER_LsbFirst : SPI_CONFIG_ORDER_MsbFirst);
    
    err_code = spi_master_open(spi_master_instance, &spi_config);
    APP_ERROR_CHECK(err_code);
    
    //Register event handler for SPI master.
    spi_master_evt_handler_reg(spi_master_instance, spi_master_event_handler);
} 


/**@brief Handler for SPI0 master events. **/
void spi_master_ads_event_handler(spi_master_evt_t spi_master_evt)
{
    switch (spi_master_evt.evt_type)
    {
        case SPI_MASTER_EVT_TRANSFER_COMPLETED:
						// Set SPI done flag
            spi_ads_done = true;
					break;
        
        default:
            //No implementation needed.
            break;
    }
}

 
/*******************************************************************************
* Reading and returning 16 Bit value of an ADC (ADC1 or ADC2)
*******************************************************************************/
uint16_t ads8320_read(selector select)
{
    uint16_t adc_value = 0;
    uint8_t adc_pin;
    
    if (select == ADC1)
    {
        adc_pin = SPIM0_ADC1_PIN;
    } else
    {
        adc_pin = SPIM0_ADC2_PIN;
    }
    
    // Send and receive
		spi_ads_done = false;
		spi_ads_init(SPI_MASTER_0, spi_master_ads_event_handler, false, adc_pin);
		spi_master_send_recv(SPI_MASTER_0, tx_data, 5, rx_data, 5);
		while(!spi_ads_done);
    
    adc_value = (rx_data[0]<<14) | (rx_data[1]<<6) | (rx_data[2]>>2);    
    
    return adc_value;
}

/*
    uart_tx_buffer[0] = 'L';
    uart_tx_buffer[1] = ':';
    uart_tx_buffer[2] = ' ';
    uart_tx_buffer[3] = (uint8_t)voltage + '0';
    voltage -= (uint8_t)voltage;
    voltage *= 10; 
    uart_tx_buffer[4] = '.';
    uart_tx_buffer[5] = (uint8_t)voltage + '0';
    voltage -= (uint8_t)voltage;
    voltage *= 10;
    uart_tx_buffer[6] = (uint8_t)voltage + '0';
    voltage -= (uint8_t)voltage;
    voltage *= 10;
    uart_tx_buffer[7] = (uint8_t)voltage + '0';
    uart_tx_buffer[8] = ' ';
    uart_tx_buffer[9] = 'V';
    uart_tx_buffer[10] = ' ';
    uart_tx_buffer[11] = ' ';
*/