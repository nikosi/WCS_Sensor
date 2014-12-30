static uint8_t m_tx_data_spi[TX_RX_MSG_LENGTH]; /**< SPI master TX buffer. */
static uint8_t m_rx_data_spi[TX_RX_MSG_LENGTH]; /**< SPI master RX buffer. */

static volatile bool m_transfer_completed = true;


/**@brief Function for initializing a SPI master driver.
 *
 * @param[in] spi_master_instance       An instance of SPI master module.
 * @param[in] spi_master_event_handler  An event handler for SPI master events.
 * @param[in] lsb                       Bits order LSB if true, MSB if false.
 */
static void spi_master_init(spi_master_hw_instance_t spi_master_instance, 
                            spi_master_event_handler_t spi_master_event_handler,
                            const bool lsb)
{
    uint32_t err_code = NRF_SUCCESS;

    //Configure SPI master.
    spi_master_config_t spi_config = SPI_MASTER_INIT_DEFAULT;
    
    switch (spi_master_instance)
    {
        #ifdef SPI_MASTER_0_ENABLE
        case SPI_MASTER_0:
        {
            spi_config.SPI_Pin_SCK = SPIM0_SCK_PIN;
            spi_config.SPI_Pin_MISO = SPIM0_MISO_PIN;
            spi_config.SPI_Pin_MOSI = SPIM0_MOSI_PIN;
            spi_config.SPI_Pin_SS = SPIM0_SS_PIN;
        }
        break; 
        #endif /* SPI_MASTER_0_ENABLE */

        #ifdef SPI_MASTER_1_ENABLE
        case SPI_MASTER_1:
        {
            spi_config.SPI_Pin_SCK = SPIM1_SCK_PIN;
            spi_config.SPI_Pin_MISO = SPIM1_MISO_PIN;
            spi_config.SPI_Pin_MOSI = SPIM1_MOSI_PIN;
            spi_config.SPI_Pin_SS = SPIM1_SS_PIN;
        }
        break;
        #endif /* SPI_MASTER_1_ENABLE */
        
        default:
            break;
    }
    
    spi_config.SPI_CONFIG_ORDER = (lsb ? SPI_CONFIG_ORDER_LsbFirst : SPI_CONFIG_ORDER_MsbFirst);
    
    err_code = spi_master_open(spi_master_instance, &spi_config);
    APP_ERROR_CHECK(err_code);
    
    //Register event handler for SPI master.
    spi_master_evt_handler_reg(spi_master_instance, spi_master_event_handler);
}





/**@brief Handler for SPI0 master events.
 *
 * @param[in] spi_master_evt    SPI master event.
 */
void spi_master_0_event_handler(spi_master_evt_t spi_master_evt)
{
    bool result = false;
    switch (spi_master_evt.evt_type)
    {
        case SPI_MASTER_EVT_TRANSFER_COMPLETED:
            //Check if received data is correct.
            result = check_buf_equal(m_tx_data_spi, m_rx_data_spi, TX_RX_MSG_LENGTH);
            if (!result)
            {
                //Set LED high to indicate that error has occurred.
                nrf_gpio_pin_set(ERROR_PIN_SPI0);
            }
            APP_ERROR_CHECK_BOOL(result);
        
            //Close SPI master.
            spi_master_close(SPI_MASTER_0);
        
            m_transfer_completed = true;
            break;
        
        default:
            //No implementation needed.
            break;
    }
}







/**@brief Function for sending and receiving data.
 *
 * @param[in]   spi_master_hw_instance  SPI master instance.
 * @param[in]   p_tx_data               A pointer to a buffer TX.
 * @param[out]  p_rx_data               A pointer to a buffer RX.
 * @param[in]   len                     A length of the data buffers.
 */
static void spi_send_recv(const spi_master_hw_instance_t spi_master_hw_instance,
                          uint8_t * const p_tx_data,
                          uint8_t * const p_rx_data,
                          const uint16_t len)
{
    //Initalize buffers.
    init_buf(p_tx_data, p_rx_data, len);
    
    //Start transfer.
    uint32_t err_code = spi_master_send_recv(spi_master_hw_instance, p_tx_data, len, p_rx_data, len);
    APP_ERROR_CHECK(err_code);
}