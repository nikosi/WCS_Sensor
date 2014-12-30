/* 
 * Nikolas Simon
 * University of Freiburg 
 * October 2014
 * 
 * Functions for initializing and reading 
 * MPU 9250 sensor via SPI @ 1MHz
 *
 */
 
 
 #include "mpu_9250.h"
 
 #include "nrf_gpio.h"
 #include "powermeter_board.h"

 #include "nrf_delay.h"
 #include "nrf_error.h"
 
 #include "spi_master.h"
 #include "nrf51_bitfields.h"
 #include "app_util_platform.h"
 
 

static uint8_t tx_data[TX_RX_MSG_LENGTH]; /*!< SPI TX buffer */
static uint8_t rx_data[TX_RX_MSG_LENGTH]; /*!< SPI RX buffer */

volatile bool spi_done = false;

static void spi_MPU_init(spi_master_hw_instance_t spi_master_instance, spi_master_event_handler_t spi_master_event_handler, const bool lsb)
{
    uint32_t err_code = NRF_SUCCESS;

    //Configure SPI master.
    spi_master_config_t spi_config = SPI_MASTER_INIT_DEFAULT;

		spi_config.SPI_Pin_SCK = SPIM0_SCK_PIN;
		spi_config.SPI_Pin_MISO = SPIM0_MISO_PIN;
		spi_config.SPI_Pin_MOSI = SPIM0_MOSI_PIN;
		spi_config.SPI_Pin_SS = SPIM0_MPU_PIN;
   
    spi_config.SPI_CONFIG_ORDER = (lsb ? SPI_CONFIG_ORDER_LsbFirst : SPI_CONFIG_ORDER_MsbFirst);
    
    err_code = spi_master_open(spi_master_instance, &spi_config);
    APP_ERROR_CHECK(err_code);
    
    //Register event handler for SPI master.
    spi_master_evt_handler_reg(spi_master_instance, spi_master_event_handler);
}


/**@brief Handler for SPI0 master events. **/
void spi_master_mpu_event_handler(spi_master_evt_t spi_master_evt)
{
    switch (spi_master_evt.evt_type)
    {
        case SPI_MASTER_EVT_TRANSFER_COMPLETED:
						// Set SPI done flag
            spi_done = true;
					break;
        
        default:
            //No implementation needed.
            break;
    }
}

/*-----------------------------------------------------------------------------------------------
 * INITIALIZATION
-----------------------------------------------------------------------------------------------*/
bool mpu9250_init(void){

    // Init values: [value][address]
    uint8_t MPU_Init_Data[18][2] = {
      
        {0x80, MPUREG_PWR_MGMT_1},              // Reset Device
        {0x01, MPUREG_PWR_MGMT_1},              // Clock Source
        {0x00, MPUREG_PWR_MGMT_2},              // Enable Acc & Gyro
        //{low_pass_filter, MPUREG_CONFIG},       // Use DLPF set Gyroscope bandwidth 184Hz, temperature bandwidth 188Hz
        
        {0x18, MPUREG_GYRO_CONFIG},             // +-2000dps
        {0x08, MPUREG_ACCEL_CONFIG},            // +-4G
        
        //{0x09, MPUREG_ACCEL_CONFIG_2},          // Set Acc Data Rates, Enable Acc LPF , Bandwidth 184Hz
      
        {0x20, MPUREG_USER_CTRL},               // Enable I2C Master mode 
        {0x0D, MPUREG_I2C_MST_CTRL},            // Set I2C Master to 400KHz
        
        {AK8963_I2C_ADDR, MPUREG_I2C_SLV0_ADDR},// Set the I2C slave addres of AK8963 and set for write.

        {AK8963_CNTL2, MPUREG_I2C_SLV0_REG},    // I2C slave 0 register address from where to begin data transfer
        {0x01, MPUREG_I2C_SLV0_DO},             // Reset AK8963
        {0x81, MPUREG_I2C_SLV0_CTRL},           // Enable I2C and set 1 byte
 
        {AK8963_CNTL1, MPUREG_I2C_SLV0_REG},    // I2C slave 0 register address from where to begin data transfer
        {0x16, MPUREG_I2C_SLV0_DO},             // Register value to continuous measurement mode2 (100Hz) and 16bit
        {0x81, MPUREG_I2C_SLV0_CTRL}            // Enable I2C and set 1 byte
        
        // Enable wake-up on motion config - missing //
        
    };

		
    //uint32_t *spi_base_address = spi_master_init(SPI0, SPI_MODE0, 0);
		spi_MPU_init(SPI_MASTER_0, spi_master_mpu_event_handler, false);
    
    uint8_t i = 0;
    for(i = 0; i < 14; i++) {
      
        tx_data[0] = MPU_Init_Data[i][1];
        tx_data[1] = MPU_Init_Data[i][0];
        
        //spi_master_tx_rx(spi_base_address, SPI_MPU_PIN0, 2, (const uint8_t *)tx_data, rx_data);
				spi_done = false;
				spi_master_send_recv(SPI_MASTER_0, tx_data, 2, rx_data, 2);
				while(!spi_done);
        
        nrf_delay_ms(1);
    }

    return 0;
}


bool mpu9250_whoami(void)
{
    // Prepare tx buffer
    tx_data[0] = 0xF5;                             // Who am I register

    // Send and receive
		spi_done = false;
		spi_MPU_init(SPI_MASTER_0, spi_master_mpu_event_handler, false);
		spi_master_send_recv(SPI_MASTER_0, tx_data, 2, rx_data, 2);
		while(!spi_done);
    
    bool IDconfirmed = false;
    // Check slave ID
    if (rx_data[1] == 0x71) IDconfirmed = true;    // Check salve ID
       
    return IDconfirmed;  
}

void mpu9250_read_acc(uint8_t *imu_data)
{
    // Prepare tx buffer
    tx_data[0] = 0xBB;	           // Read Bit and acc_x_h register address
    
    // Send and receive
		spi_done = false;
		spi_MPU_init(SPI_MASTER_0, spi_master_mpu_event_handler, false);
		spi_master_send_recv(SPI_MASTER_0, tx_data, 7, rx_data, 7);
		while(!spi_done);
    
    // Combine high and low byte and store in given imu data buffer 
    *imu_data = rx_data[1];
    imu_data++;  
    *imu_data = rx_data[2];
    imu_data++;
    *imu_data = rx_data[3];
    imu_data++;
    *imu_data = rx_data[4];
    imu_data++;
    *imu_data = rx_data[5];
    imu_data++;
    *imu_data = rx_data[6];
}

void mpu9250_read_gyro(uint8_t *imu_data)
{
    // Prepare tx buffer
    tx_data[0] = 0xC3;		    // Read Bit and acc_x_h register address
    
    // Send and receive
		spi_done = false;
		spi_MPU_init(SPI_MASTER_0, spi_master_mpu_event_handler, false);
		spi_master_send_recv(SPI_MASTER_0, tx_data, 7, rx_data, 7);
		while(!spi_done);
    
    // Combine high and low byte and store in given imu data buffer 
    *imu_data = rx_data[1];
    imu_data++;  
    *imu_data = rx_data[2];
    imu_data++;
    *imu_data = rx_data[3];
    imu_data++;
    *imu_data = rx_data[4];
    imu_data++;
    *imu_data = rx_data[5];
    imu_data++;
    *imu_data = rx_data[6];
}


uint8_t mpu9250_mag_whoami(void){
  
    uint8_t response;
	
    spi_MPU_init(SPI_MASTER_0, spi_master_mpu_event_handler, false);
    
    tx_data[0] = MPUREG_I2C_SLV0_ADDR;          // Select I2C slave 0 register
    tx_data[1] = AK8963_I2C_ADDR|READ_FLAG;     // Address of AK8963 + read bit
    tx_data[2] = AK8963_WIA;                    // I2C slave 0 register address from where to begin data transfer
    tx_data[3] = 0x81;                          // Set enable bit to read one byte from AK8963
 		
		spi_done = false;   
		spi_master_send_recv(SPI_MASTER_0, tx_data, 4, rx_data, 4);
		while(!spi_done);	
    
    nrf_delay_ms(1);
    
		// Read sensor data register 
    tx_data[0] = (MPUREG_EXT_SENS_DATA_00|READ_FLAG); 
    
    spi_done = false;   
		spi_master_send_recv(SPI_MASTER_0, tx_data, 4, rx_data, 4);
		while(!spi_done);	
		
    response = rx_data[1];
 
    return response;
}


void mpu9250_read_mag(uint8_t *imu_data){

    //uint32_t *spi_base_address = spi_master_init(SPI0, SPI_MODE0, 0);
    
    tx_data[0] = MPUREG_I2C_SLV0_ADDR;          // Select I2C slave 0 register
    tx_data[1] = AK8963_I2C_ADDR|READ_FLAG;     // Address of AK8963 + read bit
    tx_data[2] = AK8963_HXL;                    // I2C slave 0 register address from where to begin data transfer
    tx_data[3] = 0x87;                          // Set enable bit to read one byte from AK8963
    
    //spi_master_tx_rx(spi_base_address, SPI_MPU_PIN0, 4, (const uint8_t *)tx_data, rx_data);
    
    nrf_delay_ms(1);
    
    tx_data[0] = MPUREG_EXT_SENS_DATA_00|READ_FLAG;
    
    //spi_master_tx_rx(spi_base_address, SPI_MPU_PIN0, 7, (const uint8_t *)tx_data, rx_data);
    
    // Combine high and low byte and store in given imu data buffer 
    *imu_data = rx_data[2];
    imu_data++;  
    *imu_data = rx_data[1];
    imu_data++;
    *imu_data = rx_data[4];
    imu_data++;
    *imu_data = rx_data[3];
    imu_data++;
    *imu_data = rx_data[6];
    imu_data++;
    *imu_data = rx_data[5];
}
