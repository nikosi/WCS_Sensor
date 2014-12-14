/* 
 * Nikolas Simon
 * University of Freiburg 
 * October 2014
 * 
 * Functions for initializing and reading 
 * LTC 2942 Battery Gas Gauge with voltage measurement
 *
 */
 
 
 #include "ltc_2942.h"
 
 #include "nrf_gpio.h"
 #include "powermeter_board.h"
 #include "twi_master.h"
 
const float battery_max	= 3.4;
const float voltage_mul = 0.00009155413138;	//Conversion factor 6V/16Bit

 void ltc2942_init(void)
{
    // Set control register
     uint8_t i2c_data[2] = {0};
  
    i2c_data[0] = CONTROL_REG;          // Control register address
    i2c_data[1] = 0xF8;                  // 1111 1000
  
    // Send data to slave device
    twi_master_transfer(LTC_ADDRESS_WRITE, i2c_data, 2, TWI_ISSUE_STOP);
    
    /* Set threshould limits
    ic2_data[0] = ;
    ic2_data[1] = ;
    ic2_data[2] = ;
    ic2_data[3] = ;*/
}

void ltc2942_test(void)
{
    // Prepare buffer
    uint8_t i2c_data[2] = {0};
  
    i2c_data[0] = STATUS_REG;           // Status register address
    
    // Receive status register
    twi_master_transfer(LTC_ADDRESS_WRITE, i2c_data, 1, TWI_ISSUE_STOP); 
    
    // Read voltage register
    twi_master_transfer(LTC_ADDRESS_READ, i2c_data, 1, TWI_ISSUE_STOP);
}

/*******************************************************************************
* Reading battery voltage und returning battery levele in percentage
*******************************************************************************/ 
uint8_t ltc2942_read_voltage(void)
{
    // Prepare buffer and return value;
    uint8_t i2c_data[5] = {0};
    uint16_t ltc2942_data = 0;
		uint8_t battery_level = 0;
		float current_voltage = 0;
		
    i2c_data[0] = VOLTAGE_MSB;          // Voltage register address (MSB)
    
    // Write voltage register address
    twi_master_transfer(LTC_ADDRESS_WRITE, i2c_data, 1, TWI_DONT_ISSUE_STOP); 
    
    // Read voltage register
    twi_master_transfer(LTC_ADDRESS_READ, i2c_data, 5, TWI_DONT_ISSUE_STOP); 
    
    // Combine H and L and convert into voltage
    ltc2942_data = (i2c_data[0]<<8) | (i2c_data[1]);
		
		current_voltage = ltc2942_data * voltage_mul; 
		
		// Calculate battery level in %
		battery_level = (uint8_t)(((current_voltage - 3)/1.2 ) * 100); 		/* (x-3)/1.2 * 100*/
		if(battery_level > 100) battery_level = 100;

    return battery_level;
}

/*******************************************************************************
* Reading and returning 10 Bit value of the battery temperatur
*******************************************************************************/    
uint16_t ltc2942_read_temp(void)
{
    // Prepare buffer and return value;
    uint8_t i2c_data[2] = {0};
    uint16_t temperatur = 0;
    
    i2c_data[0] = TEMP_MSB;          // Voltage register address (MSB)
    
    // Write voltage register address
		twi_master_transfer(LTC_ADDRESS_WRITE, i2c_data, 1, TWI_DONT_ISSUE_STOP); 
    
    // Read voltage register
    twi_master_transfer(LTC_ADDRESS_READ, i2c_data, 2, TWI_DONT_ISSUE_STOP); 
    
    // Combine H and L and convert into temperature
    temperatur = (i2c_data[0]<<8) | (i2c_data[1]);
    // temperatur = (i2c_data * 0.009155413138) - 272.15;

    return temperatur;
}
