#include <stdint.h> //data types
#include "driver/spi_common.h" 
#include "driver/spi_master.h"
#include "bhi360_spi.h" //Header file we are defining
#include <rom/ets_sys.h> //For delay function
#include "bhi3.h" //For constants that the sensor might send over.
#define SPI_TIMEUT_TICKS = pdMS_TO_TICKS(500) // 0.5 second timeout, can change

//Reads a specified number of bytes over spi using the modern form
/*int8_t bhi360_spi_read(uint8_t regAddr, uint8_t *data, uint32_t len, void *intf_ptr)
{   
    //store intf_ptr as a context type predefined in header
    spiContext_t * cntxt = (spiContext_t *)intf_ptr; 

    //Define the transaction
    spi_transaction_t transaction = {
        .length = 8, //8 bits to send
        .tx_buffer = &regAddr, //send the address (1 byte)
        .rx_buffer = data, //store read bits in data
        .rxlength = len*8, //read len bytes after register write
    };

    esp_err_t ret = spi_device_polling_transmit(cntxt->deviceHandle, &transaction);
    return (ret == ESP_OK) ? 0 : -1;//return negative one if it fails
}*/
int8_t bhi360_spi_read(uint8_t regAddr, uint8_t *data, uint32_t len, void *intf_ptr)
{   
    spiContext_t *cntxt = (spiContext_t *)intf_ptr; 

    spi_transaction_t transaction = {
        .cmd   = regAddr | 0x80,       // Set MSB for Read
        .length = len * 8,             // Length of DATA only
        .rx_buffer = data,             // Data goes here starting at index 0
    };
    if(len <= 4) { transaction.flags = SPI_TRANS_USE_RXDATA; } //we'll see if this helps

    // To use .cmd, ensure 'command_bits = 8' is in your dev_config
    esp_err_t ret = spi_device_polling_transmit(cntxt->deviceHandle, &transaction);
    return (ret == ESP_OK) ? 0 : -1;
}

//what was i even doing with those last iterations
//Writes over SPI. We use CMD in all of our 
int8_t bhi360_spi_write(uint8_t regAddr, const uint8_t *data, uint32_t len, void *intf_ptr)
{   
    spiContext_t *cntxt = (spiContext_t *)intf_ptr; 

    spi_transaction_t transaction = {
        .cmd = regAddr & 0x7F,      // Write command byte
        .tx_buffer = data,          // Only payload here
        .length = len * 8,          // Data length in bits because len is bytes
    };

    esp_err_t ret = spi_device_polling_transmit(cntxt->deviceHandle, &transaction);
    return (ret == ESP_OK) ? 0 : -1;
}



//Simple delay function to pass to the BHI360 drivers.
void bhi360_delay_us(uint32_t period, void *intf_ptr){
    ets_delay_us(period);
}


