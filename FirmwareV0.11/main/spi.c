#include <stdint.h> //data types
#include "driver/spi_common.h" 
#include "driver/spi_master.h"
#include "bhi360_spi.h" //Header file we are defining
#include <rom/ets_sys.h> //For delay function
#include "bhi3.h" //For constants that the sensor might send over.
#define SPI_TIMEUT_TICKS = pdMS_TO_TICKS(500) // 0.5 second timeout, can change

//Reads a specified number of bytes over spi using the modern form
int8_t bhi360_spi_read(uint8_t regAddr, uint8_t *data, uint32_t len, void *intf_ptr)
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
}
//Allocates on heap instead of on stack to try and fix it
int8_t bhi360_spi_write(uint8_t regAddr, const uint8_t *data, uint32_t len, void *intf_ptr)
{   
    //store intf_ptr as a context type predefined in header
    spiContext_t * cntxt = (spiContext_t *)intf_ptr; 
    //Create buffer to send
    //First byte is the register, next bytes are actual data
    uint8_t *buf = malloc(len + 1);
    if (!buf) return -1; //return an error if buf is empty

    buf[0] = regAddr; // address first
    memcpy(&buf[1], data, len); //Copies the data into the buffer to send

    //Define the transaction
    spi_transaction_t transaction = {
        .length = (len + 1) * 8, //len is num bytes to send
        .tx_buffer = buf,
        .rx_buffer = NULL, //we dont plan on reading anything or storing it
    };

    esp_err_t ret = spi_device_polling_transmit(cntxt->deviceHandle, &transaction);

    free(buf); //get that memory back
    return (ret == ESP_OK) ? 0 : -1;

}


//Simple delay function to pass to the BHI360 drivers.
void bhi360_delay_us(uint32_t period, void *intf_ptr){
    ets_delay_us(period);
}


