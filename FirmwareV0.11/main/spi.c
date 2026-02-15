#include <stdint.h> //data types
#include "driver/spi_common.h" //more modern i2c stuff to change over to
#include "driver/spi_master.h"
#include "bhi360_spi.h" //Header file we are defining
#include <rom/ets_sys.h> //For delay function
#include "bhi3.h" //For constants that the sensor might send over.
#define SPI_TIMEOUT_MS 500 // 1 second timeout, can change

//Reads a specified number of bytes over spi using the modern form
int8_t bhi360_spi_read(uint8_t regAddr, uint8_t *data, uint32_t len, void *intf_ptr)
{
    
}


//Allocates on heap instead of on stack to try and fix it
int8_t bhi360_spi_write(uint8_t regAddr, const uint8_t *data, uint32_t len, void *intf_ptr)
{

}


//Simple delay function to pass to the BHI360 drivers.
void bhi360_delay_us(uint32_t period, void *intf_ptr){
    ets_delay_us(period);
}


