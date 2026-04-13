#ifndef BHI360_I2C_H
#define BHI360_I2C_H
//#include "bhi3.h"  // For function pointer types
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "driver/i2c_master.h" //Modern i2c functions to change over to
#include "driver/i2c_types.h" //more modern i2c stuff to change over to

/**
 * Simple header file for I2C functions that we want to use in main
 * Add any further functions here and define in i2c.c
 */
//Callback function to read over I2C
int8_t bhi360_i2c_read(uint8_t reg_addr, uint8_t *data, uint32_t len, void *intf_ptr);
//Callback function to write over I2C
int8_t bhi360_i2c_write(uint8_t reg_addr, const uint8_t *data, uint32_t len, void *intf_ptr);
//Callback function to delay for a bit to allow i2c lines to settle
//Will adjust to work better with FreeRTOS soon.
void bhi360_delay_us(uint32_t period, void *intf_ptr);


//Context struct to be passed to each bhi360_i2c function. 
//Will 
typedef struct i2cContext {
    i2c_master_dev_handle_t devHandle;
    i2c_master_dev_handle_t muxDevHandle;
    uint8_t muxChannel;
} i2cContext_t;

//Function to allow us to communicate through a MUX
//Sends a byte to the mux address (locked at 0x70) to tell it which channel to open
esp_err_t selectMuxChannel(i2cContext_t *cntxt);

#endif //BHI360_I2C_H
//intf_ptr