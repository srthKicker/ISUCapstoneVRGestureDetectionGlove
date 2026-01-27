#ifndef BHI360_I2C_H
#define BHI360_I2C_H
#include "bhi3.h"  // For function pointer types

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

typedef struct i2cContext {
    i2c_device_config_t devConfig;
    i2c_master_bus_config_t busConfig;
    i2c_master_dev_handle_t devHandle;
    i2c_master_bus_handle_t busHandle;
} i2cContext_t;

#endif //BHI360_I2C_H
//intf_ptr