#ifndef I2C_H
#define I2C_H
#include "bhi360.h"  // For function pointer types

/**
 * Simple header file for I2C functions that we want to use in main
 * Add any further functions here and define in i2c.c
 */
int8_t bhi360_i2c_read(uint8_t reg_addr, uint8_t *data, uint32_t len, void *intf_ptr);
int8_t bhi360_i2c_write(uint8_t reg_addr, const uint8_t *data, uint32_t len, void *intf_ptr);
void bhi360_delay_us(uint32_t period, void *intf_ptr);

/**
 * Stores context for the sensor, will use in the future to switch addresses
 * For now, remains constant in main
 */
typedef struct bhi360_cntxt{
    i2c_port_t i2cPortNum;
    uint8_t i2cAddress;

} bhi360_cntxt_t;
#endif
//intf_ptr