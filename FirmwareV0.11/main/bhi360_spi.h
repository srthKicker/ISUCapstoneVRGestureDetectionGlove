#ifndef BHI360_spi_H
#define BHI360_spi_H
#include "bhi3.h"  // For function pointer types
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"

/**
 * Simple header file for spi functions that we want to use in main
 * Add any further functions here and define in spi.c
 */
//Callback function to read over spi
int8_t bhi360_spi_read(uint8_t reg_addr, uint8_t *data, uint32_t len, void *intf_ptr);
//Callback function to write over spi
int8_t bhi360_spi_write(uint8_t reg_addr, const uint8_t *data, uint32_t len, void *intf_ptr);
//Callback function to delay for a bit to allow spi lines to settle
//Will adjust to work better with FreeRTOS soon.
void bhi360_delay_us(uint32_t period, void *intf_ptr);

typedef struct spiContext {

} spiContext_t;

#endif //BHI360_spi_H
//intf_ptr