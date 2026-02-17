#include <stdint.h>
#include "driver/spi_common.h" 
#include "driver/spi_master.h"
#include "bhi360_spi.h"
#include "freertos/FreeRTOS.h"  // vTaskDelay
#include "bhi3.h"

int8_t bhi360_spi_read(uint8_t regAddr, uint8_t *data, uint32_t len, void *intf_ptr)
{   
    spiContext_t * cntxt = (spiContext_t *)intf_ptr; 
    uint8_t addr = regAddr | 0x80;  // BHI360 SPI: bit7=1 for read [web:1]

    spi_transaction_t transaction = {
        .length = 8,
        .tx_buffer = &addr,
        .rx_buffer = data, 
        .rxlength = len*8,
    };

    esp_err_t ret = spi_device_polling_transmit(cntxt->deviceHandle, &transaction);
    return (ret == ESP_OK) ? 0 : -1;
}

int8_t bhi360_spi_write(uint8_t regAddr, const uint8_t *data, uint32_t len, void *intf_ptr)
{   
    spiContext_t * cntxt = (spiContext_t *)intf_ptr; 
    uint8_t *buf = malloc(len + 1);
    if (!buf) return -1;

    buf[0] = regAddr & 0x7F;  // bit7=0 for write [web:1]
    memcpy(&buf[1], data, len);

    spi_transaction_t transaction = {
        .length = (len + 1) * 8,
        .tx_buffer = buf,
        .rx_buffer = NULL,
    };

    esp_err_t ret = spi_device_polling_transmit(cntxt->deviceHandle, &transaction);
    free(buf);
    return (ret == ESP_OK) ? 0 : -1;
}

void bhi360_delay_us(uint32_t period, void *intf_ptr){
    vTaskDelay(pdMS_TO_TICKS(1));  // Non-blocking
}
