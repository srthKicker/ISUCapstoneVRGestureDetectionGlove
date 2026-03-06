#include <stdint.h> //data types
#include "driver/spi_common.h" 
#include "driver/spi_master.h"
#include "bhi360_spi.h" //Header file we are defining
#include <rom/ets_sys.h> //For delay function
#include "bhi3.h" //For constants that the sensor might send over.
#define SPI_TIMEUT_TICKS pdMS_TO_TICKS(500) // 0.5 second timeout, can change

int8_t bhi360_spi_read(uint8_t regAddr, uint8_t *data, uint32_t len, void *intf_ptr)
{   
    spiContext_t *cntxt = (spiContext_t *)intf_ptr; 

    spi_transaction_t transaction = {
        .cmd   = regAddr | 0x80,       // Set MSB for Read
        .length = len * 8,             // Length of DATA only
        .rx_buffer = data,             // Data goes here starting at index 0
    };

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


/*void Quaternion::qMultiply(const Quaternion& dQ) {
    float newW = w*dQ.w - x*dQ.x - y*dQ.y - z*dQ.z;
    float newX = w*dQ.x + x*dQ.w + y*dQ.z - z*dQ.y;
    float newY = w*dQ.y - x*dQ.z + y*dQ.w + z*dQ.x;
    float newZ = w*dQ.z + x*dQ.y - y*dQ.x + z*dQ.w;

    w = newW;
    x = newX;
    y = newY;
    z = newZ; //trust me bro its gotta be this way */
    /*
    As an explaination, multiplying 2 quaternions applies rotations in order
    Think of quaternions as rotations.
    So, q1 * q2 rotates from origin by q1, then rotates from that point by q2.
    Not commutative!
}*/
