#include <stdint.h> //data types
#include "driver/i2c_master.h" //Modern i2c functions to change over to
#include "driver/i2c_types.h" //more modern i2c stuff to change over to
#include "bhi360_i2c.h" //Header file we are defining
#include <rom/ets_sys.h> //For delay function
#include "bhi3.h" //For constants that the sensor might send over.
//#define SDA_0 21
//#define SCL_0 22
//#define BEGIN_DELAY_TICKS 10
#define I2C_TIMEOUT_MS 1000 // 1 second timeout, can change

//Reads a specified number of bytes over i2c using the modern form
//Pass a i2c_master_dev_handle_t into intf_ptr as &devHandleName
int8_t bhi360_i2c_read(uint8_t regAddr, uint8_t *data, uint32_t len, void *intf_ptr)
{
    // Cast the interface pointer to the ESP-IDF I2C device handle
    i2cContext_t * cntxt = (i2cContext_t *)intf_ptr;
    i2c_master_dev_handle_t devHandle = cntxt->devHandle;
    // First write the register we wanna read from
    //Then read length from that register
    esp_err_t ret = i2c_master_transmit_receive(
        devHandle,
        &regAddr,        // write buffer: register address
        1,               // write size (we want to write a 1 byte register address)
        data,            // read buffer (where we store the read data)
        len,             // read size
        100              // timeout in ms
    );

    return (ret == ESP_OK) ? 0 : -1;
}

//Writes a specified number of bytes over i2c using the modern form
int8_t bhi360_i2c_write(uint8_t regAddr, const uint8_t *data, uint32_t len, void *intf_ptr)
{
    i2cContext_t * cntxt = (i2cContext_t *)intf_ptr;
    i2c_master_dev_handle_t devHandle = cntxt->devHandle;

    // Build a buffer: [regAddr][data...]
    uint8_t buf[len + 1];
    buf[0] = regAddr;
    memcpy(&buf[1], data, len);

    //Simply write to the i2c address given, starting with the register address
    esp_err_t ret = i2c_master_transmit(
        devHandle,
        buf,
        len + 1,
        100
    );
    return (ret == ESP_OK) ? 0 : -1;
}

//Simple delay function to pass to the BHI360 drivers.
void bhi360_delay_us(uint32_t period, void *intf_ptr) {
    vTaskDelay(pdMS_TO_TICKS((period + 999) / 1000));  // us → ms, ceil
}

