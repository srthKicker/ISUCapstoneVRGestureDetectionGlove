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
    i2c_master_dev_handle_t i2c_dev = (i2c_master_dev_handle_t)intf_ptr;

    // Perform a write-then-read transaction
    esp_err_t ret = i2c_master_transmit_receive(
        i2c_dev,
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
    i2c_master_dev_handle_t i2c_dev = (i2c_master_dev_handle_t)intf_ptr;

    // Build a buffer: [regAddr][data...]
    uint8_t buf[len + 1];
    buf[0] = regAddr;
    memcpy(&buf[1], data, len);

    esp_err_t ret = i2c_master_transmit(
        i2c_dev,
        buf,
        len + 1,
        100
    );
    return (ret == ESP_OK) ? 0 : -1;
}


/**
    Reads a bunch of bytes from the register that is specified
    This function is used by the BHI360 drivers so do not change the
    ordering of arguments unless driver requests a different structure.
*/
/*int8_t bhi360_read(uint8_t regAddr, uint8_t *data, uint32_t len, void *intf_ptr){
    i2c_cmd_handle_t cmd = i2c_cmd_link_create(); //Create queue of i2c commands
    esp_err_t returnValue;
    bhi360_cntxt_t * cntxt = (bhi360_cntxt_t*)intf_ptr;
    //Tell it the register and address we wanna talk to
    i2c_master_start(cmd); //start signal
    i2c_master_write_byte(cmd, (((cntxt->i2cAddress) << 1) | I2C_MASTER_WRITE), true); //i wanna WRITE
    i2c_master_write_byte(cmd, regAddr,true);
    //Tell it how many bytes we want to read
    i2c_master_start(cmd); //repeated start signal
    i2c_master_write_byte(cmd, (((cntxt->i2cAddress) << 1) | I2C_MASTER_READ), true); // i wanna READ
    
    //Protocol says that ESP32 ACK all bytes read except last byte which it NACKs, this does that.
    if(len > 1) { //make sure its not asking to read 0 bytes
        i2c_master_read(cmd, data, len-1, I2C_MASTER_ACK); //ack all but last byte
        i2c_master_read_byte(cmd, (data+len-1), I2C_MASTER_NACK); //nack it!
    } else if (len == 1) { //
        i2c_master_read_byte(cmd, data, I2C_MASTER_NACK);
    }
    i2c_master_stop(cmd); //stop signal

    //Actually send the queueue we made
    returnValue = i2c_master_cmd_begin((cntxt->i2cPortNum), cmd, pdMS_TO_TICKS(portTICK_PERIOD_MS));
    i2c_cmd_link_delete(cmd);
    return (returnValue == ESP_OK) ? 0 : -1; //API expects 0 for success, negative values for failure?
}*/
/** 
    Writes bites from the data 
    Refer to comments on old simple functions for explaination of protocol

*/
/*int8_t bhi360_i2c_write(uint8_t regAddr, const uint8_t *data, uint32_t len, void *intf_ptr){
    bhi360_cntxt_t * cntxt = (bhi360_cntxt_t*)intf_ptr; //Casts to our context struct to get data from it


    i2c_cmd_handle_t cmd = i2c_cmd_link_create();//Queue create
    i2c_master_start(cmd);
    i2c_master_write_byte(cmd, (cntxt->i2cAddress << 1) | I2C_MASTER_WRITE, true);
    i2c_master_write_byte(cmd, regAddr, true);
    if(len > 0) {
        i2c_master_write(cmd, data, len, true);
    }
    i2c_master_stop(cmd);
    int8_t returnValue = i2c_master_cmd_begin((cntxt->i2cPortNum), cmd,  pdMS_TO_TICKS(portTICK_PERIOD_MS));
    i2c_cmd_link_delete(cmd);
    return (returnValue == ESP_OK) ? 0 : -1; //API expects 0 for success, negative values for failure?
}

//Simple delay function to pass to the BHI360 drivers.
void bhi360_delay_us(uint32_t period, void *intf_ptr) {
    ets_delay_us(period);
}*/

/**
 * Old simple functions, they would be good for something that doesnt do FIFO like the
 * BHI360 does Good for reference on how to I2C in ESPIDF using driver/i2c.h
 * Too bad we aren't using it anymore
 */

//Write Functions
/**
    Operation works as follows:
    Create a i2c_cmd_handle_t 
        Effectively a queue of commands to send over I2C
    Add commands to queue
        Start signal
        I2C Device Address
        Register Number
        Data
        Stop signal
    Do i2c_master_cmd_begin to run all commands. (kinda old, not used)

    OBSOLETE. Only for reference
*/
/**bool writeByte(uint8_t regNumber, uint8_t byteToWrite){
    return false; //This function only for reference
    i2c_cmd_handle_t cmd = i2c_cmd_link_create(); //Create queue
    //Fill queue with things to send
    i2c_master_start(cmd); //Start communication signal
    i2c_master_write_byte(cmd, ((0x28 << 1) | I2C_MASTER_WRITE), true); //Send I2C device address
    i2c_master_write_byte(cmd, regNumber, true); //Send register to write to
    i2c_master_write_byte(cmd, byteToWrite, true);
    i2c_master_stop(cmd);
    //Send Queue
    esp_err_t outputCode = i2c_master_cmd_begin(I2C_NUM_0, cmd, 10); //Change to num_1 for other i2c driver
    i2c_cmd_link_delete(cmd);
   // return outputCode == ESP_OK; //return true if it succeeded
   return false; //This function only for reference
}*/

//Read functions
/*
    Operation works as follows:
    Create an i2c_cmd_handle_t (same thing as last time)
    Add commands to the queue
        Start signal
        Write device address (with write signal I2C_MASTER_WRITE)
        Write register number
        Start signal
        Write device address (with read signal! I2C_MASTER_READ)
        i2c_master_read_byte(cmd, out, I2C_MASTER_NACK)
            NACK the last byte to be read
            cmd is the handle
            out is a POINTER where the data read will be stored
        Stop signal

        OBSOLETE. ONLY FOR REFERENCE
*/
/**bool readByte(uint8_t regNumber, uint8_t* output){
    i2c_cmd_handle_t cmd = i2c_cmd_link_create();
    //Start queue
    //Write register address
    i2c_master_start(cmd);//start signal
    i2c_master_write_byte(cmd, ((0x28 << 1) | I2C_MASTER_WRITE), true); //I2C address
    i2c_master_write_byte(cmd, regNumber, true); //Register value to read from
    
    //Read value 
    i2c_master_start(cmd); //repeated start to trigger a new read or something idk
    i2c_master_write_byte(cmd, ((0x28 << 1) | I2C_MASTER_READ), true); //I2c address but for read
    i2c_master_read_byte(cmd, output, I2C_MASTER_NACK); //nack last bit but read byte to output
    i2c_master_stop(cmd); //stop signal
    esp_err_t errorCode = i2c_master_cmd_begin(I2C_NUM_0, cmd, 10);
    i2c_cmd_link_delete(cmd);
    return errorCode == ESP_OK; //true if success, false if not
}       */



