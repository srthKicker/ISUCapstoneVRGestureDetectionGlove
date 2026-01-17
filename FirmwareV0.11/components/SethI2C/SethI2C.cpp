#include <SethI2C.h>
#include <stdint.h>

extern "C" {
    #include "driver/i2c.h"
}

#define SDA_0 21
#define SCL_0 22
#define BEGIN_DELAY_TICKS 10
#define I2C_PORT I2C_NUM_0 //Used to determine which controller to r/w with
//const uint8_t accStart = 0x3B;
//const uint8_t gyroStart = 0x43;
//const float accSensitivityFactor = 16384.0f;
//const float gyroSensitivityFactor = 131.0f;

SethI2C::SethI2C(uint8_t mpuAddress){
    addr = mpuAddress;
}
//Admin/Initializing functions:
/*
    Powers on I2C
*/
void SethI2C::connect(){
    writeByte(powerReg, I2C_POWER_ON_VALUE); //Power on the I2C
}

//Write Functions
/*
    Operation works as follows:
    Create a i2c_cmd_handle_t 
        Effectively a queue of commands to send over I2C
    Add commands to queue
        Start signal
        I2C Device Address
        Register Number
        Data
        Stop signal
    Do i2c_master_cmd_begin to run all commands.
*/
bool SethI2C::writeByte(uint8_t regNumber, uint8_t byteToWrite){
    i2c_cmd_handle_t cmd = i2c_cmd_link_create(); //Create queue
    //Fill queue with things to send
    i2c_master_start(cmd); //Start communication signal
    i2c_master_write_byte(cmd, ((addr << 1) | I2C_MASTER_WRITE), true); //Send I2C device address
    i2c_master_write_byte(cmd, regNumber, true); //Send register to write to
    i2c_master_write_byte(cmd, byteToWrite, true);
    i2c_master_stop(cmd);
    //Send Queue
    esp_err_t outputCode = i2c_master_cmd_begin(I2C_PORT, cmd, BEGIN_DELAY_TICKS); //Change to num_1 for other i2c driver
    i2c_cmd_link_delete(cmd);
    return outputCode == ESP_OK; //return true if it succeeded
}
//Same as writeByte but it writes (surprise) 2 bytes
bool SethI2C::writeInt16(uint8_t regNumber, uint16_t intToWrite){
    i2c_cmd_handle_t cmd = i2c_cmd_link_create(); //Create queue
    //Fill queue with things to send
    i2c_master_start(cmd); //Start communication signal
    i2c_master_write_byte(cmd, ((addr << 1) | I2C_MASTER_WRITE), true); //Send I2C device address
    i2c_master_write_byte(cmd, regNumber, true); //Send register to write to
    //WOAH BIG DIFFERENCE (not really)
    i2c_master_write_byte(cmd, ((intToWrite & 0xFF00) >> 8), true); //High byte first (dont forget to shift)
    i2c_master_write_byte(cmd, (intToWrite && 0xFF), true); //LSB second
    i2c_master_stop(cmd);
    //Send Queue
    esp_err_t outputCode = i2c_master_cmd_begin(I2C_PORT, cmd, BEGIN_DELAY_TICKS); //Change to num_1 for other i2c driver
    i2c_cmd_link_delete(cmd);
    return outputCode == ESP_OK; //return true if it succeeded
}

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
*/
bool SethI2C::readByte(uint8_t regNumber, uint8_t* output){
    i2c_cmd_handle_t cmd = i2c_cmd_link_create();
    //Start queue
    //Write register address
    i2c_master_start(cmd);//start signal
    i2c_master_write_byte(cmd, ((addr << 1) | I2C_MASTER_WRITE), true); //I2C address
    i2c_master_write_byte(cmd, regNumber, true); //Register value to read from
    
    //Read value 
    i2c_master_start(cmd); //repeated start to trigger a new read or something idk
    i2c_master_write_byte(cmd, ((addr << 1) | I2C_MASTER_READ), true); //I2c address but for read
    i2c_master_read_byte(cmd, output, I2C_MASTER_NACK); //nack last bit but read byte to output
    i2c_master_stop(cmd); //stop signal
    esp_err_t errorCode = i2c_master_cmd_begin(I2C_PORT, cmd, BEGIN_DELAY_TICKS);
    i2c_cmd_link_delete(cmd);
    return errorCode == ESP_OK; //true if success, false if not
}       

/**
    Reads a bunch of bytes from the register that is specified
    This function is used by the BHI360 drivers so do not change the
    ordering of arguments unless driver requests a different structure.
*/
int8_t SethI2C::bhi360_i2c_read(uint8_t regAddr, uint8_t *data, uint32_t len, void *intf_ptr){
    i2c_cmd_handle_t cmd = i2c_cmd_link_create(); //Create queue of i2c commands
    esp_err_t returnValue;

    i2c_master_start(cmd); //start signal
    i2c_master_write_byte(cmd, ((addr << 1) | I2C_MASTER_WRITE), true);
    i2c_master_write_byte(cmd, regAddr,)

    return returnValue;
}
/*
    Burst read function that will fill the sensorData array with the 
    gyroscope and acceleromenter data. (and some other garbage)

    Order of queue:
        start signal
        i2c address with write bit
        register (to start reading at)
        start signal
        i2c address with read bit
        i2c_master_read 

*/

