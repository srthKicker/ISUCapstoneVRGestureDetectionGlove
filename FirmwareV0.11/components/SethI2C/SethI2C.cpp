#include <SethMPU6050.h>
#include <driver/i2c.h>
#include <stdint.h>

#define SDA_0 21
#define SCL_0 22
#define BEGIN_DELAY_TICKS 10
#define I2C_PORT I2C_NUM_0 //Used to determine which controller to r/w with
#define SENSOR_DATA_SIZE_BYTES 14
const uint8_t powerReg = 0x6B;
const uint8_t MPU6050_POWER_ON_VALUE = 0x00;
const uint8_t accStart = 0x3B;
const uint8_t gyroStart = 0x43;
const float accSensitivityFactor = 16384.0f;
const float gyroSensitivityFactor = 131.0f;
const float madgRate = 500.0f; //Hz rate of polling

SethMPU6050::SethMPU6050(uint8_t mpuAddress){
    addr = mpuAddress;
}
//Admin/Initializing functions:
/*
    Powers on MPU6050
*/
void SethMPU6050::connect(){
    writeByte(powerReg, MPU6050_POWER_ON_VALUE); //Power on the MPU6050
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
bool SethMPU6050::writeByte(uint8_t regNumber, uint8_t byteToWrite){
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
bool SethMPU6050::writeInt16(uint8_t regNumber, uint16_t intToWrite){
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
bool SethMPU6050::readByte(uint8_t regNumber, uint8_t* output){
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
bool SethMPU6050::readSensorData(){
    //Write to sensorData[14], use addr for i2c address, use accStart as starting register

}
