#ifndef SethMPU6050_H
#define SethMPU6050_H
#include <stdint.h>


/*
    Stored virtual connection to an MPU6050 (customizable) over I2C
    Uses driver/i2c.h to connect, stores ONE address/connection per object.
*/
class SethMPU6050 {
    private:
        uint8_t addr;
        //First 6 bytes are acc, last 6 bytes are gyro
        uint8_t sensorData[14]; //Array of sensor data from acc-gyro


    public:
        SethMPU6050(uint8_t mpuAddress);
        void connect();
        //Input pointer location of where to store byte to read
        bool readByte(uint8_t registerNumber, uint8_t* output);
        /*
        Places sensor data into sensorData array for use, does not convert it into usable units
        That feature will be implemented using the bhi360's integrated filter, this is to learn
        how to write drivers. (plus my embedded system project is going to use this) -Seth
        */ 
        bool readSensorData();
        //Write operations return true if successful, false if not
        bool writeByte(uint8_t registerNumber, uint8_t byteToWrite);
        bool writeInt16(uint8_t registerNumber, uint16_t intToWrite);


};

#endif //ends SethMPU6050_H