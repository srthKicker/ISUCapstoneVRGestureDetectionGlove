#ifndef SethI2C_H
#define SethI2C_H
#include <stdint.h>


/*
    Stored virtual connection to an I2C (customizable) over I2C
    Uses driver/i2c.h to connect, stores ONE address/connection per object.
*/
class SethI2C {
    private:
        uint8_t addr;
        //First 6 bytes are acc, last 6 bytes are gyro


    public:
        SethI2C(uint8_t mpuAddress);
        void connect();
        //Input pointer location of where to store byte to read
        bool readByte(uint8_t registerNumber, uint8_t* output);
        /**
         * @brief Funct for Bosch drivers reading i2c data of a set length
         * @param[in] regAddr   : Reference to the data buffer where the FIFO data is copied to before parsing
         * @param[in] data  : Pointer to where we will store data
         * @param[in] intf_ptr: Something to do with status? Not sure yet...
         * @return I2C Errors
         */
        int8_t bhi360_i2c_read(uint8_t regAddr, uint8_t *data, uint32_t len, void *intf_ptr);
        //Write operations return true if successful, false if not
        bool writeByte(uint8_t registerNumber, uint8_t byteToWrite);
        bool writeInt16(uint8_t registerNumber, uint16_t intToWrite);


};

#endif //ends SethI2C_H