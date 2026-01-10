This firmware is designed for data collection of user gestures. 


TO-DO:
    Set up MPU6050 version of software (I2C, DMP on the 6050)
        This includes:
        FreeRTOS scheduling (PLEASE MAKE NEW BRANCH FOR THAT)
        Quaternion Orientation
        Gesture Labeling
    Set up final BHI360 version of software (SPI, MPU in the 360)
        This includes:
        Replace I2C with SPI communication
        Change addresses to the BHI360 ones
        Remove i2cdev and mpu6050 library