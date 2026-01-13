#ifndef MADGWICK_H
#define MADGWICK_H
#include <Arduino.h>
#include "Quaternion.h"

/*
    Filter to interpret accelerometer/Gyroscope data to a Quaternion,
    normalize the data, and stabilize it.
*/
class Madgwick {
    public:

        //State variables: Will eventually contain one Quaternion for each finger

        //Will eventually have one ulong for last time polled for each finger
        //Thus, each finger will be able to track the time delta between each poll
        ulong iFLastPollTime;
        //eventually all will be implemented
        ulong mFLastPollTime;
        ulong rFLastPollTime;
        ulong pFLastPollTime;
        ulong thmLastPollTime;
        ulong rstLastPollTime;

        //Index Finger is the initial one we will work on 
        //before we implement more than one anyway
        Quaternion iFinger; 
        /* In future, these will be added
        Quaternion mFinger;
        Quaternion rFinger;
        Quaternion pFinger;
        Quaternion thum;
        Quaternion rist;
        */

        //Constructor (doesnt do much)
        Madgwick(); 
        

        /*
            Takes a new set of readings from a six axis and modifies the quaternion
            Will eventually have another arg to select which finger to modify
        */
        void SixAxisReadingsToQ(int16_t aReadings[3], int16_t gReadings[3]);

        /*
            The nine axis function will take readings from the wrist 6 axis and the wrist magnetometer
            and modify the rist Quaternion.
        */
        void NineAxisReadingsToQ(int16_t aReadings[3], int16_t gReadings[3], int16_t mReadings[3]);
    private:
        //Takes a raw gyro reading (currently mpu 6050 specific)
        //and turns it into a radians/s float
        float gyroReadToFloat(int16_t gyroReading);

        //Takes a raw accelerometer reading (currently mpu 6050 specific)
        //and converts it into a radians/s float
        float accReadToFloat(int16_t accReading);

};
#endif