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
        Quaternion iFinger; //index Finger
        Madgwick(); //Constructor
        


        //Takes a new set of readings from a six axis and modifies the quaternion
        //Will eventually have another arg to select which finger to modify
        void SixAxisReadingsToQ(int16_t aReadings[3], int16_t gReadings[3]);
    private:
        

};
#endif