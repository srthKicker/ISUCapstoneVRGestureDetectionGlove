#include "Madgwick.h"
#include "Quaternion.h"

//Stores the constants before runtime to avoid unecessary calculations
const float pi = 3.1415927f; //It's just pi what do you want, it'll be good for 32 bit floats

const float gyroScale = 131.0f; //Changes based on precision
//To avoid division, multiply raw gyro data by this to convert to rad/s
const float invGyroScale = (1.0f/gyroScale)*(pi/180.0f); //potentially Div by 2 to speed up.
const float accScale = 16384.0f;//changes based on precision
//Mult raw accelerometer data by this to get in terms of 'gs'
const float invAccScale = 1.0f/accScale;


//Constructor
Madgwick::Madgwick() {
    //Set the last polled time for all the fingers (just  using index finger for now)
    iFLastPollTime = micros();
    mFLastPollTime = micros();
    rFLastPollTime = micros();
    pFLastPollTime = micros();
    thmLastPollTime = micros();
    rstLastPollTime = micros();
}


/*
    Convert raw gyro readings to radians/second

    Current estimated precision range I believe is +-250.
    To change this, go to the top and change the scale (NOT THE INVERSE) constants
    MPU-6050 Configuration Bytes By Default
    GYRO Config at 0x1B
    Bits [4:3]	Full-Scale Range	Sensitivity (LSB/°/s)
    00	        ±250 °/s	        131.0	
    01	        ±500 °/s	        65.5	
    10	        ±1000 °/s	        32.8	
    11	        ±2000 °/s	        16.4	
*/
float Madgwick::gyroReadToFloat(int16_t gyroReading) {
    return invGyroScale * gyroReading; //multiply to avoid division on microcontroller
}

/*
    Convert raw accelerometer readings to gs.

    Currently scaled at +/- 2G so 16,384 per g of acceleration (will change)
    To change this, go to the top and change the scale (NOT THE INVERSE) constants
    ACC Config at 0x1C
    Bits [4:3]	Precision	Scale (LSB/g)
    00	        ±2 g	    16384	    
    01	        ±4 g	    8192	    
    10	        ±8 g	    4096	    
    11	        ±16 g	
*/
float Madgwick::accReadToFloat(int16_t accReading) {
    return invAccScale * accReading; //multiply to avoid division on microcontroller
}

/*
    Takes new set of readings and changes quaternion
    Will eventually allow input to choose which finger to change
    Takes
*/
void Madgwick::SixAxisReadingsToQ(int16_t aReadings[3], int16_t gReadings[3]) {
    Quaternion qDot = iFinger; //Intermediate value
    //This quaternion is the one from the angular acceleration from gyroscope
    Quaternion omegaQuat = Quaternion(
        0.0f, 
        gyroReadToFloat(gReadings[0]), 
        gyroReadToFloat(gReadings[1]), 
        gyroReadToFloat(gReadings[2])
    );
    omegaQuat.qScale(0.5f);
    qDot.qMultiply(omegaQuat); //Multiply the two 
    float timeDelta = (micros() - iFLastPollTime) * 1e-6f; //1e-6f is 10^-6 in float. Converts from us to s
    qDot.qScale(timeDelta);
    iFinger.qAdd(qDot); //Adds the change in rotation to the 
    iFinger.normalize(); //Make sure we end with a normalized quaternion.
}//STILL NEEDS ACCELEROMETER STUFF