#include "esp_timer.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "driver/i2c.h"
#include <cmath>
#include <stdio.h>
#include "MadgwickAHRS.h"
#include "SethMPU6050.h"


    /*
        Variable Declarations
    */
//Filter object                
Madgwick madg;
SethMPU6050 mpu;

//Timestamps
unsigned long lastTime;
unsigned long currentTime;

extern "C" void app_main(void) { //main function i guess
    
}

