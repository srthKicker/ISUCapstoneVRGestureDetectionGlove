extern "C" { //ESP drivers are in C

    #include "esp_timer.h"
    #include "freertos/FreeRTOS.h"
    #include "freertos/task.h"
    #include "driver/i2c.h"
    #include <cmath>
}
#include "MadgwickAHRS.h"
#include <stdio.h>

extern "C" void app_main(void) { //main function i guess
    

}

