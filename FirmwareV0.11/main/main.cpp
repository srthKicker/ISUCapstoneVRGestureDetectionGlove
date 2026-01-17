#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "driver/i2c.h"
#include "bhi360.h"
#include "Bosch_Shuttle3_BHI360.fw.h"
#include "bhi360_i2c.h"


//These are the virtual sensor settings: (Page 103-104 of datasheet)
//Rotation vector REQUIRES magnetometer BMM150/350, but has accuracy field
//Game rotation vector does not have accuracy field (zeroed out but still sent) does not need magnetometer
//Add one to each value to get "wakeup" version, higher latency, bad for our project.
#define BHI360_SENSORID_RV 34 //Rotation vector setting 
#define BHI360_SENSORID_GV 37 // Currently Using game vector in case no magnetometer is on board
// Firmware 
extern const unsigned char bhi360_firmware_image[]; 
extern const unsigned int bhi360_firmware_image_len;
//Addresses and Registers
const uint8_t SensorAddress = 0x28; //Default for BHI360, change for I2C multiplexer when that gets added


//Scales raw data to correct data amount.
static void rot_vec_cb(const struct bhi360_fifo_parse_data_info *info, void *priv) {
    if (info->sensor_id == BHI360_SENSORID_GV) {  // 34
        int16_t *q_raw = (int16_t *)info->data_ptr;
        float q[4] = {
            q_raw[0] / 16384.0f,  // Scale factor from defs
            q_raw[1] / 16384.0f,
            q_raw[2] / 16384.0f,
            q_raw[3] / 16384.0f
        };
        printf("Rotation Vector: w=%.3f i=%.3f j=%.3f k=%.3f\n", q[0], q[1], q[2], q[3]);
    }
}


void app_main(void) {
    struct bhi360_dev dev;
    uint8_t fifo_buf[4096];
    bhi360_intf bhi360InterfaceEnumerator = BHI360_I2C_INTERFACE;
        
    // Start by initializing I2C connection
    //
    // i2c_param_config(I2C_NUM_0, &i2c_config);
    // i2c_driver_install(I2C_NUM_0, ...);

    // Initialize 
    //Still have to implement 
    if (bhi360_init(bhi360InterfaceEnumerator, bhi360_i2c_read, bhi360_i2c_write,  
                    bhi360_delay_us, 256, NULL, &dev) != BHI360_OK) {
        printf("BHI360 init failed\n");
        return;
    }
    
    // Load firmware
    printf("Uploading firmware...\n");
    if (bhi360_upload_firmware_to_ram(bhi360_firmware_image, sizeof(bhi360_firmware_image), &dev) != BHI360_OK ||
        bhi360_boot_from_ram(&dev) != BHI360_OK) {
        printf("Firmware load failed\n");
        return;
    }
    
    bhi360_update_virtual_sensor_list(&dev);
    bhi360_register_fifo_parse_callback(BHI360_SENSORID_RV, rot_vec_cb, NULL, &dev);
    
    printf("BHI360 ready - polling for rotation vector...\n");
    
    while (1) {
        bhi360_get_and_process_fifo(fifo_buf, sizeof(fifo_buf), &dev);
        vTaskDelay(pdMS_TO_TICKS(10));
    }
}
