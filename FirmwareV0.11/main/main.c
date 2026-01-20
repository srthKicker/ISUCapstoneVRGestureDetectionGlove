#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "esp_log.h"
#include "driver/i2c.h"
#include "bhi3.h"
#include "Bosch_Shuttle3_BHI360.fw.h"
#include "bhi360_i2c.h"
#include "stdint.h"

//These are the virtual sensor settings: (Page 103-104 of datasheet)
//Rotation vector REQUIRES magnetometer BMM150/350, but has accuracy field
//Game rotation vector does not have accuracy field (zeroed out but still sent) does not need magnetometer
//Add one to each value to get "wakeup" version, higher latency, bad for our project.
#define BHI360_VIRTUAL_SENSOR_ID 37 //Change to change virtual sensor value
#define BHI360_SENSORID_RV 34 //Rotation vector setting 
#define BHI360_SENSORID_GV 37 // Currently Using game vector in case no magnetometer is on board
// Firmware images
extern const unsigned char bhi360_firmware_image[]; 
//extern const unsigned int bhi360_firmware_image_len = sizeof(bhi360_firmware_image); //Might not be needed?
//Addresses and Registers
const uint8_t SensorAddress = 0x28; //Default for BHI360, change for I2C multiplexer when that gets added

//Global variables

//Scales raw data to correct data amount.
static void rot_vec_cb(const struct bhi360_fifo_parse_data_info *info, void *priv) {
    if (info->sensor_id == BHI360_VIRTUAL_SENSOR_ID) { 
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
    // Buffer for sensor data. ~0.5KB memory, shouldn't be too much at all. May expand if needed.
    uint8_t fifo_buf[4096];

    //BHI360 driver variables
    struct bhi360_dev dev; // Device structure
    enum bhi360_intf intf = BHI360_I2C_INTERFACE;
    //Context for i2c functions, include port MCU will use and address of device. Eventually will change throughout usage
    //to allow communicating with different i2c devices.
    bhi360_cntxt_t cntxt = { .i2cPortNum = I2C_NUM_0, .i2cAddress = SensorAddress }; 

    //Configure I2C communication
    i2c_config_t i2cConf = {
    .mode = I2C_MODE_MASTER,
    .sda_io_num = 21,      // SDA_0
    .scl_io_num = 22,      // SCL_0
    .sda_pullup_en = GPIO_PULLUP_ENABLE,  // Use external pull-ups in final design
    .scl_pullup_en = GPIO_PULLUP_ENABLE,
    .master = {.clk_speed = 100000},  // 100khz
    .clk_flags = 0
    };
    ESP_ERROR_CHECK(i2c_param_config(I2C_NUM_0, &i2cConf));
    ESP_ERROR_CHECK(i2c_driver_install(I2C_NUM_0, i2cConf.mode, 0, 0, 0));

    // Initialize BHI360 interface
    if (bhi360_init(intf, bhi360_i2c_read, bhi360_i2c_write,  
                    bhi360_delay_us, 256, &cntxt, &dev) != BHI360_OK) {
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
    
    while (1) { //Superloop type function (previous is setup, this is superloop)
        bhi360_get_and_process_fifo(fifo_buf, sizeof(fifo_buf), &dev);
        vTaskDelay(pdMS_TO_TICKS(10));

    }
}
