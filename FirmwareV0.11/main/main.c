#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "esp_log.h"
#include "driver/i2c_master.h"
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
//Pin numbers
#define SDA_0 21
#define SCL_0 22
//I2C stuff
#define I2C_RATE_HZ 100000 //100khz
#define I2C_TIMEOUT_US 1000 //1ms timeout for clock
// Firmware images
extern const uint8_t bhi360_firmware_image[]; 
//debugging
const char *TAG = "Testing";
//extern const unsigned int bhi360_firmware_image_len = sizeof(bhi360_firmware_image); //Might not be needed?
//Addresses and Registers
const uint8_t SensorAddress = 0x28; //Default for BHI360, change for I2C multiplexer when that gets added
const float SensorSampleRate = 100.0f; //sample rate in HZ I think
const uint32_t SensorLatency = 0; //Something with buffering and stuff, ill explain later


//Global variables

//Scales raw data to correct data amount.
static void rot_vec_cb(const struct bhy2_fifo_parse_data_info *info, void *priv) {
    if (info->sensor_id == BHI360_VIRTUAL_SENSOR_ID) { 
        int16_t *q_raw = (int16_t *)info->data_ptr;
        float q[4] = {
            q_raw[0] / 16384.0f,  // Scale factor from defs
            q_raw[1] / 16384.0f,
            q_raw[2] / 16384.0f,
            q_raw[3] / 16384.0f
        };
        ESP_LOGI("Quaternion Vector:", "w=%.3f i=%.3f j=%.3f k=%.3f\n", q[0], q[1], q[2], q[3]);
    }
}

void app_main(void) {
    ESP_LOGI(TAG, "1. Start app_main, esplog ok");
    vTaskDelay(pdMS_TO_TICKS(1000));
    ESP_LOGI(TAG, "2. ESP_LOG now safe");
    ESP_LOGI(TAG, "Delaying to get reliable boot..."); //debug
    //Delay needed for boot sequence to show on serial
    vTaskDelay(pdMS_TO_TICKS(5000)); //5s delay
    ESP_LOGI(TAG, "Done Delaying!"); //debug
    
    /**
     * Variables needed for function
     */
    // Buffer for sensor data. ~0.5KB memory, shouldn't be too much at all. May expand if needed.
    uint8_t fifo_buf[4096];
    //BHI360 driver variables
    struct bhy2_dev dev; // Device structure
    enum bhy2_intf intf = BHY2_I2C_INTERFACE; //Communication protocol
    //Context for i2c functions, include port MCU will use and address of device. Eventually will change throughout usage
    //to allow communicating with different i2c devices.

    //Modern i2c configuration method
    i2c_master_bus_config_t i2cBusConfig = {
        .clk_source = I2C_CLK_SRC_DEFAULT,
        .i2c_port = I2C_NUM_0,
        .scl_io_num = SCL_0,
        .sda_io_num = SDA_0,
        .glitch_ignore_cnt = 7,
        .flags.enable_internal_pullup = true,
    };
    i2c_device_config_t i2cDeviceConfig = {
        .dev_addr_length = I2C_ADDR_BIT_LEN_7, //bhi360 uses 7 bit addresses
        
        .device_address = SensorAddress, //change in constants at top
        .scl_speed_hz = I2C_RATE_HZ, //100khz
        .scl_wait_us = I2C_TIMEOUT_US,
        .flags.disable_ack_check = 0, //Enable NACK error detection.
    };
    i2c_master_bus_handle_t i2cBusHandle;
    i2c_master_dev_handle_t i2cDevHandle;

    ESP_LOGI(TAG, "Declared structs!"); //debug
    ESP_ERROR_CHECK(i2c_new_master_bus(&i2cBusConfig, &i2cBusHandle));
    //Add the device we've made here
    i2c_master_bus_add_device(i2cBusHandle,&i2cDeviceConfig,&i2cDevHandle);
    ESP_LOGI(TAG, "Added i2c device"); //debug
    //We pass this context to every i2c function through intf_ptr (pass by reference)
    i2cContext_t cntxt = {
        .busConfig = i2cBusConfig,
        .busHandle = i2cBusHandle,
        .devHandle = i2cDevHandle,
        .devConfig = i2cDeviceConfig,
    };
    //struct bhy2_virt_sensor_conf virtualSensorConf;

    //debug
    ESP_LOGI(TAG, "3. Testing I2C...");
    uint8_t chip_id[1];
    if (bhi360_i2c_read(0x00, chip_id, 1, &cntxt) == 0) {  // Read CHIP_ID reg
        ESP_LOGI(TAG, "I2C OK! CHIP_ID=0x%02x (expect 0xC8)", chip_id[0]);
    } else {
        ESP_LOGE(TAG, "I2C FAIL - No device @0x28");
        while(1) vTaskDelay(pdMS_TO_TICKS(1000));  // Hang safe
    }
    /**
     * Initialize BHI360 interface
     * We pass the configuration struct to this as intf_ptr to give us context for everything
     * */
    if (bhy2_init(intf, bhi360_i2c_read, bhi360_i2c_write,  
                    bhi360_delay_us, 256, &cntxt, &dev) != BHY2_OK) {
        printf("BHI360 init failed\n");
        return;
    }
    ESP_LOGI(TAG, "Initialized bhy2_init i guess"); //debug
    /**
     * Load firmware and boot from RAM
     * */
    ESP_LOGI(TAG, "Uploading firmware.."); //debug
    if (bhy2_upload_firmware_to_ram(bhi360_firmware_image, sizeof(bhi360_firmware_image), &dev) != BHY2_OK ||
        bhy2_boot_from_ram(&dev) != BHY2_OK) { //if either upload or boot fails print error
        ESP_LOGI(TAG, "Firmware load failed"); //debug
        return;
    }
    ESP_LOGI(TAG, "Firmware Uploaded!"); //debug
    /**
     * Update virtual sensor list & 
     * Declare the callback function to be called when FIFO is ready for a specific virtual sensor ID
     * */
    bhy2_update_virtual_sensor_list(&dev);
    bhy2_register_fifo_parse_callback(BHI360_VIRTUAL_SENSOR_ID, rot_vec_cb, NULL, &dev);
    ESP_LOGI(TAG, "Updated virtual sensor and fifo parse callback"); //debug
    
    
    /**
     * Update virtual sensor list & 
     * Declare the callback function to be called when FIFO is ready for a specific virtual sensor ID
     * */
    bhy2_set_virt_sensor_cfg(BHI360_VIRTUAL_SENSOR_ID, SensorSampleRate, SensorLatency, &dev);
    ESP_LOGI(TAG, "BHi360 ready, polling for rotation vecotr"); //debug

    int count = 0;
    while (1) {
        bhy2_get_and_process_fifo(fifo_buf, sizeof(fifo_buf), &dev);
        
        count++;
        if (count % 10 == 0) {  // Print every 1s
            printf("Loop %d OK\n", count);
        }
        
        vTaskDelay(pdMS_TO_TICKS(100));  // 100ms yield (100Hz), FEED WDT [web:94]
    }
}
