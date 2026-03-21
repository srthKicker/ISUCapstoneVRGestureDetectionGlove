#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "esp_log.h"
#include "esp_timer.h"
#include "driver/i2c_master.h"
#include "bhi3.h"
#include "Bosch_Shuttle3_BHI360.fw.h"
#include "bhi360_i2c.h"
#include "stdint.h"
#include "math.h"

//These are the virtual sensor settings: (Page 103-104 of datasheet)
//Rotation vector REQUIRES magnetometer BMM150/350, but has accuracy field
//Game rotation vector does not have accuracy field (zeroed out but still sent) does not need magnetometer
//Add one to each value to get "wakeup" version, higher latency, bad for our project.
#define BHI360_VIRTUAL_SENSOR_ID 37 //Change to change virtual sensor value
#define BHI360_SENSORID_RV 34 //Rotation vector setting 
#define BHI360_SENSORID_GV 37 // Currently Using game vector in case no magnetometer is on board
#define QUAT_SCALING_FACTOR 16384.0f
//Pin numbers
#define SDA_0 8 //same as mosi with my wiring
#define SCL_0 7 //same as sck with my wiring
//I2C stuff
#define I2C_RATE_HZ 400000 //The clock frequency for i2c
#define I2C_TIMEOUT_US 2000 //timeout for clock stretching (if the device needs a bit longer it stretches the clock somehow)
#define CHANNEL 0 //testing I2C Mux channel number will add into context
#define USING_MUX true //Whether or not to control the mux in the i2c functions
// Firmware images
extern const uint8_t bhi360_firmware_image[]; 
//const uint32_t bhi360_firmware_size = 130312; //The size of the firmware currently
//debugging
const char *TAG = "Testing";
//extern const unsigned int bhi360_firmware_image_len = sizeof(bhi360_firmware_image); //Might not be needed?
//Addresses and Registers
const uint8_t SensorAddress = 0x28; //0x28 if sdo grounded for BHI360 or 0z29 if sdo set to 1.8v
const uint8_t MuxAddress = 0x70; // 0x70 is for MUX when all address pins are untouched
const float SensorSampleRate = 100.0f; //sample rate in HZ I think
const uint32_t SensorLatency = 0; //Something with buffering and stuff, ill explain later


//Static variables
static uint8_t fifo_buf[4096]; // Buffer for sensor data. ~0.5KB memory, shouldn't be too much at all. May expand if needed
static float quat[4];


//Debug function to visualize whats going on
//I just found this logic, i dont know how it works but it seems to lol
static void quatToEuler(const float *q) {
    float sinr_cosp = 2.0f * (q[0] * q[1] + q[2] * q[3]);
    float cosr_cosp = 1.0f - 2.0f * (q[1]*q[1] + q[2]*q[2]);
    float roll = atan2f(sinr_cosp, cosr_cosp);
    
    float sinp = 2.0f * (q[0] * q[2] - q[3] * q[1]);  // Fixed: direct sin_pitch
    float pitch;
    if (fabsf(sinp) >= 1.0f) {
        pitch = copysignf(M_PI / 2.0f, sinp);  // Gimbal lock handling
    } else {
        pitch = asinf(sinp);
    }
    
    float siny_cosp = 2.0f * (q[0] * q[3] + q[1] * q[2]);
    float cosy_cosp = 1.0f - 2.0f * (q[2]*q[2] + q[3]*q[3]);
    float yaw = atan2f(siny_cosp, cosy_cosp);
    
    ESP_LOGI("Euler", "roll=%.1f° pitch=%.1f° yaw=%.1f°", 
             roll * 180.0f / M_PI, pitch * 180.0f / M_PI, yaw * 180.0f / M_PI);
}


//Scales raw data to correct data amount.
static void rot_vec_cb(const struct bhy2_fifo_parse_data_info *info, void *priv) {
    if (info->sensor_id == BHI360_VIRTUAL_SENSOR_ID) { 
        int16_t *q_raw = (int16_t *)info->data_ptr;
        /*float q[4] = {
            q_raw[3] / 16384.0f,  //w   // Scale factor from defs
            q_raw[0] / 16384.0f, //x
            q_raw[1] / 16384.0f, //y
            q_raw[2] / 16384.0f //z
        };
        quat[0] = q[0];
        quat[1] = q[1];
        quat[2] = q[2];
        quat[3] = q[3]; */

        quat[0] = q_raw[3] / QUAT_SCALING_FACTOR;
        quat[1] = q_raw[0] / QUAT_SCALING_FACTOR;
        quat[2] = q_raw[1] / QUAT_SCALING_FACTOR;
        quat[3] = q_raw[2] / QUAT_SCALING_FACTOR;

    }
}






void app_main(void) {
    ESP_LOGI(TAG, "Start app_main, esplog ok");
    
    /**
     * Variables needed for function
     */
    //BHI360 driver variables
    struct bhy2_dev dev; // Device structure
    enum bhy2_intf intf = BHY2_I2C_INTERFACE; //Communication protocol
    //Context for i2c functions, include port MCU will use and address of device. Eventually will change throughout usage
    //to allow communicating with different i2c devices.

    //Configure i2c for the bus
    i2c_master_bus_config_t i2cBusConfig = {
        .clk_source = I2C_CLK_SRC_DEFAULT,
        .i2c_port = I2C_NUM_0,
        .scl_io_num = SCL_0,
        .sda_io_num = SDA_0,
        .glitch_ignore_cnt = 7,
        .flags.enable_internal_pullup = false,
    };

    //Configure the sensor
    i2c_device_config_t i2cSensorDeviceConfig = {
        .dev_addr_length = I2C_ADDR_BIT_LEN_7, //bhi360 uses 7 bit addresses
        
        .device_address = SensorAddress, 
        .scl_speed_hz = I2C_RATE_HZ, 
        .scl_wait_us = I2C_TIMEOUT_US,
        .flags.disable_ack_check = 0, //Enable NACK error detection.
    };

    //Configure the mux device
    i2c_device_config_t i2cMuxDeviceConfig = {
        .dev_addr_length = I2C_ADDR_BIT_LEN_7, //bhi360 uses 7 bit addresses
        
        .device_address = MuxAddress, 
        .scl_speed_hz = I2C_RATE_HZ, 
        .scl_wait_us = I2C_TIMEOUT_US,
        .flags.disable_ack_check = 0, //Enable NACK error detection.
    };
    i2c_master_bus_handle_t i2cBusHandle;
    i2c_master_dev_handle_t i2cSensorDevHandle; //To talk to the sensor
    i2c_master_dev_handle_t i2cMuxDevHandle; //To talk to the mux

    ESP_LOGI(TAG, "Declared structs!"); //debug
    //Link the bus config to the handle we've made
    ESP_ERROR_CHECK(i2c_new_master_bus(&i2cBusConfig, &i2cBusHandle));
    //Add the devices we've made here
    i2c_master_bus_add_device(i2cBusHandle,&i2cSensorDeviceConfig,&i2cSensorDevHandle);
    i2c_master_bus_add_device(i2cBusHandle,&i2cMuxDeviceConfig,&i2cMuxDevHandle);
    ESP_LOGI(TAG, "Added i2c device"); //debug
    //We pass this context to every i2c function through intf_ptr (pass by reference)
    i2cContext_t cntxt = {
        .busConfig = i2cBusConfig,
        .busHandle = i2cBusHandle,
        .devHandle = i2cSensorDevHandle,
        .devConfig = i2cSensorDeviceConfig,
        .muxDeviceConfig = i2cMuxDeviceConfig,
        .muxDevHandle = i2cMuxDevHandle,
        .muxChannel = CHANNEL, //This will change for each device we put in, for testing itll just be one
        .usingMUX = USING_MUX, //Optional for final product, will use mux if using i2c
    };
    //struct bhy2_virt_sensor_conf virtualSensorConf;

    //debug
    ESP_LOGI(TAG, "3. Testing I2C...");
    vTaskDelay(pdMS_TO_TICKS(200));
    ESP_LOGI(TAG, "NOW");
    vTaskDelay(pdMS_TO_TICKS(200));
    uint8_t chip_id[1];
    if (bhi360_i2c_read(0x01, chip_id, 1, &cntxt) == 0) {  // Read CHIP_ID reg
        ESP_LOGI(TAG, "I2C OK! CHIP_ID=0x%02x (expect 0xC8)", chip_id[0]);
    } else {
        ESP_LOGE(TAG, "I2C FAIL - No device @0x%02x", SensorAddress);
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

     #define NUM_RETRIES 5 //how many times we are willing to sit and wait for this to try to upload

     ESP_LOGI(TAG, "Uploading Firmware");
     ESP_LOGI(TAG, "Firmware size: %d bytes", sizeof(bhi360_firmware_image));
    bool uploadOk = false;
    for( int attempt = 0; attempt < NUM_RETRIES && !uploadOk; attempt ++){
        ESP_LOGI(TAG, "Attempt %d", attempt);
        int8_t uploadResult = bhy2_upload_firmware_to_ram(bhi360_firmware_image, sizeof(bhi360_firmware_image), &dev);
        
        if(uploadResult != BHY2_OK){ //if it fails
            ESP_LOGE("OOPS", "Try %d failed with uploadResult %d", attempt, uploadResult);
            vTaskDelay(pdMS_TO_TICKS(200)); //wait a minute before retrying
        } else{
                int8_t bootResult = bhy2_boot_from_ram(&dev);
                if(bootResult != BHY2_OK){ //on boot failure
                    ESP_LOGE("OOPS", "Try %d failed with bootResult %d", attempt, bootResult);
                    vTaskDelay(pdMS_TO_TICKS(200));
                } else{
                    uploadOk = true;
                }
            
        }
    }
    
    if(!uploadOk) {
        ESP_LOGE("OOPS", "scratch everything upload/boot failed after retries");
        return; //exit main
    }

    ESP_LOGI(TAG, "WHOOO WE UPLOADED");

     

    /* Old version, no retries
    ESP_LOGI(TAG, "Uploading firmware.."); //debug
    ESP_LOGI(TAG, "Firmware size: %d bytes", sizeof(bhi360_firmware_image));
    if (bhy2_upload_firmware_to_ram(bhi360_firmware_image, sizeof(bhi360_firmware_image), &dev) != BHY2_OK ||
        bhy2_boot_from_ram(&dev) != BHY2_OK) { //if either upload or boot fails print error
        ESP_LOGI(TAG, "Firmware load failed"); //debug
        return;
    }
    ESP_LOGI(TAG, "Firmware Uploadqed!"); //debug */
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
    //int count = 0;
    while (1) {
        int8_t err = bhy2_get_and_process_fifo(fifo_buf, sizeof(fifo_buf), &dev);//read from sensor
        if(err != BHY2_OK){
            ESP_LOGW("I2C Error", "FIFO err: %d", err);
        }
        vTaskDelay(pdMS_TO_TICKS(10));
        //ESP_LOGI("Quaternion", "w=%.3f i=%.3f j=%.3f k=%.3f", quat[0], quat[1], quat[2], quat[3]);
        quatToEuler(quat);
        //count++;

    }
}