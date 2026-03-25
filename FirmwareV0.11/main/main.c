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
#include "driver/gpio.h"

//These are the virtual sensor settings: (Page 103-104 of datasheet)
//Rotation vector REQUIRES magnetometer BMM150/350, but has accuracy field
//Game rotation vector does not have accuracy field (zeroed out but still sent) does not need magnetometer
//Add one to each value to get "wakeup" version, higher latency, bad for our project.

#define BHI360_SENSORID_RV 34 //Rotation vector setting 
#define BHI360_SENSORID_GV 37 // Currently Using game vector in case no magnetometer is on board
#define QUAT_SCALING_FACTOR 16384.0f

#define BHI360_VIRTUAL_SENSOR_ID BHI360_SENSORID_GV //Change to change virtual sensor value

#define NUMBER_OF_SENSORS 6 //this won't change lol, just removes magic numbers
//Pin numbers
#define SDA_PIN 7 //same as mosi with my wiring
#define SCL_PIN 8 //same as sck with my wiring
#define RESET_PIN 6
//I2C stuff
#define I2C_RATE_HZ 400000 //The clock frequency for i2c
#define I2C_TIMEOUT_US 2000 //timeout for clock stretching (if the device needs a bit longer it stretches the clock somehow)

/*
GUIDE TO CHANNELS ON PROTOTYPE
Channel 0 is nothing
Channel 1 is Wrist IMU
Channel 2 is nothing
Channel 3 is Thumb (left hand)
Channel 4 is Pointer (left  hand)
Channel 5 is Middle (left hand)
Channel 6 is Ring (left hand)
Channel 7 is Pointer (left hand)
*/
#define CHANNEL 6 //testing I2C Mux channel number will add into context
#define WRIST_CHANNEL 1
#define THUMB_CHANNEL 3
#define POINTER_CHANNEL 4
#define MIDDLE_CHANNEL 5
#define RING_CHANNEL 6
#define PINKY_CHANNEL 7

// Firmware images
extern const uint8_t bhi360_firmware_image[]; 
//const uint32_t bhi360_firmware_size = 130312; //The size of the firmware currently
//debugging
const char *TAG = "Testing";

//Addresses and Registers
const uint8_t SensorAddress = 0x28; //0x28 if sdo grounded for BHI360 or 0x29 if sdo set to 1.8v
const uint8_t MuxAddress = 0x70; // 0x70 is for MUX when all address pins are untouched
const float SensorSampleRate = 100.0f; //sample rate in HZ I think
const uint32_t SensorLatency = 0; //Something with buffering and stuff, ill explain later
//Helps determine which channel goes to which sensor
static const uint8_t MUX_CHANNEL_BY_SENSOR[NUMBER_OF_SENSORS] =
    {WRIST_CHANNEL, THUMB_CHANNEL, POINTER_CHANNEL, MIDDLE_CHANNEL, RING_CHANNEL, PINKY_CHANNEL}; 
const enum bhy2_intf intf = BHY2_I2C_INTERFACE;

/*
Static variables
*/
static gpio_config_t reset_pin_gpio_config; //Configured in setup
//i2c bus configurations and handles
static i2c_master_bus_config_t bus_config;
static i2c_master_bus_handle_t bus_handle;
//i2c device configurations and handles
static i2c_device_config_t mux_config;
static i2c_master_dev_handle_t mux_handle;
//For the bhi360 statics, we will order it like this: Wrist, Thumb, Pointer, Middle, Ring, Pinky
static i2c_device_config_t bhi360_configs[NUMBER_OF_SENSORS]; //Device configurations for each sensor
static i2c_master_dev_handle_t bhi360_handles[NUMBER_OF_SENSORS];
static i2cContext_t bhi360_contexts[NUMBER_OF_SENSORS]; //i2c context for each sensor
static struct bhy2_dev bhi360_devs[NUMBER_OF_SENSORS]; //The actual API devices
static uint8_t fifo_buf[NUMBER_OF_SENSORS][4096]; // Buffer for sensor data. ~0.5KB memory, shouldn't be too much at all. May expand if needed
static float quat[NUMBER_OF_SENSORS][4]; //Current quaternion we have read from the sensor

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
    i2cContext_t * cntxt = (i2cContext_t *)priv;
    //Ensure we are saving to the correct quaternion

    uint8_t sensorNumber;
    switch(cntxt->muxChannel){ //switch statement to determine which channel 
            case WRIST_CHANNEL:
                sensorNumber = 0;
                break;
            case THUMB_CHANNEL:
                sensorNumber = 1;
                break;
            case POINTER_CHANNEL:
                sensorNumber = 2;
                break;
            case MIDDLE_CHANNEL:
                sensorNumber = 3;
                break;
            case RING_CHANNEL:
                sensorNumber = 4;
                break;
            case PINKY_CHANNEL:
                sensorNumber = 5;
                break;
            default:
                return; //uhhh should never hit this spot
            break;
    }

    if (info->sensor_id == BHI360_VIRTUAL_SENSOR_ID) { 
        int16_t *q_raw = (int16_t *)info->data_ptr;
        quat[sensorNumber][0] = q_raw[3] / QUAT_SCALING_FACTOR;
        quat[sensorNumber][1] = q_raw[0] / QUAT_SCALING_FACTOR;
        quat[sensorNumber][2] = q_raw[1] / QUAT_SCALING_FACTOR;
        quat[sensorNumber][3] = q_raw[2] / QUAT_SCALING_FACTOR;
    }
}

//Resets all bhi360s at once
static void resetSensors(){
    gpio_set_level(RESET_PIN, 0); //Turn sensors off (active low reset pin)
    vTaskDelay(pdMS_TO_TICKS(100));
    gpio_set_level(RESET_PIN, 1); //Turn sensors back on
}
/*
SETUP FUNCTIONS

Below is a set of functions used to initialize communication with the BHI360 sensors
They are aimed to abstract as much as possible so that further development will not have to work with 
the API or any GPIO nonsense
*/

//Sets up the predetermined pin to reset all six bhi360 sensors at once if needed, pulls high to avoid reset
static void setupResetPin(){

    ESP_LOGI(TAG, "Setting up reset pins");
    //Configure reset pins
    reset_pin_gpio_config.pin_bit_mask = (1ULL << RESET_PIN);
    reset_pin_gpio_config.mode = GPIO_MODE_OUTPUT;
    reset_pin_gpio_config.pull_up_en = GPIO_PULLUP_DISABLE;
    reset_pin_gpio_config.pull_down_en = GPIO_PULLDOWN_DISABLE;
    reset_pin_gpio_config.intr_type = GPIO_INTR_DISABLE;
    gpio_config(&reset_pin_gpio_config);
    gpio_set_level(RESET_PIN, 1); //Pull reset high until we want to reset (active low = reset)
}

//Misleading name, sets up the i2c bus as well as all devices on it to allow for communication
static void setupI2CBus(){
    //Configure i2c for the bus
    bus_config.clk_source = I2C_CLK_SRC_DEFAULT;
    bus_config.i2c_port = I2C_NUM_0;
    bus_config.scl_io_num = SCL_PIN;
    bus_config.sda_io_num = SDA_PIN;
    bus_config.glitch_ignore_cnt = 7; //Just set it to typical value which is 7
    bus_config.flags.enable_internal_pullup = false; //we have stronger ones on the bus

    ESP_ERROR_CHECK(i2c_new_master_bus(&bus_config, &bus_handle));//Link the bus config to the handle we've made

    //Configure i2c for the multiplexer
    mux_config.dev_addr_length = I2C_ADDR_BIT_7;
    mux_config.device_address = MuxAddress;
    mux_config.scl_speed_hz = I2C_RATE_HZ;
    mux_config.scl_wait_us = I2C_TIMEOUT_US;
    mux_config.flags.disable_ack_check = 0; //Enable NACK error detection

    ESP_ERROR_CHECK(i2c_master_bus_add_device(bus_handle,&mux_config,&mux_handle)); //Add the mulitplexer to the bus

    //Configure i2c for the sensors (configs and contexts)
    // Ordered       Wrist, Thumb, Pointer, Middle, Ring, Pinky (0-5)
    // Channel number: 1       3      4       5       6     7 
    for(int sensorNum = 0; sensorNum < NUMBER_OF_SENSORS; sensorNum++){
        //Set up config
        bhi360_configs[sensorNum].dev_addr_length = I2C_ADDR_BIT_7;
        bhi360_configs[sensorNum].device_address = SensorAddress;
        bhi360_configs[sensorNum].scl_speed_hz = I2C_RATE_HZ;
        bhi360_configs[sensorNum].scl_wait_us = I2C_TIMEOUT_US;
        bhi360_configs[sensorNum].flags.disable_ack_check = 0;
        //Add sensor to bus
        ESP_ERROR_CHECK(i2c_master_bus_add_device(bus_handle, &bhi360_configs[sensorNum], &bhi360_handles[sensorNum]));

        //Set up context
        bhi360_contexts[sensorNum].devHandle = bhi360_handles[sensorNum];
        bhi360_contexts[sensorNum].muxDevHandle = mux_handle;
        bhi360_contexts[sensorNum].muxChannel = MUX_CHANNEL_BY_SENSOR[sensorNum]; //set the Mux channel number
    }

    ESP_LOGI(TAG, "Declared structs!"); //debug
    
}

//Initializes all BHI360 sensors to allow the API to communicate with them
static void setupBHI360Devices(){
    //We have to initialize each sensor so lets do this 6 times
    //It will take a while to upload one at a time
    for(int sensorNumber = 0; sensorNumber < NUMBER_OF_SENSORS; sensorNumber++){
        //Initialize BHI360 interface
        if(bhy2_init(intf, bhi360_i2c_read, bhi360_i2c_write, bhi360_delay_us, 256, &bhi360_contexts[sensorNumber], &bhi360_devs[sensorNumber]) != BHY2_OK) {
            ESP_LOGE(TAG, "Failed to initialize BHI360 #%d", sensorNumber);
        }
        //Upload firmware and boot from RAM
        #define NUM_RETRIES 3 //If it fails 3 times, we just give up and quit
        bool uploadOk = false; 
        for( int attempt = 0; attempt < NUM_RETRIES && !uploadOk; attempt ++){ //retry until 
        ESP_LOGI(TAG, "Attempt %d", attempt);
        int8_t uploadResult = bhy2_upload_firmware_to_ram(bhi360_firmware_image, sizeof(bhi360_firmware_image), &bhi360_devs[sensorNumber]);
        
        if(uploadResult != BHY2_OK){ //if it fails
            ESP_LOGE("OOPS", "Try %d failed with uploadResult %d", attempt, uploadResult);
            vTaskDelay(pdMS_TO_TICKS(200)); //wait a minute before retrying
        } else{ //if upload succeeds
                if(bhy2_boot_from_ram(&bhi360_devs[sensorNumber]) != BHY2_OK){ //on boot failure
                    ESP_LOGE("OOPS", "Try %d failed", attempt);
                    vTaskDelay(pdMS_TO_TICKS(200)); //wait a second to see if that helps
                } else{
                    uploadOk = true;
                }
            
        }
        } //end upload attempt loop
        if(!uploadOk) {
            ESP_LOGE("OOPS", "scratch everything upload/boot failed after retries");
            return; //exit main
        }
        ESP_LOGI(TAG, "WHOOO WE UPLOADED");
        
        /**
         * Update virtual sensor list & 
         * Declare the callback function to be called when FIFO is ready for a specific virtual sensor ID
         * */
        bhy2_update_virtual_sensor_list(&bhi360_devs[sensorNumber]);
        bhy2_register_fifo_parse_callback(BHI360_VIRTUAL_SENSOR_ID, rot_vec_cb, &bhi360_contexts[sensorNumber], &bhi360_devs[sensorNumber]);
        ESP_LOGI(TAG, "Updated virtual sensor and fifo parse callback"); //debug
        
        
        /**
         * Update virtual sensor list & 
         * Declare the callback function to be called when FIFO is ready for a specific virtual sensor ID
         * */
        bhy2_set_virt_sensor_cfg(BHI360_VIRTUAL_SENSOR_ID, SensorSampleRate, SensorLatency, &bhi360_devs[sensorNumber]);
        ESP_LOGI(TAG, "BHi360 #%d ready, polling for rotation vecotr", sensorNumber); //debug
        
    } //end sensor init loop
    ESP_LOGI(TAG, "All six sensors ready!");
}

//Called to initialize i2c, reset, multiplexer, and sensors
static void setupAll(){
    setupResetPin();
    setupI2CBus();
    setupBHI360Devices();
}

static void quaternionMultiplication(){
    
}

static void pollSensors(){
    for(int sensorNum = 0; sensorNum < NUMBER_OF_SENSORS; sensorNum++){
        int8_t err = bhy2_get_and_process_fifo(fifo_buf[sensorNum], sizeof(fifo_buf[sensorNum]), &bhi360_devs[sensorNum]);//read from sensor
        if(err != BHY2_OK){
            ESP_LOGE("I2C Error", "FIFO err: %d", err);
        }
    }
}
/********************************************************
 * This is Seth's code that polls the IMUs
 * to print their output as Euler vectors
 * converted to a FreeRTOS task function.
 * (Originally app_main)
 ********************************************************/
void pollSensors_Task(void *pvParameters) {
    setupAll();

    while(1){
        pollSensors();
        for(int sensorNum = 0; sensorNum < NUMBER_OF_SENSORS; sensorNum++){
            ESP_LOGI("Euler", "sensor %d", sensorNum);
            quatToEuler(quat[sensorNum]);
        }
        vTaskDelay(pdMS_TO_TICKS(10));
    }
}
/****************************************************
 * Basic Task structure without Core pinning.
 * Testing for Familiarity.
 ****************************************************/
void app_main(void) {
    xTaskCreate(
        pollSensors_Task,           //Task Function
        "Sensor Data Collection",   //Debugging Task Name
        4096,                       //Stack Size in Bytes
        NULL,                       //Task Parameters
        1,                          //Task Priority
        NULL                        //Task Handle (Optional)
    );
}