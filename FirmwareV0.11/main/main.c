#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "esp_log.h"
#include "esp_timer.h"
#include "driver/spi_master.h"
#include "bhi3.h"
#include "Bosch_Shuttle3_BHI360.fw.h"
#include "bhi360_spi.h"
#include "stdint.h"
#include "math.h"


//These are the virtual sensor settings: (Page 103-104 of datasheet)
//Rotation vector REQUIRES magnetometer BMM150/350, but has accuracy field
//Game rotation vector does not have accuracy field (zeroed out but still sent) does not need magnetometer
//Add one to each value to get "wakeup" version, higher latency, bad for our project.
#define BHI360_VIRTUAL_SENSOR_ID 37 //Change to change virtual sensor value
#define BHI360_SENSORID_RV 34 //Rotation vector setting 
#define BHI360_SENSORID_GV 37 // Currently Using game vector in case no magnetometer is on board

//Pin numbers for SPI
#define SPI_MOSI 11
#define SPI_MISO 13
#define SPI_SCLK 12
#define SPI_CS   10

//SPI stuff
#define SPI_HOST    SPI2_HOST
#define SPI_CLK_HZ  (5 * 100 * 1000)  // 500kHz
#define SPI_MODE    0
#define SPI_BITS    8

// Firmware images
extern const uint8_t bhi360_firmware_image[]; 

//debugging
const char *TAG = "Testing";

//Addresses and Registers
const uint8_t SensorAddress = 0x28; //Default for BHI360
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
        float q[4] = {
            q_raw[0] / 16384.0f,  // Scale factor from defs
            q_raw[1] / 16384.0f,
            q_raw[2] / 16384.0f,
            q_raw[3] / 16384.0f
        };
        quat[0] = q[0];
        quat[1] = q[1];
        quat[2] = q[2];
        quat[3] = q[3];
    }
}

void app_main(void) {
    ESP_LOGI(TAG, "Start app_main, esplog ok");
    
    /**
     * Variables needed for function
     */
    //BHI360 driver variables
    struct bhy2_dev dev; // Device structure
    enum bhy2_intf intf = BHY2_SPI_INTERFACE; //Changed to SPI interface
    
    //SPI configuration
    spi_bus_config_t spiBusConfig = {
        .mosi_io_num = SPI_MOSI,
        .miso_io_num = SPI_MISO,
        .sclk_io_num = SPI_SCLK,
        .quadwp_io_num = -1,
        .quadhd_io_num = -1,
        .max_transfer_sz = 4096,
    };
    
    spi_device_interface_config_t spiDeviceConfig = {
        .command_bits = 8, // BHI360 spi drivers i made use 8 command bits
        .address_bits = 0, 
        .dummy_bits = 0,
        .mode = SPI_MODE,
        .duty_cycle_pos = 0,
        .cs_ena_pretrans = 0,
        .cs_ena_posttrans = 0,
        .clock_speed_hz = SPI_CLK_HZ,
        .spics_io_num = SPI_CS,
        .flags = 0,
        .queue_size = 7,
        .pre_cb = NULL,
        .post_cb = NULL,
    };
    
    spi_device_handle_t spiDeviceHandle;
    
    ESP_LOGI(TAG, "Declared structs!");
    
    // Initialize SPI bus
    ESP_ERROR_CHECK(spi_bus_initialize(SPI_HOST, &spiBusConfig, SPI_DMA_CH_AUTO));
    
    // Add SPI device
    ESP_ERROR_CHECK(spi_bus_add_device(SPI_HOST, &spiDeviceConfig, &spiDeviceHandle));
    ESP_LOGI(TAG, "Added SPI device");
    
    //SPI context for bhi360 functions
    spiContext_t cntxt = {
        .busConfig = spiBusConfig,
        .deviceConfig = spiDeviceConfig,
        .deviceHandle = spiDeviceHandle,
    };
    
    //debug - test SPI communication
    ESP_LOGI(TAG, "3. Testing SPI...");
    uint8_t chip_id[1];
    if (bhi360_spi_read(0x01, chip_id, 1, &cntxt) == 0) {  // Read CHIP_ID reg
        ESP_LOGI(TAG, "SPI OK! CHIP_ID=0x%02x (expect 0xC8)", chip_id[0]);
    } else {
        ESP_LOGE(TAG, "SPI FAIL - No device");
        while(1) vTaskDelay(pdMS_TO_TICKS(1000));  // Hang safe
    }
    
    /**
     * Initialize BHI360 interface
     * We pass the configuration struct to this as intf_ptr to give us context for everything
     */
    if (bhy2_init(intf, bhi360_spi_read, bhi360_spi_write,  
                    bhi360_delay_us, 256, &cntxt, &dev) != BHY2_OK) {
        printf("BHI360 init failed\n");
        return;
    }
    ESP_LOGI(TAG, "Initialized bhy2_init i guess");
    
    /**
     * Load firmware and boot from RAM
     */
    ESP_LOGI(TAG, "Uploading firmware..");
    if (bhy2_upload_firmware_to_ram(bhi360_firmware_image, sizeof(bhi360_firmware_image), &dev) != BHY2_OK ||
        bhy2_boot_from_ram(&dev) != BHY2_OK) {
        ESP_LOGI(TAG, "Firmware load failed");
        return;
    }
    ESP_LOGI(TAG, "Firmware Uploadqed!");
    
    /**
     * Update virtual sensor list & 
     * Declare the callback function to be called when FIFO is ready for a specific virtual sensor ID
     */
    bhy2_update_virtual_sensor_list(&dev);
    bhy2_register_fifo_parse_callback(BHI360_VIRTUAL_SENSOR_ID, rot_vec_cb, NULL, &dev);
    ESP_LOGI(TAG, "Updated virtual sensor and fifo parse callback");
    
    bhy2_set_virt_sensor_cfg(BHI360_VIRTUAL_SENSOR_ID, SensorSampleRate, SensorLatency, &dev);
    ESP_LOGI(TAG, "BHi360 ready, polling for rotation vector");
    
    while (1) {
        esp_err_t err = bhy2_get_and_process_fifo(fifo_buf, sizeof(fifo_buf), &dev);
        if(err != BHY2_OK){
            ESP_LOGW("SPI Error", "FIFO err: %d", err);
        }
        vTaskDelay(pdMS_TO_TICKS(1));
        quatToEuler(quat);
        vTaskDelay(pdMS_TO_TICKS(10));  // 10ms yield 100hz
    }
}
