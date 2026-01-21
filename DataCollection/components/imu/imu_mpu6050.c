#include "imu_common.h"

static const char *TAG = "MPU6050";

void mpu6050_init(int index) {
    ESP_LOGI(TAG, "MPU6050 %d initialized", index);
}

void mpu6050_read(int index, imu_sample_t *out) {
    out->ax = 0;
    out->ay = 0;
    out->az = 0;
    out->gx = 0;
    out->gy = 0;
    out->gz = 0;
}