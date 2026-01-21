#include "imu_common.h"

static const char *TAG = "MPU9250";

void mpu9250_init(void) {
    ESP_LOGI(TAG, "MPU9250 initialized");
}

void mpu9250_read(imu_sample_t *out) {
    out->ax = 0;
    out->ay = 0;
    out->az = 0;
    out->gx = 0;
    out->gy = 0;
    out->gz = 0;
}