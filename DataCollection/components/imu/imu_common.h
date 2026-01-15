#pragma once

typedef struct {
    float ax, ay, az;
    float gx, gy, gz;
    float mx, my, mz;
    uint32_t timestamp;
} imu_sample_t;

void imu_init(void);
void imu_read_all(imu_sample_t *samples);
