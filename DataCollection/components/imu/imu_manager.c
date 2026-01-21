#include "imu_common.h"

// Forward declarations
void mpu6050_init(int index);
void mpu6050_read(int index, imu_sample_t *out);

void mpu9250_init(void);
void mpu9250_read(imu_sample_t *out);

#define NUM_IMUS 3

void imu_init(void) {
    mpu9250_init();     // IMU 0
    mpu6050_init(1);    // IMU 1
    mpu6050_init(2);    // IMU 2
}

void imu_read_all(imu_sample_t *samples) {
    mpu9250_read(&samples[0]);
    mpu6050_read(1, &samples[1]);
    mpu6050_read(2, &samples[2]);
}
