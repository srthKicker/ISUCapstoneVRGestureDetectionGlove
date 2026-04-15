#include "imu_mpu6050.h"

void mpu6050_init(void)
{
    // TODO: I2C init + config
}

void mpu6050_read(imu_sample_t *sample)
{
    sample->ax = 0;
    sample->ay = 0;
    sample->az = 0;
    sample->gx = 0;
    sample->gy = 0;
    sample->gz = 0;
    sample->mx = 0;
    sample->my = 0;
    sample->mz = 0;
    sample->timestamp = 0;
}
