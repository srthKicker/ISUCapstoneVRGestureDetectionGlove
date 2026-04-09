#pragma once
#include "imu_common.h"

void mpu6050_init(void);
void mpu6050_read(imu_sample_t *sample);
