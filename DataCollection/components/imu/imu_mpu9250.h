#pragma once
#include "imu_common.h"

void mpu9250_init(void);
void mpu9250_read(imu_sample_t *sample);
