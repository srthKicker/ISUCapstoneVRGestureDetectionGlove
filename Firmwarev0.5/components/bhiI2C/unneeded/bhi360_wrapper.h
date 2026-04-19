#pragma once
#include <stdint.h>
#include <stdbool.h>

#ifdef __cplusplus
extern "C" {
#endif

#define BHI360_NUM_SENSORS 6

bool bhi360_wrapper_init(void);
bool bhi360_wrapper_poll_all(void);
bool bhi360_wrapper_get_quat(int sensor_idx, float out_quat[4]);

#ifdef __cplusplus
}
#endif