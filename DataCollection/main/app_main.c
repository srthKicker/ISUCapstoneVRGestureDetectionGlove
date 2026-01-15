#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "imu_common.h"

void imu_task(void *arg) {
    imu_sample_t samples[3];

    while (1) {
        imu_read_all(samples);

        // TODO: stream / store
        vTaskDelay(pdMS_TO_TICKS(5)); // 200Hz
    }
}

void app_main(void) {
    imu_init();
    xTaskCreate(imu_task, "imu_task", 4096, NULL, 5, NULL);
}
