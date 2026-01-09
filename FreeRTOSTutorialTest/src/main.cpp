#include <stdio.h>
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "esp_log.h"
#include "esp_timer.h"

static const char *TAG = "DUAL_CORE_TEST";

// A computationally intensive dummy task
void intensive_task(void *pvParameters)
{
    int core_id = xPortGetCoreID();
    uint32_t task_num = (uint32_t)pvParameters;
    volatile uint64_t counter = 0;

    ESP_LOGI(TAG, "Task %lu started on Core %d", task_num, core_id);

    for (int i = 0; i < 200000000; i++) {
        counter++;
    }

    ESP_LOGI(TAG, "Task %lu finished on Core %d", task_num, core_id);
    vTaskDelete(NULL);
}

void app_main(void)
{
    ESP_LOGI(TAG, "Starting dual-core performance test.");

    // Get the start time
    int64_t start_time = esp_timer_get_time();

    // Create and pin Task 1 to Core 0
    xTaskCreatePinnedToCore(
        intensive_task,   // Task function
        "IntensiveTask1", // Task name
        4096,             // Stack size
        (void *)1,        // Task parameter (task number)
        5,                // Priority
        NULL,             // Task handle
        0                 // Core ID (PRO_CPU)
    );

    // Create and pin Task 2 to Core 1
    xTaskCreatePinnedToCore(
        intensive_task,   // Task function
        "IntensiveTask2", // Task name
        4096,             // Stack size
        (void *)2,        // Task parameter (task number)
        5,                // Priority
        NULL,             // Task handle
        1                 // Core ID (APP_CPU)
    );

    // Note: In a real app, we'd wait for tasks to finish.
    // Here, we just log the start and observe the finish logs.
    // We add a delay to let the tasks run.
    vTaskDelay(pdMS_TO_TICKS(10000));

    // The finish logs from the tasks will show they run in parallel.
    // Try changing both tasks to run on the same core and observe the time difference.
}
