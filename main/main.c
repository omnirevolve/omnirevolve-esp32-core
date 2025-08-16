#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include <stdio.h>
#include "esp32_to_stm32.h"

void app_main() {
    printf("ESP32 Plotter ESP32 Controller starting...\n");
    
    plotter_init();
    plotter_start_all_tasks();

    while (1) {
        vTaskDelay(pdMS_TO_TICKS(100));
    }
}