#include <stdio.h>
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "omnirevolve/esp32_core.h"

static void IRAM_ATTR ready_isr(void *arg){
    // User ISR to signal READY pin from STM32; you can wake a sender task here if you add one.
    (void)arg;
}

void app_main(void){
    or_plotter_init_sync(ready_isr);
    or_plotter_start_tasks();
    for(;;){
        vTaskDelay(pdMS_TO_TICKS(1000));
    }
}
