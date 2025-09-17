    #include <stdio.h>
    #include "freertos/FreeRTOS.h"
    #include "freertos/task.h"
    #include "omnirevolve/esp32_core.h"
    #include "omnirevolve/core/state.h"
    #include "omnirevolve/link/stm_link.h"
    #include "omnirevolve/link/uart_proto.h"
    #include "omnirevolve/drivers/oled_display.h"
    #include "omnirevolve/drivers/keypad.h"

    static SemaphoreHandle_t s_init_sem = NULL;

    static void init_task(void *arg) {
        ready_signal_isr_callback cb = (ready_signal_isr_callback)arg;
        printf("Initializing hardware...
");
        or_stm_spi_init();
        or_oled_init();
        or_uart_init();
        or_stm_config_ready_irq(cb);
        if (s_init_sem) xSemaphoreGive(s_init_sem);
        vTaskDelete(NULL);
    }

    void or_plotter_init_sync(ready_signal_isr_callback cb) {
        if (!s_init_sem) s_init_sem = xSemaphoreCreateBinary();
        xTaskCreatePinnedToCore(init_task, "or_init", 4096, (void*)cb, 9, NULL, 1);
        xSemaphoreTake(s_init_sem, portMAX_DELAY);
    }

    void or_plotter_start_tasks(void) {
        xTaskCreatePinnedToCore(or_uart_task, "or_uart", 4096, NULL, 3, NULL, 1);
        xTaskCreatePinnedToCore(or_keypad_task, "or_keypad", 4096, NULL, 1, NULL, 1);
        xTaskCreatePinnedToCore(or_oled_task, "or_oled", 4096, NULL, 1, NULL, 1);
        // plotter control loop
        extern void or_plotter_control_task(void *pvParameters);
        xTaskCreatePinnedToCore(or_plotter_control_task, "or_ctrl", 4096, NULL, 2, NULL, 1);
    }

    void or_get_state(plotter_state_t *ps) { if (ps) *ps = g_or_plotter.state; }
