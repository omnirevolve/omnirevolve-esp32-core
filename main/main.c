#include <stdio.h>
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "omnirevolve/esp32_core/esp32_core.h"
#include "omnirevolve/protocol/cmd_ids.h"

static void state_reader(plotter_state_t *out) {
    plotter_get_state(out);
}

void app_main(void) {
    // Basic demo for standalone build.
    plotter_init_sync(NULL);
    oled_set_state_reader(state_reader);
    plotter_start_all_tasks();

    // Optional: send a heartbeat and query status on boot.
    plotter_send_cmd(CMD_HEARTBEAT, NULL);
    plotter_send_cmd(CMD_GET_STATUS, NULL);

    // Idle loop.
    while (1) {
        vTaskDelay(pdMS_TO_TICKS(1000));
    }
}
