#include <stdio.h>
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "omnirevolve/core/state.h"
#include "omnirevolve/protocol/cmd_ids.h"
#include "omnirevolve/link/uart_proto.h"

#define HEARTBEAT_PERIOD_MS 1000
#define CONNECT_TIMEOUT_MS  3000

void or_plotter_control_task(void *pvParameters) {
    (void)pvParameters;
    printf("Plotter control task started\n");
    vTaskDelay(pdMS_TO_TICKS(500));

    or_send_cmd(CMD_HEARTBEAT, NULL);
    g_or_plotter.last_heartbeat_sent_ticks = xTaskGetTickCount();

    for(;;) {
        TickType_t now = xTaskGetTickCount();
        if ((now - g_or_plotter.last_heartbeat_sent_ticks) >= pdMS_TO_TICKS(HEARTBEAT_PERIOD_MS)) {
            or_send_cmd(CMD_HEARTBEAT, NULL);
            g_or_plotter.last_heartbeat_sent_ticks = now;
        }
        if ((now - g_or_plotter.last_status_request_ticks) >= pdMS_TO_TICKS(1000)) {
            or_send_cmd(CMD_GET_STATUS, NULL);
            g_or_plotter.last_status_request_ticks = now;
        }
        if ((now - g_or_plotter.last_position_request_ticks) >= pdMS_TO_TICKS(500)) {
            or_send_cmd(CMD_GET_POSITION, NULL);
            g_or_plotter.last_position_request_ticks = now;
        }
        if ((now - g_or_plotter.last_cmd_status_request_ticks) >= pdMS_TO_TICKS(500)) {
            g_or_plotter.last_cmd_status_request_ticks = now;
            CmdState_t cs = g_or_plotter.current_cmd_status.cmd_state;
            bool active = (cs == PLOTTER_CMD_STATE__RECEIVED || cs == PLOTTER_CMD_STATE__EXECUTING);
            if (active) {
                char buf[16];
                snprintf(buf, sizeof(buf), "%u", (unsigned)g_or_plotter.current_cmd_status.request_id);
                or_send_cmd(CMD_GET_CMD_STATUS, buf);
            }
        }

        TickType_t last_alive = (g_or_plotter.last_rx_heartbeat_ticks > g_or_plotter.last_rx_any_ticks)
                                ? g_or_plotter.last_rx_heartbeat_ticks
                                : g_or_plotter.last_rx_any_ticks;
        uint8_t prev = g_or_plotter.state.is_connected;
        g_or_plotter.state.is_connected = (last_alive != 0) &&
            ((now - last_alive) <= pdMS_TO_TICKS(CONNECT_TIMEOUT_MS));
        if (prev != g_or_plotter.state.is_connected) {
            printf("[link] is_connected = %s\n", g_or_plotter.state.is_connected ? "true" : "false");
        }
        vTaskDelay(pdMS_TO_TICKS(50));
    }
}
