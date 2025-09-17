#include <string.h>
#include "freertos/FreeRTOS.h"
#include "omnirevolve/core/state.h"

plotter_manager_t g_or_plotter = {
    .state = {
        .is_connected = 0,
        .is_calibrated = 0,
        .is_homed = 0,
        .is_processing_cmd = 0,
        .is_idle = 0,
        .x_pos = -1,
        .y_pos = -1,
        .x_max = 0,
        .y_max = 0,
        .pen_is_down = 0,
        .current_color = 0,
        .current_cmd = CURRENT_CMD_IDLE
    },
    .request_counter = 1,
    .heartbeat_ok = 0,
    .last_heartbeat_sent_ticks = 0,
    .last_status_request_ticks = 0,
    .last_position_request_ticks = 0,
    .last_cmd_status_request_ticks = 0,
    .last_rx_any_ticks = 0,
    .last_rx_heartbeat_ticks = 0,
    .current_cmd_status = { .request_id = 0, .cmd_state = PLOTTER_CMD_STATE__UNDEFINED }
};
