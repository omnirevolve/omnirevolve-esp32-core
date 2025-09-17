#pragma once
#include <stdint.h>
#include "omnirevolve/protocol/cmd_ids.h"

#ifdef __cplusplus
extern "C" {
#endif

typedef enum {
    CURRENT_CMD_IDLE = 0,
    CURRENT_CMD_CALIBRATING,
    CURRENT_CMD_HOMING,
    CURRENT_CMD_DRAWING,
    CURRENT_CMD_ERROR
} CurrentCmd_t;

typedef struct {
    volatile uint8_t is_connected;
    volatile uint8_t is_calibrated;
    volatile uint8_t is_homed;
    volatile uint8_t is_processing_cmd;
    volatile uint8_t is_idle;

    volatile int32_t x_pos;
    volatile int32_t y_pos;

    volatile uint32_t bytes_processed;

    volatile float x_max;
    volatile float y_max;

    volatile uint8_t pen_is_down;
    volatile uint8_t current_color;
    volatile char    color_name[16];

    volatile uint8_t current_cmd; // CurrentCmd_t
} plotter_state_t;

typedef struct {
    uint32_t request_id;
    CmdState_t cmd_state;
} CurrentCommand_t;

typedef struct {
    volatile plotter_state_t state;

    uint32_t request_counter;

    uint8_t  heartbeat_ok;
    char     status_text[64];
    char     last_command[64];
    char     last_response[64];

    // Timestamps
    uint32_t last_heartbeat_sent_ticks;
    uint32_t last_status_request_ticks;
    uint32_t last_position_request_ticks;
    uint32_t last_cmd_status_request_ticks;
    uint32_t last_rx_any_ticks;
    uint32_t last_rx_heartbeat_ticks;

    CurrentCommand_t current_cmd_status;
} plotter_manager_t;

extern plotter_manager_t g_or_plotter;

// Accessors
static inline plotter_manager_t* or_state(void) { return &g_or_plotter; }

#ifdef __cplusplus
}
#endif
