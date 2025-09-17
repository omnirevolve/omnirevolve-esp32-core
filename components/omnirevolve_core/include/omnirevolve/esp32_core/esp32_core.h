#pragma once
#include <stdint.h>
#include "omnirevolve/protocol/cmd_ids.h"

#ifdef __cplusplus
extern "C" {
#endif

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
    volatile char color_name[16];
    volatile uint8_t current_cmd; // see CurrentCmd_t in core
} plotter_state_t;

typedef void (*ready_signal_isr_callback)(void *arg);

void plotter_init(void);
void plotter_init_sync(ready_signal_isr_callback cb);
void plotter_start_all_tasks(void);

uint32_t plotter_send_cmd(command_id_t cmd_id, const char* params);
void plotter_send_draw_stream_data(const uint8_t* data, uint32_t len);

void plotter_get_state(plotter_state_t *ps);

// Exposed by OLED driver to allow UI to read state
typedef void (*plotter_state_reader_t)(plotter_state_t *out);
void oled_set_state_reader(plotter_state_reader_t reader);

#ifdef __cplusplus
}
#endif
