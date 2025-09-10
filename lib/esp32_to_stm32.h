#ifndef ESP32_TO_STM32_H
#define ESP32_TO_STM32_H

#include <stdint.h>
#include "shared/cmd_ids.h"

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
    volatile uint16_t feedrate;
    volatile float speed_factor;
} plotter_state_t;

typedef void (*ready_signal_isr_callback)(void *arg);

void plotter_init(void);
uint32_t plotter_send_cmd(command_id_t cmd_id, const char* params);
void plotter_send_draw_stream_data(const uint8_t* data, uint32_t len);
void plotter_get_state(plotter_state_t *ps);
void plotter_start_all_tasks(void);
void plotter_init_sync(ready_signal_isr_callback cb);

#endif