#ifndef ESP32_TO_STM32_H
#define ESP32_TO_STM32_H

#include <stdint.h>
#include "../main/shared/cmd_ids.h"

typedef struct {
    volatile uint8_t is_connected;

    volatile uint8_t is_calibrated;
    volatile uint8_t is_homed;
    volatile uint8_t is_processing_cmd;
    volatile uint8_t is_idle;
    
    volatile int32_t x_pos;
    volatile int32_t y_pos;
    volatile float x_max;
    volatile float y_max;
    
    volatile uint8_t pen_is_down;
    volatile uint8_t current_color;
    volatile char color_name[16];
    volatile uint16_t feedrate;
    volatile float speed_factor;
} plotter_state_t;

void plotter_init(void);
uint32_t plotter_send_cmd(command_id_t cmd_id, const char* params);
uint8_t plotter_is_ready_to_receive_draw_stream_data(void);
void plotter_send_draw_stream_data(const uint8_t* data, uint32_t size);
void plotter_get_state(plotter_state_t *ps);

#endif