#include <stdio.h>
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "driver/gpio.h"
#include "omnirevolve/hal/pins.h"
#include "omnirevolve/link/uart_proto.h"
#include "omnirevolve/protocol/cmd_ids.h"

static const char keys[KEYPAD_ROWS][KEYPAD_COLS] = {
    {'1','2','3'},
    {'4','5','6'},
    {'7','8','9'},
};

static void keypad_init(void) {
    for (int i=0;i<KEYPAD_ROWS;i++) {
        gpio_set_direction(or_keypad_row_pins[i], GPIO_MODE_OUTPUT);
        gpio_set_level(or_keypad_row_pins[i], 1);
    }
    for (int i=0;i<KEYPAD_COLS;i++) {
        gpio_set_direction(or_keypad_col_pins[i], GPIO_MODE_INPUT);
        gpio_set_pull_mode(or_keypad_col_pins[i], GPIO_PULLUP_ONLY);
    }
}

static char keypad_scan(void) {
    for (int r=0;r<KEYPAD_ROWS;r++) {
        gpio_set_level(or_keypad_row_pins[r], 0);
        vTaskDelay(pdMS_TO_TICKS(1));
        for (int c=0;c<KEYPAD_COLS;c++) {
            if (gpio_get_level(or_keypad_col_pins[c]) == 0) {
                gpio_set_level(or_keypad_row_pins[r], 1);
                return keys[r][c];
            }
        }
        gpio_set_level(or_keypad_row_pins[r], 1);
    }
    return 0;
}

static void process_key(char k) {
    switch(k) {
        case '1': or_send_cmd(CMD_CALIBRATE, NULL); break;
        case '2': or_send_cmd(CMD_HOME, NULL); break;
        case '3': or_send_cmd(CMD_SET_COLOR, "C0"); break;
        case '4': or_send_cmd(CMD_SET_COLOR, "C1"); break;
        case '5': or_send_cmd(CMD_SET_COLOR, "C2"); break;
        case '6': or_send_cmd(CMD_SET_COLOR, "C3"); break;
        default: break;
    }
}

void or_keypad_task(void *pvParameters) {
    (void)pvParameters;
    keypad_init();
    printf("Keypad task started\n");
    char last=0;
    uint32_t last_tick=0;
    for(;;) {
        char k = keypad_scan();
        if (k) {
            uint32_t now = xTaskGetTickCount();
            if (k!=last || (now-last_tick)>pdMS_TO_TICKS(200)) {
                process_key(k);
                last=k; last_tick=now;
            }
        } else {
            last=0;
        }
        vTaskDelay(pdMS_TO_TICKS(50));
    }
}
