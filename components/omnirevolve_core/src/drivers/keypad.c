// Simple 3x3 keypad scan. Minimal logging.
#include <stdio.h>
#include <string.h>
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "driver/gpio.h"
#include "omnirevolve/drivers/keypad.h"

#define KEYPAD_ROWS 3
#define KEYPAD_COLS 3

// Defaults; adjust as needed.
#ifndef KEYPAD_ROW0_PIN
#define KEYPAD_ROW0_PIN 32
#endif
#ifndef KEYPAD_ROW1_PIN
#define KEYPAD_ROW1_PIN 33
#endif
#ifndef KEYPAD_ROW2_PIN
#define KEYPAD_ROW2_PIN 27
#endif

#ifndef KEYPAD_COL0_PIN
#define KEYPAD_COL0_PIN 22
#endif
#ifndef KEYPAD_COL1_PIN
#define KEYPAD_COL1_PIN 4
#endif
#ifndef KEYPAD_COL2_PIN
#define KEYPAD_COL2_PIN 19
#endif

static const int row_pins[KEYPAD_ROWS] = {KEYPAD_ROW0_PIN, KEYPAD_ROW1_PIN, KEYPAD_ROW2_PIN};
static const int col_pins[KEYPAD_COLS] = {KEYPAD_COL0_PIN, KEYPAD_COL1_PIN, KEYPAD_COL2_PIN};

static const char keypad_keys[KEYPAD_ROWS][KEYPAD_COLS] = {
    {'1', '2', '3'},
    {'4', '5', '6'},
    {'7', '8', '9'}
};

typedef struct {
    char last_key;
    uint32_t last_key_time;
} keypad_state_t;

static keypad_state_t s_state = {0};
static keypad_key_cb_t s_cb = NULL;

void keypad_set_callback(keypad_key_cb_t cb) { s_cb = cb; }

void keypad_init(void) {
    for (int i = 0; i < KEYPAD_ROWS; i++) {
        gpio_set_direction(row_pins[i], GPIO_MODE_OUTPUT);
        gpio_set_level(row_pins[i], 1);
    }
    for (int i = 0; i < KEYPAD_COLS; i++) {
        gpio_set_direction(col_pins[i], GPIO_MODE_INPUT);
        gpio_set_pull_mode(col_pins[i], GPIO_PULLUP_ONLY);
    }
    printf("[keypad] init\n");
}

static char keypad_scan_once(void) {
    for (int r = 0; r < KEYPAD_ROWS; r++) {
        gpio_set_level(row_pins[r], 0);
        vTaskDelay(pdMS_TO_TICKS(1));
        for (int c = 0; c < KEYPAD_COLS; c++) {
            if (gpio_get_level(col_pins[c]) == 0) {
                gpio_set_level(row_pins[r], 1);
                return keypad_keys[r][c];
            }
        }
        gpio_set_level(row_pins[r], 1);
    }
    return 0;
}

static void keypad_task(void *arg) {
    (void)arg;
    printf("[keypad] task\n");
    while (1) {
        char key = keypad_scan_once();
        if (key != 0) {
            uint32_t now = xTaskGetTickCount();
            if (key != s_state.last_key || (now - s_state.last_key_time) > pdMS_TO_TICKS(200)) {
                if (s_cb) s_cb(key);
                s_state.last_key = key;
                s_state.last_key_time = now;
            }
        } else {
            s_state.last_key = 0;
        }
        vTaskDelay(pdMS_TO_TICKS(50));
    }
}

void keypad_start_task(void) {
    xTaskCreatePinnedToCore(keypad_task, "keypad", 4096, NULL, 1, NULL, 1);
}
