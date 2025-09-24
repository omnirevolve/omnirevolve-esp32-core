// Core orchestration: state, periodic polls, keypad mapping.
#include <stdio.h>
#include <string.h>
#include <stdlib.h>
#include <inttypes.h>
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "freertos/semphr.h"
#include "driver/gpio.h"
#include "omnirevolve/protocol/cmd_ids.h"
#include "omnirevolve/protocol/uart_framing.h"
#include "omnirevolve/drivers/keypad.h"
#include "omnirevolve/drivers/oled_display.h"
#include "omnirevolve/link/uart_link.h"
#include "omnirevolve/link/stm_link.h"
#include "omnirevolve/esp32_core/esp32_core.h"

#define HEARTBEAT_PERIOD_MS    1000
#define CONNECT_TIMEOUT_MS     3000

// UART wiring (default)
#define STM32_UART_PORT UART_NUM_2
#define STM32_TX_PIN 25
#define STM32_RX_PIN 26
#define STM32_UART_BAUD 115200

typedef enum {
    CURRENT_CMD_IDLE = 0,
    CURRENT_CMD_CALIBRATING,
    CURRENT_CMD_HOMING,
    CURRENT_CMD_DRAWING,
    CURRENT_CMD_ERROR
} CurrentCmd_t;

typedef struct {
    uint32_t request_id;
    CmdState_t cmd_state;
} CurrentCommand_t;

typedef struct {
    volatile plotter_state_t state;
    TickType_t last_heartbeat_sent;
    TickType_t last_status_request;
    TickType_t last_position_request;
    TickType_t last_cmd_status_request;
    TickType_t last_rx_any;
    TickType_t last_rx_heartbeat;
    char last_command[64];
    volatile CurrentCommand_t current_cmd_status;
} plotter_manager_t;

static plotter_manager_t g_plotter = {
    .state.is_connected = 0,
    .state.is_calibrated = 0,
    .state.is_homed = 0,
    .state.is_processing_cmd = 0,
    .state.is_idle = 0,
    .state.x_pos = -1,
    .state.y_pos = -1,
    .state.x_max = 0,
    .state.y_max = 0,
    .state.pen_is_down = 0,
    .state.current_color = 0,
    .state.current_cmd = CURRENT_CMD_IDLE,

    .last_heartbeat_sent = 0,
    .last_status_request = 0,
    .last_position_request = 0,
    .last_cmd_status_request = 0,
    .last_rx_any = 0,
    .last_rx_heartbeat = 0,
    .current_cmd_status = { .request_id = 0, .cmd_state = PLOTTER_CMD_STATE__UNDEFINED }
};

static SemaphoreHandle_t s_init_done = NULL;

// Optional string names for debug
static const char* command_names[CMD_COUNT] = {
    "INVALID","CALIBRATE","CMD_HOME","EMERGENCY_STOP","PEN_UP","PEN_DOWN",
    "SET_COLOR","GET_COLOR","DRAW_BEGIN","DUMMY_ACTIVE_COMMANDS_DELIMITER",
    "HEARTBEAT","GET_STATUS","GET_POSITION","GET_LIMITS","GET_CMD_STATUS"
};

// ------------- UART RX callbacks -> decode RESP and update state -------------
static inline uint16_t rd_u16le(const uint8_t *p){ return (uint16_t)(p[0] | ((uint16_t)p[1]<<8)); }
static inline uint32_t rd_u32le(const uint8_t *p){ return (uint32_t)p[0] | ((uint32_t)p[1]<<8) | ((uint32_t)p[2]<<16) | ((uint32_t)p[3]<<24); }

static void process_cmd_response(uint8_t status, uint32_t req_id) {
    if (g_plotter.current_cmd_status.request_id == req_id) {
        if (status == RESP_ACK) {
            g_plotter.current_cmd_status.cmd_state = PLOTTER_CMD_STATE__RECEIVED;
        } else if (status == RESP_DONE) {
            g_plotter.current_cmd_status.cmd_state = PLOTTER_CMD_STATE__FINISHED;
            g_plotter.state.current_cmd = CURRENT_CMD_IDLE;
        } else if (status == RESP_BUSY) {
            // keep as-is
        } else if (status == RESP_ERR) {
            g_plotter.state.current_cmd = CURRENT_CMD_ERROR;
            g_plotter.current_cmd_status.cmd_state = PLOTTER_CMD_STATE__FAILED;
        }
    }
}

static void on_uart_resp(const uint8_t *p, uint16_t n) {
    if (n < 8) return;
    uint32_t req_id = rd_u32le(p+0);
    uint8_t  cmd_id = p[4];
    uint8_t  status = p[5];
    uint16_t dlen   = rd_u16le(p+6);
    if (8u + dlen != n) return;
    const uint8_t *d = p + 8;

    TickType_t now = xTaskGetTickCount();
    g_plotter.last_rx_any = now;
    if (cmd_id == CMD_HEARTBEAT) g_plotter.last_rx_heartbeat = now;

    switch (cmd_id) {
        case CMD_GET_STATUS:
            if (dlen >= 5) {
                uint8_t flags = d[0];
                uint32_t bytes = rd_u32le(d+1);
                bool new_is_cal        = (flags & 0x80u) != 0;
                bool new_is_home       = (flags & 0x40u) != 0;
                bool new_is_terminal   = (flags & 0x20u) != 0;
                bool new_stream_active = (flags & 0x10u) != 0;
                g_plotter.state.is_calibrated   = new_is_cal ? 1 : 0;
                g_plotter.state.is_homed        = new_is_home ? 1 : 0;
                g_plotter.state.bytes_processed = bytes;
                g_plotter.state.is_processing_cmd = new_stream_active ? 1 : 0;
                g_plotter.state.is_idle           = (!new_stream_active && new_is_terminal) ? 1 : 0;
            }
            break;
        case CMD_GET_CMD_STATUS:
            if (dlen >= 5) {
                uint32_t qid = rd_u32le(d + 0);
                uint8_t  st  = d[4];
                if (g_plotter.current_cmd_status.request_id == qid) {
                    g_plotter.current_cmd_status.cmd_state = (CmdState_t)st;
                    if (st == PLOTTER_CMD_STATE__FINISHED) {
                        g_plotter.state.current_cmd = CURRENT_CMD_IDLE;
                    } else if (st == PLOTTER_CMD_STATE__FAILED) {
                        g_plotter.state.current_cmd = CURRENT_CMD_ERROR;
                    }
                }
            }
            break;
        case CMD_GET_POSITION:
            if (dlen >= 8) {
                int32_t xs = (int32_t)rd_u32le(d+0);
                int32_t ys = (int32_t)rd_u32le(d+4);
                g_plotter.state.x_pos = xs;
                g_plotter.state.y_pos = ys;
            } else {
                g_plotter.state.x_pos = -1; g_plotter.state.y_pos = -1;
            }
            break;
        case CMD_GET_LIMITS:
            if (dlen >= 8) {
                float xmm, ymm;
                memcpy(&xmm, d+0, 4);
                memcpy(&ymm, d+4, 4);
                g_plotter.state.x_max = xmm;
                g_plotter.state.y_max = ymm;
            }
            break;
        case CMD_GET_COLOR:
            if (dlen >= 1) {
                g_plotter.state.current_color = d[0];
            }
            break;
        case CMD_CALIBRATE:
            process_cmd_response(status, req_id);
            if (status == RESP_ACK) g_plotter.state.current_cmd = CURRENT_CMD_CALIBRATING;
            if (status == RESP_DONE) { plotter_send_cmd(CMD_GET_LIMITS, NULL); }
            break;
        case CMD_HOME:
            process_cmd_response(status, req_id);
            if (status == RESP_ACK) g_plotter.state.current_cmd = CURRENT_CMD_HOMING;
            break;
        case CMD_DRAW_BEGIN:
            process_cmd_response(status, req_id);
            if (status == RESP_ACK) g_plotter.state.current_cmd = CURRENT_CMD_DRAWING;
            break;
        case CMD_PEN_UP:
        case CMD_PEN_DOWN:
        case CMD_SET_COLOR:
        case CMD_EMERGENCY_STOP:
            process_cmd_response(status, req_id);
            break;
        default: break;
    }
    g_plotter.state.is_connected = 1;
}

static void on_uart_debug(const char *line) {
    printf("STM32[DBG]: %s\n", line);
}

// ------------- Keypad integration -------------
static void on_key(char key) {
    printf("\n=== KEYPAD: Key '%c' pressed ===\n", key);

    switch(key) {
        case '1':
            printf("KEYPAD: Starting calibration\n");
            plotter_send_cmd(CMD_CALIBRATE, NULL);
            break;

        case '2':
            printf("KEYPAD: Starting homing\n");
            plotter_send_cmd(CMD_HOME, NULL);
            break;

        case '3':
            printf("KEYPAD: Set color #0\n");
            plotter_send_cmd(CMD_SET_COLOR, "C0");
            break;

        case '4':
            printf("KEYPAD: Set color #1\n");
            plotter_send_cmd(CMD_SET_COLOR, "C1");
            break;

        case '5':
            printf("KEYPAD: Set color #2\n");
            plotter_send_cmd(CMD_SET_COLOR, "C2");
            break;

        case '6':
            printf("KEYPAD: Set color #3\n");
            plotter_send_cmd(CMD_SET_COLOR, "C3");
            break;

        default:
            printf("KEYPAD: Unknown key\n");
            break;
    }
}

// ------------- Control task -------------
static void control_task(void *arg) {
    (void)arg;
    vTaskDelay(pdMS_TO_TICKS(500));
    plotter_send_cmd(CMD_HEARTBEAT, NULL);
    g_plotter.last_heartbeat_sent = xTaskGetTickCount();
    for (;;) {
        TickType_t now = xTaskGetTickCount();
        if ((now - g_plotter.last_heartbeat_sent) >= pdMS_TO_TICKS(HEARTBEAT_PERIOD_MS)) {
            plotter_send_cmd(CMD_HEARTBEAT, NULL);
            g_plotter.last_heartbeat_sent = now;
        }
        if ((now - g_plotter.last_status_request) >= pdMS_TO_TICKS(1000)) {
            plotter_send_cmd(CMD_GET_STATUS, NULL);
            g_plotter.last_status_request = now;
        }
        if ((now - g_plotter.last_position_request) >= pdMS_TO_TICKS(500)) {
            plotter_send_cmd(CMD_GET_POSITION, NULL);
            g_plotter.last_position_request = now;
        }
        if ((now - g_plotter.last_cmd_status_request) >= pdMS_TO_TICKS(500)) {
            g_plotter.last_cmd_status_request = now;
            const CmdState_t cs = g_plotter.current_cmd_status.cmd_state;
            const bool active = (cs == PLOTTER_CMD_STATE__RECEIVED || cs == PLOTTER_CMD_STATE__EXECUTING);
            if (active) {
                char p[16];
                snprintf(p, sizeof(p), "%" PRIu32, (uint32_t)g_plotter.current_cmd_status.request_id);
                plotter_send_cmd(CMD_GET_CMD_STATUS, p);
            }
        }
        TickType_t last_alive = (g_plotter.last_rx_heartbeat > g_plotter.last_rx_any) ? g_plotter.last_rx_heartbeat : g_plotter.last_rx_any;
        bool prev = g_plotter.state.is_connected;
        g_plotter.state.is_connected = (last_alive != 0) && ((now - last_alive) <= pdMS_TO_TICKS(CONNECT_TIMEOUT_MS));
        if (prev != g_plotter.state.is_connected) {
            printf("[link] connected=%d\n", g_plotter.state.is_connected);
        }
        vTaskDelay(pdMS_TO_TICKS(50));
    }
}

// ------------- Public API -------------
uint32_t plotter_send_cmd(command_id_t cmd_id, const char* params) {
    if (cmd_id < CMD_COUNT) snprintf((char*)g_plotter.last_command, sizeof(g_plotter.last_command), "%s", command_names[cmd_id]);
    uint32_t req_id = uart_link_send_cmd(cmd_id, params);
    if (cmd_id < CMD_DUMMY_ACTIVE_COMMANDS_DELIMITER) {
        g_plotter.current_cmd_status.request_id = req_id;
        g_plotter.current_cmd_status.cmd_state  = PLOTTER_CMD_STATE__UNDEFINED;
    }
    return req_id;
}

void plotter_send_draw_stream_data(const uint8_t* data, uint32_t len) {
    stm_link_send_draw_data(data, len);
}

void plotter_get_state(plotter_state_t *ps) { *ps = g_plotter.state; }

void plotter_start_all_tasks(void) {
    uart_link_start();
    keypad_start_task();
    oled_start_task();
    xTaskCreatePinnedToCore(control_task, "plotter_control", 4096, NULL, 2, NULL, 1);
}

static void state_reader(plotter_state_t *out) {
    plotter_get_state(out);
}

static void init_task(void *arg) {
    ready_signal_isr_callback cb = (ready_signal_isr_callback)arg;
    printf("[core] init\n");
    g_plotter.last_heartbeat_sent = 0;
    strcpy((char*)g_plotter.last_command, "None");

    stm_link_init();
    oled_init();
    oled_set_state_reader(state_reader);
    uart_link_init(STM32_UART_PORT, STM32_TX_PIN, STM32_RX_PIN, STM32_UART_BAUD);
    if (cb) stm_link_config_ready_pin(cb);

    keypad_init();
    keypad_set_callback(on_key);

    uart_link_set_callbacks(on_uart_resp, on_uart_debug);

    xSemaphoreGive(s_init_done);
    vTaskDelete(NULL);
}

void plotter_init_sync(ready_signal_isr_callback cb) {
    if (!s_init_done) s_init_done = xSemaphoreCreateBinary();
    xTaskCreatePinnedToCore(init_task, "plotter_io_init", 2*4096, (void*)cb, 9, NULL, 1);
    xSemaphoreTake(s_init_done, portMAX_DELAY);
}
