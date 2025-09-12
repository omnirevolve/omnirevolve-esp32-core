#include "esp32_to_stm32.h"

#include <stdio.h>
#include <string.h>
#include <inttypes.h>
#include <math.h>
#include <errno.h>

#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "driver/spi_master.h"
#include "driver/gpio.h"
#include "driver/uart.h"
#include "shared/cmd_ids.h"
#include "driver/spi_master.h" 


// SPI Configuration for STM32 communication
#define STM32_SPI_HOST    HSPI_HOST
#define STM32_SPI_MOSI    13   // GPIO13
#define STM32_SPI_MISO    12   // GPIO12 (not used but connected)
#define STM32_SPI_CLK     14   // GPIO14
//#define STM32_SPI_CS      15   // GPIO15

#define STM32_READY_PIN   15   // GPIO15 - READY signal from STM32
// =============================== TEST STREAMING BEGIN 1 ===================================

// Continuous streaming parameters

#define HEARTBEAT_PERIOD_MS    1000 
#define CONNECT_TIMEOUT_MS     3000

#define UART_BUFFER_SIZE 1024

static spi_device_handle_t stm32_spi = NULL;

const char* command_names[CMD_COUNT] = {
    "INVALID",
    "CALIBRATE", 
    "CMD_HOME",
    "EMERGENCY_STOP",
    "PEN_UP",
    "PEN_DOWN",
    "SET_COLOR",
    "GET_COLOR",
    "DRAW_BEGIN",
    "DUMMY_ACTIVE_COMMANDS_DELIMITER",
    "HEARTBEAT",
    "GET_STATUS",
    "GET_POSITION",
    "GET_LIMITS",
    "GET_CMD_STATUS"
};

// UART to STM32
#define STM32_UART_PORT UART_NUM_2
#define STM32_TX_PIN 25  // GPIO25
#define STM32_RX_PIN 26  // GPIO26 


#define KEYPAD_ROWS 3
#define KEYPAD_COLS 3

static const int row_pins[KEYPAD_ROWS] = {32, 33, 27};
static const int col_pins[KEYPAD_COLS] = {22, 4, 19};

static const char keypad_keys[KEYPAD_ROWS][KEYPAD_COLS] = {
    {'1', '2', '3'},
    {'4', '5', '6'},
    {'7', '8', '9'}
};

typedef struct {
    char last_key;
    uint32_t last_key_time;
} keypad_state_t;

typedef struct {
	uint32_t request_id;
	CmdState_t cmd_state;
} CurrentCommand_t;

typedef struct {
    volatile plotter_state_t state;

    TickType_t last_heartbeat_sent;
    TickType_t last_status_request;
    TickType_t last_rx_any;
    TickType_t last_rx_heartbeat;

    uint8_t heartbeat_ok;
    char status_text[64];
    char last_command[64];
    char last_response[64];
    volatile CurrentCommand_t current_cmd_status;
    volatile uint32_t request_counter;
} plotter_manager_t;

static plotter_manager_t plotter = {
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
    .state.feedrate = 3000,
    .state.speed_factor = 1.0,

    .last_heartbeat_sent = 0,
    .last_status_request = 0,
    .last_rx_any = 0,
    .last_rx_heartbeat = 0,
    .heartbeat_ok = 0,
    .request_counter = 1
};

static keypad_state_t keypad_state = {
    .last_key = 0,
    .last_key_time = 0
};

static SemaphoreHandle_t s_plotter_io_devices_init_done = NULL;

// =========================== PROTOTYPES =============================

// =========================== STM32 IO ROUTINES =============================
uint32_t send_to_stm32_cmd(command_id_t cmd_id, const char* params) {
    char full_msg[UART_BUFFER_SIZE];
    
    // Добавить проверку
    if (cmd_id >= CMD_COUNT) {
        printf("ERROR: Invalid cmd_id %d\n", cmd_id);
        return 0;
    }
    
    if (params && strlen(params) > 0) {
        snprintf(full_msg, sizeof(full_msg), "%" PRIu32 ":%d:%s\n",
         (uint32_t)plotter.request_counter, cmd_id, params ? params : "");
    } else {
        snprintf(full_msg, sizeof(full_msg), "%" PRIu32 ":%d\n",
         (uint32_t)plotter.request_counter, cmd_id);
    }
    if (cmd_id < CMD_DUMMY_ACTIVE_COMMANDS_DELIMITER)
    {
        plotter.current_cmd_status.request_id = plotter.request_counter;
        plotter.current_cmd_status.cmd_state = CMD_NOT_PRESENTED;
    }
    
    if (cmd_id != CMD_GET_CMD_STATUS && cmd_id != CMD_GET_STATUS && cmd_id != CMD_HEARTBEAT)
        printf("-> STM32: %s", full_msg);
    
    uart_write_bytes(STM32_UART_PORT, full_msg, strlen(full_msg));
    uart_wait_tx_done(STM32_UART_PORT, pdMS_TO_TICKS(50));
    
    if (cmd_id < CMD_COUNT) {
        snprintf(plotter.last_command, sizeof(plotter.last_command), "%s", 
                 command_names[cmd_id]);
    }

    return plotter.request_counter++;
}

// =========================== KEYPAD =============================

void keypad_init(void) {
    for (int i = 0; i < KEYPAD_ROWS; i++) {
        gpio_set_direction(row_pins[i], GPIO_MODE_OUTPUT);
        gpio_set_level(row_pins[i], 1); // HIGH by default
    }
    
    // input, pull-up:
    for (int i = 0; i < KEYPAD_COLS; i++) {
        gpio_set_direction(col_pins[i], GPIO_MODE_INPUT);
        gpio_set_pull_mode(col_pins[i], GPIO_PULLUP_ONLY);
    }
    
    printf("Keypad initialized\n");
}

char keypad_scan(void) {
    for (int row = 0; row < KEYPAD_ROWS; row++) {
        gpio_set_level(row_pins[row], 0); // row -> LOW
        vTaskDelay(pdMS_TO_TICKS(1)); // stab delay
        for (int col = 0; col < KEYPAD_COLS; col++) { // check columns:
            if (gpio_get_level(col_pins[col]) == 0) { // button pressed
                gpio_set_level(row_pins[row], 1); // resume HIGH
                return keypad_keys[row][col];
            }
        }
        gpio_set_level(row_pins[row], 1); // resumed HIGH for the row
    }
    return 0;
}

void process_keypad_command(char key);

void keypad_task(void *pvParameters) {
    printf("Keypad task started\n");
    keypad_init();
    while (1) {
        char key = keypad_scan();
        if (key != 0) {
            // Debounce in 200ms
            uint32_t current_time = xTaskGetTickCount();
            if (key != keypad_state.last_key || 
                (current_time - keypad_state.last_key_time) > pdMS_TO_TICKS(200)) {
                
                process_keypad_command(key);
                keypad_state.last_key = key;
                keypad_state.last_key_time = current_time;
            }
        } else {
            keypad_state.last_key = 0;
        }
        vTaskDelay(pdMS_TO_TICKS(50));
    }
}

// ========================= UART COMMANDS CONTROL ROUTINES ===============
static int hexval(int c) {
    if (c >= '0' && c <= '9') return c - '0';
    if (c >= 'a' && c <= 'f') return 10 + (c - 'a');
    if (c >= 'A' && c <= 'F') return 10 + (c - 'A');
    return -1;
}

void process_stm32_response(const char* response) {
    uint32_t received_id;
    uint32_t cmd_id;
    char data[UART_BUFFER_SIZE] = {0};
    
    int parsed = sscanf(response, "%" SCNu32 ":%" SCNu32 ":%255s",
                    &received_id, &cmd_id, data);
    
    if (parsed >= 2 && received_id > 0 && cmd_id < CMD_COUNT) {
        if (cmd_id != CMD_GET_CMD_STATUS && cmd_id != CMD_GET_STATUS && cmd_id != CMD_HEARTBEAT)
            printf("<- STM32: %s\n", response);
            
        TickType_t now = xTaskGetTickCount();
        plotter.last_rx_any = now;
        
        switch (cmd_id) {
            case CMD_HEARTBEAT:
                plotter.last_rx_heartbeat = now;
                break;
                
            case CMD_GET_STATUS: {
                // Ожидаем строку "HH:DDDD..." (2 hex-цифры + ':' + десятичный uint32)
                const char *p = data;
                if (p && strlen(p) >= 4) {
                    int h1 = hexval((unsigned char)p[0]);
                    int h2 = hexval((unsigned char)p[1]);
                    if (h1 >= 0 && h2 >= 0 && p[2] == ':') {
                        uint8_t status = (uint8_t)((h1 << 4) | h2);

                        // Битовая схема (STM32->ESP32):
                        // bit7 (0x80): is_calibrated
                        // bit6 (0x40): is_homed
                        // bit5 (0x20): терминальное состояние команды (не в PlotterTelemetry, но можно держать в state)
                        // bit4 (0x10): поток рисования активен (не в PlotterTelemetry)

                        bool new_is_cal       = (status & 0x80u) != 0;
                        bool new_is_home      = (status & 0x40u) != 0;
                        bool new_is_terminal  = (status & 0x20u) != 0;
                        bool new_stream_active= (status & 0x10u) != 0;

                        errno = 0;
                        char *endp = NULL;
                        unsigned long v = strtoul(p + 3, &endp, 10);
                        if (errno == 0 && endp != p + 3 && *endp == '\0' && v <= UINT32_MAX) {
                            plotter.state.is_calibrated   = (uint8_t)new_is_cal;
                            plotter.state.is_homed        = (uint8_t)new_is_home;
                            plotter.state.bytes_processed = (uint32_t)v;

                            plotter.state.is_processing_cmd = new_stream_active ? 1 : 0;
                            plotter.state.is_idle           = (!new_stream_active && new_is_terminal) ? 1 : 0;
                        } else {
                            printf("ERROR: GET_STATUS bytes parse failed: %s\n", p);
                        }
                    } else {
                        printf("ERROR: GET_STATUS header parse failed: %s\n", p);
                    }
                } else {
                    printf("ERROR: GET_STATUS too short: %s\n", p ? p : "<null>");
                }
                break;
            }

            case CMD_GET_CMD_STATUS:
            {
                uint32_t request_id = 0;
                int cmd_state_int = CMD_NOT_PRESENTED;
                
                if (sscanf(data, "%" SCNu32 ":%d", &request_id, &cmd_state_int) == 2) {
                    CmdState_t cmd_state = (CmdState_t)cmd_state_int;
                    
                    if (plotter.current_cmd_status.request_id == request_id) {
                        if (plotter.current_cmd_status.cmd_state != cmd_state) {
                            printf("Current command %" PRIu32 " status changed: %d -> %d\n",
                                request_id, plotter.current_cmd_status.cmd_state, cmd_state);
                            plotter.current_cmd_status.cmd_state = cmd_state;
                        }
                    } else if (request_id != 0) {
                        printf("Received status for command %" PRIu32 ": %d\n",
                            request_id, cmd_state);
                        plotter.current_cmd_status.request_id = request_id;
                        plotter.current_cmd_status.cmd_state = cmd_state;
                    }
                } else {
                    printf("ERROR: Unable to parse CMD_GET_CMD_STATUS response: %s\n", data);
                }
                break;
            }            
            
            case CMD_GET_POSITION:
            if (strstr(data, "?")) {
                plotter.state.x_pos = -1;
                plotter.state.y_pos = -1;
            } else {
                int32_t x_steps, y_steps;
                sscanf(data, "X%" SCNd32 ":Y%" SCNd32, &x_steps, &y_steps);
                plotter.state.x_pos = x_steps;
                plotter.state.y_pos = y_steps;
            }
            break;
            
            case CMD_GET_LIMITS:
                {
                    float x_mm, y_mm;
                    if (sscanf(data, "X%f,Y%f", &x_mm, &y_mm) == 2) {
                        plotter.state.x_max = x_mm;
                        plotter.state.y_max = y_mm;
                        printf("Limits received: X=%.2f mm, Y=%.2f mm\n", x_mm, y_mm);
                    } else {
                        printf("Failed to parse limits from: %s\n", data);
                    }
                }
                break;
                    
            case CMD_GET_COLOR:
                sscanf(data, "C%hhu:%15s", &plotter.state.current_color, plotter.state.color_name);
                break;
                
            case CMD_CALIBRATE:
            case CMD_HOME:
            case CMD_EMERGENCY_STOP:
            case CMD_PEN_UP:
            case CMD_PEN_DOWN:
            case CMD_SET_COLOR:
            case CMD_DRAW_BEGIN: {
                if (strcmp(data, "OK") == 0) {
                    printf("Command %s started\n", command_names[cmd_id]);
                } else if (strcmp(data, "DONE") == 0) {
                    printf("Command %s completed\n", command_names[cmd_id]);
                    if (plotter.current_cmd_status.request_id == received_id) {
                        plotter.current_cmd_status.cmd_state = CMD_FINISHED;
                    }
                } else if (strncmp(data, "ERROR", 5) == 0) {
                    printf("Command %s failed: %s\n", command_names[cmd_id], data);
                    if (plotter.current_cmd_status.request_id == received_id) {
                        plotter.current_cmd_status.cmd_state = CMD_FAILED;
                    }
                } else if (strcmp(data, "BUSY") == 0) {
                    printf("Command %s rejected - system busy\n", command_names[cmd_id]);
                }
                break;
            }
                
            default:
                printf("Unknown cmd_id: %" PRIu32 "\n", cmd_id);
                break;
        }
    }
    else {
    }
    plotter.state.is_connected = true;
}

void stm32_uart_task(void *pvParameters) {
    uint8_t data;
    char buffer[UART_BUFFER_SIZE];
    size_t pos = 0;

    printf("UART: Entering read loop\n");

    while (1) {
        if (uart_read_bytes(STM32_UART_PORT, &data, 1, pdMS_TO_TICKS(100)) == 1) {
            if (data == '\n' || data == '\r') {
                if (pos > 0) {
                    buffer[pos] = '\0';
                    printf("STM32 says:  '%s'\n", buffer);
                    process_stm32_response(buffer);
                    pos = 0;
                }
            } else if (pos < sizeof(buffer) - 1) {
                if (data >= 32 || data == '\t') {
                    buffer[pos++] = data;
                    if (pos >= UART_BUFFER_SIZE)
                    {
                        printf("ATTENTION: Overflow attempt in UART");
                        pos = 0;
                    }
                }
            } else {
                pos = 0;
            }
        }
        
        taskYIELD();
    }
}

// ========================= PLOTTER CONTROL ROUTINES ====================
void plotter_control_task(void *pvParameters) {
    printf("Plotter control task started\n");
    
    vTaskDelay(pdMS_TO_TICKS(500));
    
    send_to_stm32_cmd(CMD_HEARTBEAT, NULL);
    plotter.last_heartbeat_sent = xTaskGetTickCount();
    
    while (1) {
        TickType_t now = xTaskGetTickCount();
        if ((now - plotter.last_heartbeat_sent) >= pdMS_TO_TICKS(HEARTBEAT_PERIOD_MS)) {
            send_to_stm32_cmd(CMD_HEARTBEAT, NULL);
            plotter.last_heartbeat_sent = now;
        }
        if ((now - plotter.last_status_request) >= pdMS_TO_TICKS(1000)) {
            send_to_stm32_cmd(CMD_GET_STATUS, NULL);
            plotter.last_status_request = now;
        }
        TickType_t last_alive = (plotter.last_rx_heartbeat > plotter.last_rx_any)
                                ? plotter.last_rx_heartbeat
                                : plotter.last_rx_any;
        bool prev_connected = plotter.state.is_connected;
        plotter.state.is_connected = (last_alive != 0) &&
                            ((now - last_alive) <= pdMS_TO_TICKS(CONNECT_TIMEOUT_MS));
        if (prev_connected != plotter.state.is_connected) {
            printf("[link] is_connected = %s\n", plotter.state.is_connected ? "true" : "false");
        }
        vTaskDelay(pdMS_TO_TICKS(50));    
    }
}

// ============ KEYPAD HANDLING ============

void process_keypad_command(char key) {
    printf("\n=== KEYPAD: Key '%c' pressed ===\n", key);
    
    switch(key) {
        case '1':
            printf("KEYPAD: Starting calibration\n");
            send_to_stm32_cmd(CMD_CALIBRATE, NULL);
            break;
            
        case '2':
            printf("KEYPAD: Starting homing\n");
            send_to_stm32_cmd(CMD_HOME, NULL);
            break;

        case '3':
            printf("KEYPAD: Set color #0\n");
            send_to_stm32_cmd(CMD_SET_COLOR, "C0");
            break;

        case '4':
            printf("KEYPAD: Set color #1\n");
            send_to_stm32_cmd(CMD_SET_COLOR, "C1");
            break;

        case '5':
            printf("KEYPAD: Set color #2\n");
            send_to_stm32_cmd(CMD_SET_COLOR, "C2");
            break;

        case '6':
            printf("KEYPAD: Set color #2\n");
            send_to_stm32_cmd(CMD_SET_COLOR, "C3");
            break;

        default:
            printf("KEYPAD: Unknown key\n");
            break;
    }
}

// ============ SPI & UART INIT ROUTINES ============
static void stm32_spi_init(void)
{
    printf("\n=== STM32 SPI INITIALIZATION ===\n");

    // Initialize SPI bus
    spi_bus_config_t buscfg = {
        .mosi_io_num = STM32_SPI_MOSI,
        .miso_io_num = -1,
        .sclk_io_num = STM32_SPI_CLK,
        .quadwp_io_num = -1,
        .quadhd_io_num = -1,
        .max_transfer_sz = SPI_CHUNK_SIZE,
        .flags = SPICOMMON_BUSFLAG_MASTER};

    esp_err_t ret = spi_bus_initialize(STM32_SPI_HOST, &buscfg, 1);
    if (ret != ESP_OK)
    {
        printf("ERROR: SPI bus init failed: %s\n", esp_err_to_name(ret));
        return;
    }
    printf("SPI bus initialized successfully\n");

    // Add STM32 as SPI device
    spi_device_interface_config_t devcfg = {
        .mode = 0,
        .clock_speed_hz = 8 * 1000 * 1000,
        .spics_io_num = -1,
        .queue_size = 8,
    };

    ret = spi_bus_add_device(STM32_SPI_HOST, &devcfg, &stm32_spi);
    if (ret != ESP_OK)
    {
        printf("ERROR: Failed to add SPI device: %s\n", esp_err_to_name(ret));
        return;
    }

    printf("STM32 SPI device added successfully\n");
    printf("Configuration:\n");
    printf("  MOSI: GPIO%d\n", STM32_SPI_MOSI);
    printf("  CLK:  GPIO%d\n", STM32_SPI_CLK);
    printf("  Ready pin: GPIO%d\n", STM32_READY_PIN);
    printf("  Speed: %d Hz\n", devcfg.clock_speed_hz);
    printf("  Mode: 0 (CPOL=0, CPHA=0)\n");
}

static void config_ready_pin(ready_signal_isr_callback isr_callback)
{
    gpio_config_t io = {
        .pin_bit_mask = 1ULL << STM32_READY_PIN,
        .mode = GPIO_MODE_INPUT,
        .pull_up_en = GPIO_PULLUP_DISABLE,
        .pull_down_en = GPIO_PULLDOWN_DISABLE,
        .intr_type = GPIO_INTR_ANYEDGE
    };
    gpio_config(&io);

    esp_err_t err = gpio_install_isr_service(ESP_INTR_FLAG_IRAM);
    if (err != ESP_OK && err != ESP_ERR_INVALID_STATE) {
        ESP_ERROR_CHECK(err);
    }

    ESP_ERROR_CHECK(gpio_isr_handler_add(STM32_READY_PIN, isr_callback, NULL));
    ESP_ERROR_CHECK(gpio_intr_enable(STM32_READY_PIN));
}

void uart_init(void)
{
    // Initialize UART to STM32
    uart_config_t uart_config = {
        .baud_rate = 115200,
        .data_bits = UART_DATA_8_BITS,
        .parity = UART_PARITY_DISABLE,
        .stop_bits = UART_STOP_BITS_1,
        .flow_ctrl = UART_HW_FLOWCTRL_DISABLE,
        .source_clk = UART_SCLK_APB,
    };
    
    uart_driver_install(STM32_UART_PORT, UART_BUFFER_SIZE, UART_BUFFER_SIZE, 0, NULL, 0);
    uart_param_config(STM32_UART_PORT, &uart_config);
    uart_set_pin(STM32_UART_PORT, STM32_TX_PIN, STM32_RX_PIN, UART_PIN_NO_CHANGE, UART_PIN_NO_CHANGE);

    printf("UART2 configured: TX=GPIO%d, RX=GPIO%d\n", STM32_TX_PIN, STM32_RX_PIN);
}

// ============ HAL ROUTINES ============
static void plotter_init_task(void *arg) {
    ready_signal_isr_callback isr_callback = (ready_signal_isr_callback)arg;
    printf("Initializing plotter hardware...\n");
    plotter.last_heartbeat_sent = 0;
    strcpy(plotter.last_command, "None");
    strcpy(plotter.status_text, "Initializing");
    stm32_spi_init();
    uart_init();
    config_ready_pin(isr_callback);
    xSemaphoreGive(s_plotter_io_devices_init_done);
    vTaskDelete(NULL);
}

void plotter_init_sync(ready_signal_isr_callback cb)
{
    if (!s_plotter_io_devices_init_done) {
        s_plotter_io_devices_init_done = xSemaphoreCreateBinary();
    }
    xTaskCreatePinnedToCore(plotter_init_task, "plotter_io_devices_init",
                            4096, (void*)cb, 9, NULL, 1);
    xSemaphoreTake(s_plotter_io_devices_init_done, portMAX_DELAY);
}

uint32_t plotter_send_cmd(command_id_t cmd_id, const char* params) {
    return send_to_stm32_cmd(cmd_id, params);
}

void plotter_send_draw_stream_data(const uint8_t* data, uint32_t len) {
    if (!stm32_spi || !data || len == 0)
    {
        printf("ERROR: Invalid parameters for SPI send\n");
        return;
    }
    printf("INFO: sending draw data len = %" PRIu32 "\n",len);

    spi_transaction_t trans = {
        .length = len * 8,
        .tx_buffer = data,
        .rx_buffer = NULL,
        .flags = 0};

    esp_err_t ret = spi_device_transmit(stm32_spi, &trans);
    if (ret != ESP_OK)
    {
        printf("ERROR: SPI transmit failed: %s\n", esp_err_to_name(ret));
    }
    printf("INFO: sending draw data exited\n");
}

void plotter_get_state(plotter_state_t *ps) {
    *ps = plotter.state;
}

void plotter_start_all_tasks(void) {
    xTaskCreatePinnedToCore(stm32_uart_task, "stm32_uart", 4096, NULL, 3, NULL, 1);
    xTaskCreatePinnedToCore(plotter_control_task, "plotter_control", 4096, NULL, 2, NULL, 1);
    xTaskCreatePinnedToCore(keypad_task, "keypad", 4096, NULL, 1, NULL, 1);
}
