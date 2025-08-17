#include "esp32_to_stm32.h"

#include <stdio.h>
#include <string.h>
#include <inttypes.h>
#include <math.h>

#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "driver/spi_master.h"
#include "driver/gpio.h"
#include "driver/uart.h"
#include "shared/cmd_ids.h"
//#include "u8g2.h"
#include "driver/spi_master.h"

// SPI Configuration for STM32 communication
#define STM32_SPI_HOST    HSPI_HOST
#define STM32_SPI_MOSI    13   // GPIO13
#define STM32_SPI_MISO    12   // GPIO12 (not used but connected)
#define STM32_SPI_CLK     14   // GPIO14
#define STM32_SPI_CS      15   // GPIO15
#define STM32_READY_PIN   21   // GPIO21 - READY signal from STM32
// =============================== TEST STREAMING BEGIN 1 ===================================
// --- Streaming protocol (must match STM32) ---
#ifndef CHUNK
// Размер кусочка DATA-пейлоада; 512 оптимально (низкий overhead).
#define CHUNK 512
#endif

// В байте STREAM: low nibble -> [bit0:X_STEP][bit1:X_DIR+][bit2:Y_STEP][bit3:Y_DIR+]
// high nibble -> тот же набор битов, но сдвинутый на +4 (чтобы опустить STEP).
// Т.е. если X_STEP=1 в младшем полупериоде, то в старшем надо тоже поставить X_STEP=1.
#define STEPS_PER_MM 40.0f     // синхронизировать со STM32 (PULSES_PER_REV=1600, 20T GT2 => ~40 steps/mm)
#define STREAM_F_HZ  10000u    // частота тиков на STM32 (TIM4 Update)
#define STREAM_STEP_US  6u     // ширина высокого уровня STEP, если понадобится в будущем
#define STREAM_GUARD_US 6u
#define STREAM_INV_MASK 0x00   // инверсий DIR нет (см. логику на STM32)

// Отладочные параметры стрима
#define STREAM_PRELOAD_BYTES   2000    // сколько байт закачать ДО START (≈0.4 c буфера на 10 кГц)
#define STREAM_DATA_CHUNK      CHUNK   // размер одного кадра DATA
#define STREAM_BYTES_PER_SEC   (STREAM_F_HZ/2)  // 1 байт = 2 ниббла => 5000 Б/с при 10 кГц
#define STREAM_RATE_SAFETY_PC  98      // учёт overhead (%, 98% от теории)

#define STREAM_CMD_REQUEST_MORE  0x14
#define STREAM_CMD_END          0x15
#define STREAM_CMD_ACK          0x16

// Continuous streaming parameters
#define CONTINUOUS_CHUNK_SIZE    2048   // Send 2KB chunks
#define CONTINUOUS_BUFFER_SIZE   16384  // 16KB buffer for continuous data

#define HEARTBEAT_PERIOD_MS    1000
#define CONNECT_TIMEOUT_MS     3000

#define UART_BUFFER_SIZE 256

#ifndef SPI_DMA_CH_AUTO
#define SPI_DMA_CH_AUTO 1
#endif

static spi_device_handle_t stm32_spi = NULL;

const char* command_names[CMD_COUNT] = {
    "INVALID",
    "HOME",
    "CALIBRATE", 
    "EMERGENCY_STOP",
    "MOVE_TO",
    "G0_RAPID",
    "G1_LINEAR",
    "G28_HOME",
    "RESET_SYSTEM",
    "PEN_UP",
    "PEN_DOWN",
    "SET_PEN",
    "SET_COLOR",
    "GET_COLOR",
    "DRAW_BEGIN",
    "DRAW_FINISH_WHEN_EMPTY",
    "DUMMY_ACTIVE_COMMANDS_DELIMITER",
    "HEARTBEAT",
    "GET_STATUS",
    "GET_POSITION",
    "SET_SPEED",
    "SET_ACCELERATION",
    "M114_POSITION",
    "M112_EMERGENCY",
    "M220_SPEED_FACTOR",
    "GET_LIMITS",
    "SET_ORIGIN",
    "GET_DIAGNOSTICS",
    "GET_CMD_STATUS",
    "G90_ABSOLUTE",
    "G91_RELATIVE"
};

// Hardware pins for OLED
#define PIN_NUM_MOSI 23
#define PIN_NUM_CLK  18
#define PIN_NUM_CS   5
#define PIN_NUM_DC   17
#define PIN_NUM_RST  16

// UART to STM32
#define STM32_UART_PORT UART_NUM_2
#define STM32_TX_PIN 25  // GPIO25
#define STM32_RX_PIN 26  // GPIO26


// Добавить в начало файла ESP32
#define KEYPAD_ROWS 4
#define KEYPAD_COLS 4

#define STEPS_PER_MM  40.0f
#define MM_PER_STEP   (1.0f / STEPS_PER_MM)

// GPIO пины для клавиатуры (можно изменить под вашу схему)
static const int row_pins[KEYPAD_ROWS] = {32, 33, 27}; // Выходы (строки)
static const int col_pins[KEYPAD_COLS] = {22, 4, 19};  // Входы (столбцы)

// Матрица символов клавиатуры
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
    TickType_t last_ready_request;

    uint8_t heartbeat_ok;
    char status_text[64];
    char last_command[64];  // Increased from 32
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
    .last_ready_request = 0,
    .heartbeat_ok = 0,
    .request_counter = 1
};

static keypad_state_t keypad_state = {
    .last_key = 0,
    .last_key_time = 0
};

// =========================== PROTOTYPES =============================

// =========================== STM32 IO ROUTINES =============================
uint32_t send_to_stm32_cmd(command_id_t cmd_id, const char* params) {
    char full_msg[128];
    
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

uint8_t is_stm32_ready(void) {
    return gpio_get_level(STM32_READY_PIN) == 1;
}

void spi_send_nibbles(const uint8_t *data, size_t len)
{
    printf("\n=== SPI SEND NIBBLES ===\n");
    printf("Sending %zu bytes via SPI\n", len);

    if (!stm32_spi || !data || len == 0)
    {
        printf("ERROR: Invalid parameters for SPI send\n");
        return;
    }

    // Проверка READY сигнала
    int ready_wait = 0;
    while (!is_stm32_ready() && ready_wait < 50)
    {
        vTaskDelay(pdMS_TO_TICKS(10));
        ready_wait++;
    }

    if (!is_stm32_ready())
    {
        printf("ERROR: STM32 not ready after %dms wait\n", ready_wait * 10);
        printf("READY pin state: %d\n", gpio_get_level(STM32_READY_PIN));
        return;
    }

    printf("STM32 is ready, starting transfer\n");

    // Отправляем данные кусками
    size_t sent = 0;
    int chunk_num = 0;

    while (sent < len)
    {
        size_t chunk_size = (len - sent > SPI_CHUNK_SIZE) ? SPI_CHUNK_SIZE : (len - sent);

        printf("  Chunk %d: sending %zu bytes (offset %zu)\n", chunk_num++, chunk_size, sent);

        // Отладка: печать первых байт чанка
        if (chunk_size > 0)
        {
            printf("    First bytes: ");
            for (size_t i = 0; i < 4 && i < chunk_size; i++)
            {
                printf("%02X ", data[sent + i]);
            }
            printf("\n");
        }

        spi_transaction_t trans = {
            .length = chunk_size * 8, // В битах
            .tx_buffer = &data[sent],
            .rx_buffer = NULL,
            .flags = 0};

        esp_err_t ret = spi_device_transmit(stm32_spi, &trans);
        if (ret != ESP_OK)
        {
            printf("ERROR: SPI transmit failed: %s\n", esp_err_to_name(ret));
            break;
        }

        sent += chunk_size;

        // Проверяем READY после каждого чанка
        if (sent < len)
        {
            vTaskDelay(pdMS_TO_TICKS(5));
            if (!is_stm32_ready())
            {
                printf("WARNING: STM32 became not ready after %zu bytes\n", sent);
                vTaskDelay(pdMS_TO_TICKS(50));
            }
        }
    }

    printf("SPI transfer complete: sent %zu/%zu bytes\n", sent, len);
}

// =========================== KEYPAD =============================

void keypad_init(void) {
    // Настройка пинов строк как выходы
    for (int i = 0; i < KEYPAD_ROWS; i++) {
        gpio_set_direction(row_pins[i], GPIO_MODE_OUTPUT);
        gpio_set_level(row_pins[i], 1); // HIGH по умолчанию
    }
    
    // Настройка пинов столбцов как входы с подтяжкой
    for (int i = 0; i < KEYPAD_COLS; i++) {
        gpio_set_direction(col_pins[i], GPIO_MODE_INPUT);
        gpio_set_pull_mode(col_pins[i], GPIO_PULLUP_ONLY);
    }
    
    printf("Keypad initialized\n");
}

char keypad_scan(void) {
    for (int row = 0; row < KEYPAD_ROWS; row++) {
        // Устанавливаем текущую строку в LOW
        gpio_set_level(row_pins[row], 0);
        
        // Небольшая задержка для стабилизации
        vTaskDelay(pdMS_TO_TICKS(1));
        
        // Проверяем все столбцы
        for (int col = 0; col < KEYPAD_COLS; col++) {
            if (gpio_get_level(col_pins[col]) == 0) {
                // Кнопка нажата
                gpio_set_level(row_pins[row], 1); // Восстанавливаем HIGH
                return keypad_keys[row][col];
            }
        }
        
        // Восстанавливаем HIGH для этой строки
        gpio_set_level(row_pins[row], 1);
    }
    
    return 0; // Ничего не нажато
}

void process_keypad_command(char key);

void keypad_task(void *pvParameters) {
    printf("Keypad task started\n");
    
    keypad_init();
    
    while (1) {
        char key = keypad_scan();
        
        if (key != 0) {
            // Debounce - игнорируем повторные нажатия в течение 200мс
            uint32_t current_time = xTaskGetTickCount();
            if (key != keypad_state.last_key || 
                (current_time - keypad_state.last_key_time) > pdMS_TO_TICKS(200)) {
                
                process_keypad_command(key);
                keypad_state.last_key = key;
                keypad_state.last_key_time = current_time;
            }
        } else {
            keypad_state.last_key = 0; // Кнопка отпущена
        }
        
        vTaskDelay(pdMS_TO_TICKS(50)); // Сканируем каждые 50мс
    }
}

// ========================= UART COMMANDS CONTROL ROUTINES ===============
void process_stm32_response(const char* response) {
    uint32_t received_id;
    uint32_t cmd_id;
    char data[256] = {0};
    
    int parsed = sscanf(response, "%" SCNu32 ":%" SCNu32 ":%255s",
                    &received_id, &cmd_id, data);
    
    // Если это валидный формат команды - обрабатываем
    if (parsed >= 2 && received_id > 0 && cmd_id < CMD_COUNT) {
        if (cmd_id != CMD_GET_CMD_STATUS && cmd_id != CMD_GET_STATUS && cmd_id != CMD_HEARTBEAT)
            printf("<- STM32: %s\n", response);
            
        TickType_t now = xTaskGetTickCount();
        plotter.last_rx_any = now;
        
        switch (cmd_id) {
            case CMD_HEARTBEAT:
                plotter.last_rx_heartbeat = now;
                break;
                
            case CMD_GET_STATUS:
                
                printf("Processing CMD_GET_STATUS, data: %s\n", data);
                if (strlen(data) >= 4) {
                    plotter.state.is_calibrated = (data[0] == '1');
                    plotter.state.is_homed = (data[1] == '1');
                    plotter.state.is_processing_cmd = (data[2] == '1');
                    plotter.state.is_idle = (data[3] == '1');
                    printf("Status updated: cal=%d, home=%d, proc=%d, idle=%d\n",
                           plotter.state.is_calibrated, plotter.state.is_homed,
                           plotter.state.is_processing_cmd, plotter.state.is_idle);
                }
                break;
                
            case CMD_GET_CMD_STATUS:
            {
                uint32_t request_id = 0;
                int cmd_state_int = CMD_NOT_PRESENTED;
                
                if (sscanf(data, "%" SCNu32 ":%d", &request_id, &cmd_state_int) == 2) {
                    CmdState_t cmd_state = (CmdState_t)cmd_state_int;
                    
                    // Обновляем статус только если это наша команда
                    if (plotter.current_cmd_status.request_id == request_id) {
                        if (plotter.current_cmd_status.cmd_state != cmd_state) {
                            printf("Current command %" PRIu32 " status changed: %d -> %d\n",
                                request_id, plotter.current_cmd_status.cmd_state, cmd_state);
                            plotter.current_cmd_status.cmd_state = cmd_state;
                        }
                    } else if (request_id != 0) {
                        // Это другая команда - обновляем
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
                    
            case CMD_GET_DIAGNOSTICS: {
                int pen_state;
                sscanf(data, "PEN:%d,FEED:%hu,SF:%f", 
                    &pen_state, &plotter.state.feedrate, &plotter.state.speed_factor);
                plotter.state.pen_is_down = (pen_state == 1);
                break;
            }
                
            case CMD_GET_COLOR:
                sscanf(data, "C%hhu:%15s", &plotter.state.current_color, plotter.state.color_name);
                break;
                
            case CMD_CALIBRATE:
            case CMD_HOME:
            case CMD_G28_HOME:
            case CMD_EMERGENCY_STOP:
            case CMD_M112_EMERGENCY:
            case CMD_MOVE_TO:
            case CMD_G0_RAPID:
            case CMD_G1_LINEAR:
            case CMD_SET_SPEED:
            case CMD_M220_SPEED_FACTOR:
            case CMD_SET_ORIGIN:
            case CMD_RESET_SYSTEM:
            case CMD_PEN_UP:
            case CMD_PEN_DOWN:
            case CMD_SET_PEN:
            case CMD_SET_COLOR:
            case CMD_DRAW_BEGIN: {
                // Command acknowledged - check response
                if (strcmp(data, "OK") == 0) {
                    printf("Command %s started\n", command_names[cmd_id]);
                } else if (strcmp(data, "DONE") == 0) {
                    printf("Command %s completed\n", command_names[cmd_id]);
                    // Обновляем статус команды на FINISHED
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
            // Normal text processing
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
        // Поддержка связи: периодические отправки
        TickType_t now = xTaskGetTickCount();

        // HEARTBEAT раз в секунду
        if ((now - plotter.last_heartbeat_sent) >= pdMS_TO_TICKS(HEARTBEAT_PERIOD_MS)) {
            send_to_stm32_cmd(CMD_HEARTBEAT, NULL);
            plotter.last_heartbeat_sent = now;
        }

        // Запрос статуса раз в секунду
        if ((now - plotter.last_status_request) >= pdMS_TO_TICKS(1000)) {
            send_to_stm32_cmd(CMD_GET_STATUS, NULL);
            plotter.last_status_request = now;
        }

        // Watchdog связи: если давно не было ответов — считаем, что отключились
        TickType_t last_alive = (plotter.last_rx_heartbeat > plotter.last_rx_any)
                                ? plotter.last_rx_heartbeat
                                : plotter.last_rx_any;

        bool prev_connected = plotter.state.is_connected;
        plotter.state.is_connected = (last_alive != 0) &&
                            ((now - last_alive) <= pdMS_TO_TICKS(CONNECT_TIMEOUT_MS));

        // (опционально) логируем только смену состояния
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
            
        case '9':
            printf("KEYPAD: Emergency STOP!\n");
            send_to_stm32_cmd(CMD_EMERGENCY_STOP, NULL);
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
    // Configure READY pin as input with pull-down
    gpio_config_t ready_conf = {
        .pin_bit_mask = (1ULL << STM32_READY_PIN),
        .mode = GPIO_MODE_INPUT,
        .pull_up_en = GPIO_PULLUP_DISABLE,
        .pull_down_en = GPIO_PULLDOWN_ENABLE,
        .intr_type = GPIO_INTR_DISABLE};
    gpio_config(&ready_conf);
    printf("READY pin (GPIO%d) configured as input with pull-down\n", STM32_READY_PIN);

    // Test READY pin
    int ready_state = gpio_get_level(STM32_READY_PIN);
    printf("Initial READY pin state: %d\n", ready_state);

    // Initialize SPI bus
    spi_bus_config_t buscfg = {
        .mosi_io_num = STM32_SPI_MOSI,
        .miso_io_num = STM32_SPI_MISO,
        .sclk_io_num = STM32_SPI_CLK,
        .quadwp_io_num = -1,
        .quadhd_io_num = -1,
        .max_transfer_sz = 2048,
        .flags = SPICOMMON_BUSFLAG_MASTER};

    esp_err_t ret = spi_bus_initialize(STM32_SPI_HOST, &buscfg, SPI_DMA_CH_AUTO);
    if (ret != ESP_OK)
    {
        printf("ERROR: SPI bus init failed: %s\n", esp_err_to_name(ret));
        return;
    }
    printf("SPI bus initialized successfully\n");

    // Add STM32 as SPI device
    spi_device_interface_config_t devcfg = {
        .command_bits = 0,
        .address_bits = 0,
        .mode = 0,                         // SPI mode 0 (CPOL=0, CPHA=0)
        .duty_cycle_pos = 128,             // 50% duty cycle
        .cs_ena_pretrans = 2,              // Setup time before transmission
        .cs_ena_posttrans = 2,             // Hold time after transmission
        .clock_speed_hz = 1 * 1000 * 1000, // 1 MHz (reduced for stability)
        .input_delay_ns = 0,
        .spics_io_num = STM32_SPI_CS,
        .flags = 0,
        .queue_size = 1,
        .pre_cb = NULL,
        .post_cb = NULL};

    ret = spi_bus_add_device(STM32_SPI_HOST, &devcfg, &stm32_spi);
    if (ret != ESP_OK)
    {
        printf("ERROR: Failed to add SPI device: %s\n", esp_err_to_name(ret));
        return;
    }

    printf("STM32 SPI device added successfully\n");
    printf("Configuration:\n");
    printf("  MOSI: GPIO%d\n", STM32_SPI_MOSI);
    printf("  MISO: GPIO%d\n", STM32_SPI_MISO);
    printf("  CLK:  GPIO%d\n", STM32_SPI_CLK);
    printf("  CS:   GPIO%d\n", STM32_SPI_CS);
    printf("  READY: GPIO%d\n", STM32_READY_PIN);
    printf("  Speed: 1 MHz\n");
    printf("  Mode: 0 (CPOL=0, CPHA=0)\n");
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
    
    uart_driver_install(STM32_UART_PORT, 2048, 2048, 0, NULL, 0);
    uart_param_config(STM32_UART_PORT, &uart_config);
    uart_set_pin(STM32_UART_PORT, STM32_TX_PIN, STM32_RX_PIN, UART_PIN_NO_CHANGE, UART_PIN_NO_CHANGE);

    printf("UART2 configured: TX=GPIO%d, RX=GPIO%d\n", STM32_TX_PIN, STM32_RX_PIN);
}

// ============ HAL ROUTINES ============
void plotter_init(void) {
    plotter.last_heartbeat_sent = 0;
    strcpy(plotter.last_command, "None");
    strcpy(plotter.status_text, "Initializing");

    stm32_spi_init();
    uart_init();
}

uint32_t plotter_send_cmd(command_id_t cmd_id, const char* params) {
    return send_to_stm32_cmd(cmd_id, params);
}

uint8_t plotter_is_ready_to_receive_draw_stream_data(void) {
    return is_stm32_ready();
}

void plotter_send_draw_stream_data(const uint8_t* data, uint32_t size) {
    spi_send_nibbles(data, size);
}

void plotter_get_state(plotter_state_t *ps) {
    *ps = plotter.state;
}

void plotter_start_all_tasks(void) {
    xTaskCreate(stm32_uart_task, "stm32_uart", 4096, NULL, 5, NULL);
    xTaskCreate(plotter_control_task, "plotter_control", 4096, NULL, 4, NULL);
    xTaskCreate(keypad_task, "keypad", 4096, NULL, 3, NULL);
}

// idf.py -p /dev/ttyUSB0 -b 115200 flash monitor
//. ~/esp-idf/export.sh

