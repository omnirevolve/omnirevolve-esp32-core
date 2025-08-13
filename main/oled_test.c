#include <stdio.h>
#include <string.h>
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "driver/spi_master.h"
#include "driver/gpio.h"
#include "driver/uart.h"
#include "u8g2.h"
#include "shared/cmd_ids.h"
#include <math.h>

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

static spi_device_handle_t stm32_spi = NULL;

// CRC16-CCITT (0xFFFF, poly 0x1021), как на STM32
static inline uint16_t crc16_ccitt(uint16_t crc, uint8_t b) {
    crc ^= (uint16_t)b << 8;
    for (int i=0;i<8;i++) {
        crc = (crc & 0x8000) ? (uint16_t)((crc<<1) ^ 0x1021) : (uint16_t)(crc<<1);
    }
    return crc;
}

static void spi_send_nibbles(const uint8_t *data, size_t len);

// =============================== TEST STREAMING END 1 ===================================


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

static uint8_t line_nibbles_buffer[2048];  // Буфер для nibbles одной линии

// GPIO пины для клавиатуры (можно изменить под вашу схему)
static const int row_pins[KEYPAD_ROWS] = {32, 33, 27}; // Выходы (строки)
static const int col_pins[KEYPAD_COLS] = {22, 4, 19};  // Входы (столбцы)

// Матрица символов клавиатуры
static const char keypad_keys[KEYPAD_ROWS][KEYPAD_COLS] = {
    {'1', '2', '3'},
    {'4', '5', '6'},
    {'7', '8', '9'}
};

// Состояние клавиатуры
static char last_key = 0;
static uint32_t last_key_time = 0;

static spi_device_handle_t spi;
static u8g2_t u8g2;

typedef struct {
	uint32_t request_id;
	CmdState_t cmd_state;
} CurrentCommand_t;

typedef struct {
    volatile bool is_connected;
    TickType_t last_heartbeat_sent;
    TickType_t last_status_request;
    TickType_t last_rx_any;        // время последнего ЛЮБОГО валидного ответа
    TickType_t last_rx_heartbeat;  // время последнего HEARTBEAT от STM32

    bool is_calibrated;
    bool is_homed;
    bool is_processing_cmd;
    bool is_idle;
    
    int32_t x_pos;  // Changed to int32_t to match STM32
    int32_t y_pos;
    float x_max;
    float y_max;
    
    // Extended state
    bool pen_is_down;
    uint8_t current_color;
    char color_name[16];
    uint16_t feedrate;
    float speed_factor;
    
    bool heartbeat_ok;
    char status_text[64];
    char last_command[64];  // Increased from 32
    char last_response[64];
    volatile CurrentCommand_t current_cmd_status;
} plotter_state_t;


static plotter_state_t plotter = {
    .is_connected = false,
    .last_heartbeat_sent = 0,
    .last_status_request = 0,
    .last_rx_any = 0,
    .last_rx_heartbeat = 0,
    .is_calibrated = false,
    .is_homed = false,
    .is_processing_cmd = false,
    .is_idle = false,
    .x_pos = -1,
    .y_pos = -1,
    .x_max = 0,
    .y_max = 0,
    .pen_is_down = false,
    .current_color = 0,
    .feedrate = 3000,
    .speed_factor = 1.0,
    .heartbeat_ok = false
};

static uint32_t request_counter = 1;
static SemaphoreHandle_t display_mutex;

typedef struct {
    uint32_t f_hz;       // sample rate, e.g. 10000
    uint16_t step_us;    // STEP high width
    uint16_t guard_us;   // low guard after high
    uint8_t  inv_mask;   // bit0=X_DIR invert, bit1=Y_DIR invert
} __attribute__((packed)) stream_meta_t;


uint32_t send_to_stm32_cmd(command_id_t cmd_id, const char* params);
static bool wait_command_completion(uint32_t request_id, const char* step_name, uint32_t timeout_ms);

// =============================== TEST STREAMING BEGIN 1 ===================================
// =============================== TEST STREAMING END 1 ===================================

// =============================== TEST STREAMING BEGIN 2 ===================================
// =============================== TEST STREAMING END 2 ===================================

// =============================== TEST STREAMING BEGIN 2 ===================================
// size_t generate_line_nibbles(float x0_mm, float y0_mm, float x1_mm, float y1_mm, float speed_mm_s);

size_t generate_line_nibbles(float x0_mm, float y0_mm, float x1_mm, float y1_mm, float speed_mm_s);
static inline bool is_stm32_ready(void) {
    return gpio_get_level(STM32_READY_PIN) == 1;
}

void send_line(float x0_mm, float y0_mm, float x1_mm, float y1_mm, float speed_mm_s)
{
    uint32_t req_id = send_to_stm32_cmd(CMD_DRAW_BEGIN, NULL);
    vTaskDelay(pdMS_TO_TICKS(500));
    
    int wait_count = 0;
    while (!is_stm32_ready() && wait_count < 50) {
        vTaskDelay(pdMS_TO_TICKS(10));
        wait_count++;
    }
    
    if (!is_stm32_ready()) {
        printf("ERROR: STM32 not ready for streaming\n");
        return;
    }

    size_t nibbles_count = generate_line_nibbles(
        x0_mm, y0_mm, x1_mm, y1_mm, speed_mm_s);

    if (nibbles_count > 0)
    {
        size_t bytes_to_send = SPI_CHUNK_SIZE; // Всегда 2048
        memset(&line_nibbles_buffer[nibbles_count/2], 0, bytes_to_send - nibbles_count/2);
        spi_send_nibbles(line_nibbles_buffer, bytes_to_send);
        vTaskDelay(pdMS_TO_TICKS(500));
        send_to_stm32_cmd(CMD_DRAW_FINISH_WHEN_EMPTY, NULL);        
    }
    else
    {
        printf("WARNING: No nibbles generated for test line\n");
    }
}

void debug_print_nibbles(const uint8_t *data, size_t byte_count, const char *label)
{
    printf("DEBUG %s: %zu bytes = %zu nibbles\n", label, byte_count, byte_count * 2);
    // Печатаем первые несколько байт для анализа
    size_t print_count = (byte_count > 10) ? 10 : byte_count;
    printf("  First bytes (hex): ");
    for (size_t i = 0; i < print_count; i++)
    {
        printf("%02X ", data[i]);
    }
    printf("\n");

    // Декодируем первые nibbles
    printf("  First nibbles decoded:\n");
    for (size_t i = 0; i < print_count && i < 5; i++)
    {
        uint8_t low_nib = data[i] & 0x0F;
        uint8_t high_nib = (data[i] >> 4) & 0x0F;

        printf("    [%zu] Low: X%s%s Y%s%s | High: X%s%s Y%s%s\n", i,
               (low_nib & NIBBLE_X_DIR) ? "+" : "-",
               (low_nib & NIBBLE_X_STEP) ? "STEP" : "----",
               (low_nib & NIBBLE_Y_DIR) ? "+" : "-",
               (low_nib & NIBBLE_Y_STEP) ? "STEP" : "----",
               (high_nib & NIBBLE_X_DIR) ? "+" : "-",
               (high_nib & NIBBLE_X_STEP) ? "STEP" : "----",
               (high_nib & NIBBLE_Y_DIR) ? "+" : "-",
               (high_nib & NIBBLE_Y_STEP) ? "STEP" : "----");
    }
}

size_t generate_line_nibbles(float x0_mm, float y0_mm, float x1_mm, float y1_mm, float speed_mm_s)
{
    // Конвертируем в шаги
    int32_t x0_steps = (int32_t)(x0_mm * STEPS_PER_MM);
    int32_t y0_steps = (int32_t)(y0_mm * STEPS_PER_MM);
    int32_t x1_steps = (int32_t)(x1_mm * STEPS_PER_MM);
    int32_t y1_steps = (int32_t)(y1_mm * STEPS_PER_MM);
    
    int32_t dx = x1_steps - x0_steps;
    int32_t dy = y1_steps - y0_steps;
    
    // Направления
    uint8_t x_dir = (dx >= 0) ? 1 : 0;
    uint8_t y_dir = (dy >= 0) ? 1 : 0;
    
    uint32_t ax = abs(dx);
    uint32_t ay = abs(dy);
    
    // Расчет времени и тиков
    float distance_mm = sqrtf((float)(dx * dx + dy * dy)) / STEPS_PER_MM;
    float time_s = distance_mm / speed_mm_s;
    uint32_t ticks_total = (uint32_t)(time_s * STREAM_F_HZ);
    
    // Минимум один тик на шаг
    uint32_t max_steps = (ax > ay) ? ax : ay;
    if (ticks_total < max_steps) {
        ticks_total = max_steps;
    }
    
    printf("Generating line: %lu x-steps, %lu y-steps in %lu ticks\n", 
           ax, ay, ticks_total);
    
    // Классический Bresenham с временной интерполяцией
    memset(line_nibbles_buffer, 0, sizeof(line_nibbles_buffer));
    size_t nibble_count = 0;
    
    // Счетчики прогресса
    float x_progress = 0;
    float y_progress = 0;
    float x_increment = (float)ax / ticks_total;
    float y_increment = (float)ay / ticks_total;
    uint32_t x_steps_done = 0;
    uint32_t y_steps_done = 0;
    
    for (uint32_t tick = 0; tick < ticks_total && nibble_count < sizeof(line_nibbles_buffer) * 2; tick++) {
        uint8_t nib = 0;
        
        // Направления всегда устанавливаем
        if (x_dir) nib |= NIBBLE_X_DIR;
        if (y_dir) nib |= NIBBLE_Y_DIR;
        
        // Накапливаем прогресс
        x_progress += x_increment;
        y_progress += y_increment;
        
        // Генерируем шаг X если накопилось
        if (x_progress >= 1.0f && x_steps_done < ax) {
            nib |= NIBBLE_X_STEP;
            x_progress -= 1.0f;
            x_steps_done++;
        }
        
        // Генерируем шаг Y если накопилось
        if (y_progress >= 1.0f && y_steps_done < ay) {
            nib |= NIBBLE_Y_STEP;
            y_progress -= 1.0f;
            y_steps_done++;
        }
        
        // Сохраняем nibble
        if (nibble_count % 2 == 0) {
            line_nibbles_buffer[nibble_count / 2] = nib;
        } else {
            line_nibbles_buffer[nibble_count / 2] |= (nib << 4);
        }
        nibble_count++;
    }
    
    printf("Generated %zu nibbles: X_steps=%lu/%lu, Y_steps=%lu/%lu\n",
           nibble_count, x_steps_done, ax, y_steps_done, ay);
    
    return nibble_count;
}


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
            if (key != last_key || 
                (current_time - last_key_time) > pdMS_TO_TICKS(200)) {
                
                process_keypad_command(key);
                last_key = key;
                last_key_time = current_time;
            }
        } else {
            last_key = 0; // Кнопка отпущена
        }
        
        vTaskDelay(pdMS_TO_TICKS(50)); // Сканируем каждые 50мс
    }
}

uint32_t send_to_stm32_cmd(command_id_t cmd_id, const char* params);

// Display management
void update_display() {
    if (xSemaphoreTake(display_mutex, pdMS_TO_TICKS(100)) == pdTRUE) {
        u8g2_ClearBuffer(&u8g2);

        // Line 1: Connection status
        u8g2_SetFont(&u8g2, u8g2_font_6x10_tf);
        char conn_str[32];
        sprintf(conn_str, "STM:%s HB:%s", 
            plotter.is_connected ? "OK" : "NO",
            plotter.heartbeat_ok ? "OK" : "LOST");
        u8g2_DrawStr(&u8g2, 0, 10, conn_str);

        // Line 2: Position  
        char pos_str[64];
        if (plotter.x_pos < 0 || plotter.y_pos < 0) {
            strcpy(pos_str, "Pos: X? Y?");
        } else {
            sprintf(pos_str, "Pos: X%ld Y%ld", plotter.x_pos, plotter.y_pos);
        }
        u8g2_DrawStr(&u8g2, 0, 20, pos_str);

        // Line 3: State flags - ИСПРАВЛЕНО
        char state_str[64];
        sprintf(state_str, "H:%d C:%d P:%s Col:%d", 
            plotter.is_homed ? 1 : 0,
            plotter.is_calibrated ? 1 : 0,
            plotter.pen_is_down ? "DN" : "UP",
            plotter.current_color);
        u8g2_DrawStr(&u8g2, 0, 30, state_str);

        // Line 4: Current status
        if (plotter.is_processing_cmd) {
            strcpy(state_str, "Status: BUSY");
        } else if (!plotter.is_calibrated) {
            strcpy(state_str, "Status: UNCALIBRATED");
        } else if (!plotter.is_homed) {
            strcpy(state_str, "Status: NEED HOME");
        } else if (plotter.is_idle) {
            strcpy(state_str, "Status: READY");
        } else {
            strcpy(state_str, "Status: UNKNOWN");
        }
        u8g2_DrawStr(&u8g2, 0, 40, state_str);

        // Line 5: Last command
        char cmd_str[64];
        snprintf(cmd_str, sizeof(cmd_str), "Cmd: %.25s", plotter.last_command);
        u8g2_DrawStr(&u8g2, 0, 50, cmd_str);

        u8g2_SendBuffer(&u8g2);
        xSemaphoreGive(display_mutex);
    }
}

// STM32 Communication
uint32_t send_to_stm32_cmd(command_id_t cmd_id, const char* params) {
    char full_msg[128];
    
    // Добавить проверку
    if (cmd_id >= CMD_COUNT) {
        printf("ERROR: Invalid cmd_id %d\n", cmd_id);
        return 0;
    }
    
    if (params && strlen(params) > 0) {
        snprintf(full_msg, sizeof(full_msg), "%lu:%d:%s\n", request_counter, cmd_id, params);
    } else {
        snprintf(full_msg, sizeof(full_msg), "%lu:%d\n", request_counter, cmd_id);
    }
    if (cmd_id < CMD_DUMMY_ACTIVE_COMMANDS_DELIMITER)
    {
        plotter.current_cmd_status.request_id = request_counter;
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

    return request_counter++;
}

void process_stm32_response(const char* response) {
    uint32_t received_id;
    uint32_t cmd_id;
    char data[256] = {0};
    
    int parsed = sscanf(response, "%lu:%lu:%255s", &received_id, &cmd_id, data);
    
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
                    plotter.is_calibrated = (data[0] == '1');
                    plotter.is_homed = (data[1] == '1');
                    plotter.is_processing_cmd = (data[2] == '1');
                    plotter.is_idle = (data[3] == '1');
                    printf("Status updated: cal=%d, home=%d, proc=%d, idle=%d\n",
                           plotter.is_calibrated, plotter.is_homed,
                           plotter.is_processing_cmd, plotter.is_idle);
                }
                break;
                
            case CMD_GET_CMD_STATUS:
            {
                uint32_t request_id = 0;
                int cmd_state_int = CMD_NOT_PRESENTED;
                
                if (sscanf(data, "%lu:%d", &request_id, &cmd_state_int) == 2) {
                    CmdState_t cmd_state = (CmdState_t)cmd_state_int;
                    
                    // Обновляем статус только если это наша команда
                    if (plotter.current_cmd_status.request_id == request_id) {
                        if (plotter.current_cmd_status.cmd_state != cmd_state) {
                            printf("Current command %lu status changed: %d -> %d\n", 
                                request_id, plotter.current_cmd_status.cmd_state, cmd_state);
                            plotter.current_cmd_status.cmd_state = cmd_state;
                        }
                    } else if (request_id != 0) {
                        // Это другая команда - обновляем
                        printf("Received status for command %lu: %d\n", request_id, cmd_state);
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
                plotter.x_pos = -1;
                plotter.y_pos = -1;
            } else {
                int32_t x_steps, y_steps;
                sscanf(data, "X%ld:Y%ld", &x_steps, &y_steps);
                plotter.x_pos = x_steps;
                plotter.y_pos = y_steps;
            }
            break;
            
            case CMD_GET_LIMITS:
                {
                    float x_mm, y_mm;
                    if (sscanf(data, "X%f,Y%f", &x_mm, &y_mm) == 2) {
                        plotter.x_max = x_mm;
                        plotter.y_max = y_mm;
                        printf("Limits received: X=%.2f mm, Y=%.2f mm\n", x_mm, y_mm);
                    } else {
                        printf("Failed to parse limits from: %s\n", data);
                    }
                }
                break;
                    
            case CMD_GET_DIAGNOSTICS: {
                int pen_state;
                sscanf(data, "PEN:%d,FEED:%hu,SF:%f", 
                    &pen_state, &plotter.feedrate, &plotter.speed_factor);
                plotter.pen_is_down = (pen_state == 1);
                break;
            }
                
            case CMD_GET_COLOR:
                sscanf(data, "C%hhu:%15s", &plotter.current_color, plotter.color_name);
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
                printf("Unknown cmd_id: %lu\n", cmd_id);
                break;
        }
    }
    else {
    }
    plotter.is_connected = true;
}

static void stm32_uart_task(void *pvParameters) {
    uint8_t data;
    char buffer[UART_BUFFER_SIZE];
    uint8_t pos = 0;

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
// ================================ STREAMING FNCs END =============================

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

        bool prev_connected = plotter.is_connected;
        plotter.is_connected = (last_alive != 0) &&
                            ((now - last_alive) <= pdMS_TO_TICKS(CONNECT_TIMEOUT_MS));

        // (опционально) логируем только смену состояния
        if (prev_connected != plotter.is_connected) {
            printf("[link] is_connected = %s\n", plotter.is_connected ? "true" : "false");
        }

        vTaskDelay(pdMS_TO_TICKS(50));    
    }
}

// Test movement task (optional - for testing)
void test_movement_task(void *pvParameters) {
    
    while (1) {
        if (plotter.is_homed && plotter.is_idle && !plotter.is_processing_cmd) {
            // Example: draw a square
            static int step = 0;
            char params[32];
            
            switch(step) {
                case 0:
                    sprintf(params, "X%d,Y%d", 10, 10);
                    send_to_stm32_cmd(CMD_G0_RAPID, params);
                    break;
                case 1:
                    sprintf(params, "X%d,Y%d", 50, 10);
                    send_to_stm32_cmd(CMD_G1_LINEAR, params);
                    break;
                case 2:
                    sprintf(params, "X%d,Y%d", 50, 50);
                    send_to_stm32_cmd(CMD_G1_LINEAR, params);
                    break;
                case 3:
                    sprintf(params, "X%d,Y%d", 10, 50);
                    send_to_stm32_cmd(CMD_G1_LINEAR, params);
                    break;
                case 4:
                    sprintf(params, "X%d,Y%d", 10, 10);
                    send_to_stm32_cmd(CMD_G1_LINEAR, params);
                    break;
            }
            
            step = (step + 1) % 5;
            vTaskDelay(pdMS_TO_TICKS(5000)); // Wait 5 seconds between moves
        }
        
        vTaskDelay(pdMS_TO_TICKS(1000));
    }
}

// Display hardware callbacks
uint8_t u8x8_byte_hw_spi(u8x8_t *u8x8, uint8_t msg, uint8_t arg_int, void *arg_ptr) {
    switch(msg) {
        case U8X8_MSG_BYTE_SET_DC:
            gpio_set_level(PIN_NUM_DC, arg_int);
            break;
        case U8X8_MSG_BYTE_SEND:;
            spi_transaction_t t = {
                .length = 8 * arg_int,
                .tx_buffer = arg_ptr
            };
            spi_device_transmit(spi, &t);
            break;
    }
    return 1;
}

uint8_t u8x8_gpio_and_delay(u8x8_t *u8x8, uint8_t msg, uint8_t arg_int, void *arg_ptr) {
    switch(msg) {
        case U8X8_MSG_GPIO_AND_DELAY_INIT:
            gpio_set_direction(PIN_NUM_DC, GPIO_MODE_OUTPUT);
            gpio_set_direction(PIN_NUM_RST, GPIO_MODE_OUTPUT);
            break;
        case U8X8_MSG_DELAY_MILLI:
            vTaskDelay(pdMS_TO_TICKS(arg_int));
            break;
        case U8X8_MSG_GPIO_RESET:
            gpio_set_level(PIN_NUM_RST, arg_int);
            break;
    }
    return 1;
}

static bool wait_command_completion(uint32_t request_id, const char* step_name, uint32_t timeout_ms) {
    printf("CIRCLES: %s - waiting for cmd %lu...\n", step_name, request_id);
    uint32_t start = xTaskGetTickCount();
    
    plotter.current_cmd_status.request_id = request_id;
    plotter.current_cmd_status.cmd_state = CMD_NOT_PRESENTED;
    
    char buf[64];
    sprintf(buf, "%lu", request_id);
    
    // Даем минимальное время команде начать выполнение
    vTaskDelay(pdMS_TO_TICKS(20));  // Сократили со 100 до 20
    
    while ((xTaskGetTickCount() - start) < pdMS_TO_TICKS(timeout_ms)) {
        send_to_stm32_cmd(CMD_GET_CMD_STATUS, buf);
        vTaskDelay(pdMS_TO_TICKS(50));  // Сократили со 100 до 50 - чаще опрашиваем
        
        if (plotter.current_cmd_status.request_id == request_id) {
            if (plotter.current_cmd_status.cmd_state == CMD_FINISHED) {
                printf("CIRCLES: %s - completed\n", step_name);
                return true;
            } else if (plotter.current_cmd_status.cmd_state == CMD_FAILED) {
                printf("CIRCLES: %s - failed\n", step_name);
                return false;
            }
        }
    }
    
    printf("CIRCLES: ERROR - %s timeout!\n", step_name);
    return false;
}

// ============ Send nibbles via SPI ============

static void spi_send_nibbles(const uint8_t *data, size_t len)
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

// ============ NIBBLES GEN FOR LINE ============

// size_t generate_line_nibbles(float x0_mm, float y0_mm, float x1_mm, float y1_mm, float speed_mm_s) {
//     // Конвертировать в шаги
//     int32_t x0_steps = (int32_t)(x0_mm * STEPS_PER_MM);
//     int32_t y0_steps = (int32_t)(y0_mm * STEPS_PER_MM);
//     int32_t x1_steps = (int32_t)(x1_mm * STEPS_PER_MM);
//     int32_t y1_steps = (int32_t)(y1_mm * STEPS_PER_MM);
    
//     int32_t dx = x1_steps - x0_steps;
//     int32_t dy = y1_steps - y0_steps;
    
//     if (dx == 0 && dy == 0) return 0;
    
//     // Определить направления
//     uint8_t x_dir = (dx >= 0) ? 1 : 0;
//     uint8_t y_dir = (dy >= 0) ? 1 : 0;
    
//     uint32_t ax = abs(dx);
//     uint32_t ay = abs(dy);
//     uint32_t steps_total = (ax > ay) ? ax : ay;
    
//     // Рассчитать timing
//     float distance_mm = sqrtf(dx*dx + dy*dy) / STEPS_PER_MM;
//     float time_s = distance_mm / speed_mm_s;
//     uint32_t ticks_total = (uint32_t)(time_s * STREAM_F_HZ);
//     if (ticks_total < steps_total) ticks_total = steps_total;
    
//     // Bresenham с timing
//     int32_t err_x = 0, err_y = 0;
//     uint32_t steps_done = 0;
//     size_t nibble_count = 0;
//     uint32_t tick_interval = ticks_total / steps_total;
//     if (tick_interval < 1) tick_interval = 1;
    
//     for (uint32_t tick = 0; tick < ticks_total && nibble_count < sizeof(line_nibbles_buffer)*2; tick++) {
//         uint8_t nib = 0;
        
//         // Установить биты направления
//         if (x_dir) nib |= (1<<1);  // X_DIR
//         if (y_dir) nib |= (1<<3);  // Y_DIR
        
//         // Проверить нужность шагов (только каждый tick_interval тиков)
//         if ((tick % tick_interval == 0) && steps_done < steps_total) {
//             err_x += ax;
//             if (err_x >= steps_total) {
//                 err_x -= steps_total;
//                 nib |= (1<<0);  // X_STEP
//             }
            
//             err_y += ay;
//             if (err_y >= steps_total) {
//                 err_y -= steps_total;
//                 nib |= (1<<2);  // Y_STEP
//             }
            
//             if (nib & ((1<<0) | (1<<2))) {  // Если есть хотя бы один шаг
//                 steps_done++;
//             }
//         }
        
//         // Упаковать nibble
//         if (nibble_count % 2 == 0) {
//             line_nibbles_buffer[nibble_count/2] = nib;
//         } else {
//             line_nibbles_buffer[nibble_count/2] |= (nib << 4);
//         }
//         nibble_count++;
//     }
    
//     printf("LINE: Generated %zu nibbles for %lu steps\n", nibble_count, steps_done);
//     return nibble_count;
// }
// ============ KEYPAD HANDLING ============

void process_keypad_command(char key) {
    printf("\n=== KEYPAD: Key '%c' pressed ===\n", key);
    
    char params[32];
    
    switch(key) {
        case '1':
            printf("KEYPAD: Starting calibration\n");
            send_to_stm32_cmd(CMD_CALIBRATE, NULL);
            break;
            
        case '2':
            printf("KEYPAD: Starting homing\n");
            send_to_stm32_cmd(CMD_HOME, NULL);
            break;
            
        case '3':  // УПРОЩЕННЫЙ тест SPI без пера
            plotter.x_max = 15;
            plotter.y_max = 15;
            printf("KEYPAD: Testing SPI circle draw (no pen)\n");
            if (plotter.is_calibrated && plotter.is_homed) {
                // Сразу рисовать круг БЕЗ команд пера
                if (plotter.x_max > 0 && plotter.y_max > 0) {
                    float cx = plotter.x_max / 2.0f;
                    float cy = plotter.y_max / 2.0f;
                    float radius = fminf(plotter.x_max, 
                                       plotter.y_max) / 6.0f;
                    
                    printf("KEYPAD: Drawing circle at (%.1f,%.1f) r=%.1f\n", cx, cy, radius);
                    send_line(0, 0, 5, 5, 30);
                    //draw_circle_spi(cx, cy, radius, 30.0f);
                } else {
                    printf("KEYPAD: Invalid plotter limits\n");
                }
            } else {
                printf("KEYPAD: Need calibration and homing first!\n");
            }
            break;

        case '4':  // Простой тест - движение по квадрату
            printf("KEYPAD: Testing simple square movement\n");
            if (plotter.is_calibrated && plotter.is_homed) {
                // Простое движение по квадрату через G-код
                sprintf(params, "X%d,Y%d", 50, 50);
                send_to_stm32_cmd(CMD_G1_LINEAR, params);
            }
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

// ============ SPI Initialization ============

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

void app_main() {
    printf("ESP32 Plotter Bridge Starting\n");
    
    // Create display mutex
    display_mutex = xSemaphoreCreateMutex();
    
    // Initialize state
    plotter.last_heartbeat_sent = 0;
    strcpy(plotter.last_command, "None");
    strcpy(plotter.status_text, "Initializing");
    
    // Initialize SPI for OLED
    spi_bus_config_t buscfg = {
        .mosi_io_num = PIN_NUM_MOSI,
        .miso_io_num = -1,
        .sclk_io_num = PIN_NUM_CLK,
        .quadwp_io_num = -1,
        .quadhd_io_num = -1,
        .max_transfer_sz = 1024
    };
    
    spi_device_interface_config_t devcfg = {
        .clock_speed_hz = 1000000,
        .mode = 0,
        .spics_io_num = PIN_NUM_CS,
        .queue_size = 7
    };
    
    stm32_spi_init();

    printf("init spi2_host for OLED");
    // Initialize OLED display
    // u8g2_Setup_ssd1309_128x64_noname0_f(&u8g2, U8G2_R0, u8x8_byte_hw_spi, u8x8_gpio_and_delay);
    // u8g2_InitDisplay(&u8g2);
    // u8g2_SetPowerSave(&u8g2, 0);
    
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

    
    // Start tasks with proper priorities
    xTaskCreate(stm32_uart_task, "stm32_uart", 4096, NULL, 5, NULL);
    xTaskCreate(plotter_control_task, "plotter_control", 4096, NULL, 4, NULL);
    xTaskCreate(keypad_task, "keypad", 4096, NULL, 3, NULL);
    // Optional: enable test movement task
    
    // Main display loop - reduced delay for more responsive display
    while (1) {
        //update_display();
        vTaskDelay(pdMS_TO_TICKS(100));
    }
}

// idf.py -p /dev/ttyUSB0 -b 115200 flash monitor
//. ~/esp-idf/export.sh
