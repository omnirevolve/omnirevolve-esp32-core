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

// =============================== TEST STREAMING BEGIN 1 ===================================
// --- Streaming protocol (must match STM32) ---
#define STREAM_SYNC0      0xA5
#define STREAM_SYNC1      0x5A
#define STREAM_CMD_BEGIN  0x10
#define STREAM_CMD_DATA   0x11
#define STREAM_CMD_START  0x12
#define STREAM_CMD_ABORT  0x13

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

// CRC16-CCITT (0xFFFF, poly 0x1021), как на STM32
static inline uint16_t crc16_ccitt(uint16_t crc, uint8_t b) {
    crc ^= (uint16_t)b << 8;
    for (int i=0;i<8;i++) {
        crc = (crc & 0x8000) ? (uint16_t)((crc<<1) ^ 0x1021) : (uint16_t)(crc<<1);
    }
    return crc;
}
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

#define STREAM_PKT_BYTES 1024
static uint8_t stream_pkt[STREAM_PKT_BYTES];

// GPIO пины для клавиатуры (можно изменить под вашу схему)
static const int row_pins[KEYPAD_ROWS] = {32, 33, 27, 2}; // Выходы (строки)
static const int col_pins[KEYPAD_COLS] = {12, 13, 15, 4};  // Входы (столбцы)

// Матрица символов клавиатуры
static const char keypad_keys[KEYPAD_ROWS][KEYPAD_COLS] = {
    {'1', '2', '3', 'A'},
    {'4', '5', '6', 'B'},
    {'7', '8', '9', 'C'},
    {'*', '0', '#', 'D'}
};

// Состояние клавиатуры
static char last_key = 0;
static uint32_t last_key_time = 0;

static spi_device_handle_t spi;
static u8g2_t u8g2;

typedef struct {
    uint8_t active;
    uint8_t *buffer;
    size_t buffer_size;
    size_t write_pos;
    size_t read_pos;
    size_t total_sent;
    size_t total_generated;
    uint8_t generation_complete;
    TaskHandle_t generator_task;
    TaskHandle_t sender_task;
    SemaphoreHandle_t buffer_mutex;
    SemaphoreHandle_t data_ready_sem;
} continuous_stream_t;

static continuous_stream_t cont_stream = {0};

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

#define STREAM_SYNC0      0xA5
#define STREAM_SYNC1      0x5A
#define STREAM_CMD_BEGIN  0x10  // meta: f_hz, step_us, guard_us, inv_mask
#define STREAM_CMD_DATA   0x11  // payload: packed nibbles (2 ticks per byte)
#define STREAM_CMD_START  0x12  // start playback
#define STREAM_CMD_ABORT  0x13  // abort playback

typedef struct {
    uint32_t f_hz;       // sample rate, e.g. 10000
    uint16_t step_us;    // STEP high width
    uint16_t guard_us;   // low guard after high
    uint8_t  inv_mask;   // bit0=X_DIR invert, bit1=Y_DIR invert
} __attribute__((packed)) stream_meta_t;


uint32_t send_to_stm32_cmd(command_id_t cmd_id, const char* params);
static bool wait_command_completion(uint32_t request_id, const char* step_name, uint32_t timeout_ms);

// =============================== TEST STREAMING BEGIN 1 ===================================
// --- Low-level: отправка одного бинарного кадра STREAM_* ---
static void stream_send_frame(uint8_t cmd, const uint8_t *payload, uint16_t len)
{
    uint8_t hdr[5];
    hdr[0] = STREAM_SYNC0;
    hdr[1] = STREAM_SYNC1;
    hdr[2] = cmd;
    hdr[3] = (uint8_t)(len & 0xFF);
    hdr[4] = (uint8_t)(len >> 8);

    // CRC только по payload (как на STM32)
    uint16_t crc = 0xFFFF;
    for (uint16_t i=0;i<len;i++) crc = crc16_ccitt(crc, payload[i]);

    uint8_t tail[2];
    tail[0] = (uint8_t)(crc >> 8);   // CRC hi
    tail[1] = (uint8_t)(crc & 0xFF); // CRC lo

    uart_write_bytes(STM32_UART_PORT, (const char*)hdr, sizeof(hdr));
    if (len) uart_write_bytes(STM32_UART_PORT, (const char*)payload, len);
    uart_write_bytes(STM32_UART_PORT, (const char*)tail, sizeof(tail));
    // без \n — это бинарь
}

static inline void stream_begin(uint32_t f_hz, uint16_t step_us, uint16_t guard_us, uint8_t inv_mask)
{
    uint8_t pl[9];
    pl[0] = (uint8_t)(f_hz & 0xFF);
    pl[1] = (uint8_t)((f_hz >> 8) & 0xFF);
    pl[2] = (uint8_t)((f_hz >> 16) & 0xFF);
    pl[3] = (uint8_t)((f_hz >> 24) & 0xFF);
    pl[4] = (uint8_t)(step_us & 0xFF);
    pl[5] = (uint8_t)(step_us >> 8);
    pl[6] = (uint8_t)(guard_us & 0xFF);
    pl[7] = (uint8_t)(guard_us >> 8);
    pl[8] = inv_mask;
    stream_send_frame(STREAM_CMD_BEGIN, pl, sizeof(pl));
}

static inline void stream_data(const uint8_t *buf, uint16_t len)
{
    if (len) stream_send_frame(STREAM_CMD_DATA, buf, len);
}

static inline void stream_start(void)
{
    stream_send_frame(STREAM_CMD_START, NULL, 0);
}

static inline void stream_abort(void)
{
    stream_send_frame(STREAM_CMD_ABORT, NULL, 0);
}
// =============================== TEST STREAMING END 1 ===================================

// =============================== TEST STREAMING BEGIN 2 ===================================
// Собирает байт тика: low nibble = STEP/DIR; high nibble = те же STEP-биты (опустить импульс)
static inline uint8_t make_tick_byte(uint8_t x_step, uint8_t x_dir_pos, uint8_t y_step, uint8_t y_dir_pos)
{
    uint8_t low = 0;
    if (x_step) low |= 0x01;
    if (x_dir_pos) low |= 0x02;
    if (y_step) low |= 0x04;
    if (y_dir_pos) low |= 0x08;

    uint8_t high = 0;
    if (x_step) high |= 0x10;  // X_STEP в старшем полубайте
    if (y_step) high |= 0x40;  // Y_STEP в старшем полубайте (bit6)

    return (uint8_t)(low | high);
}

// Пейсинг для недопущения переполнения кольца на STM32
static inline void paced_delay_for_bytes(uint32_t bytes_sent_after_start)
{
    // реальная полезная пропускная = STREAM_BYTES_PER_SEC * 0.98
    const uint32_t bps = (STREAM_BYTES_PER_SEC * STREAM_RATE_SAFETY_PC) / 100; // ~4900 Б/с
    // мы ждём после каждого кадра на CHUNK байт ~ CHUNK/bps секунд:
    // делать здесь нечего; задержка делается в месте отправки кадра
    (void)bytes_sent_after_start;
}

// Генерация круга, отправка DATA, START и пейсинг
static void stream_generate_circle_and_send(float cx_mm, float cy_mm, float R_mm, float speed_mm_s)
{
    // 0) Подготовка параметров
    const float spm = STEPS_PER_MM;
    const float two_pi = 6.28318530717958647692f;
    const float L_mm = two_pi * R_mm;               // длина окружности
    const float T_s  = (speed_mm_s > 0.1f) ? (L_mm / speed_mm_s) : 0.0f;
    const uint32_t N = (uint32_t)ceilf(T_s * (float)STREAM_F_HZ); // кол-во сэмплов (байтов)

    if (N == 0) return;

    // 1) BEGIN
    stream_begin(STREAM_F_HZ, STREAM_STEP_US, STREAM_GUARD_US, STREAM_INV_MASK);

    // 2) Генерим данные и отправляем: часть ДО START (preload), остальное — c пейсингом
    uint8_t buf[STREAM_DATA_CHUNK];
    uint32_t in_buf = 0;
    uint32_t sent_total = 0;

    // приращение угла за тик: omega = v/R; dtheta = omega / f
    const float dtheta = (R_mm > 0.001f) ? ((speed_mm_s / R_mm) / (float)STREAM_F_HZ) : 0.0f;

    float theta = 0.0f;
    float acc_x = 0.0f, acc_y = 0.0f;  // накопители дробных шагов (в шагах)
    // шаги/тик в проекции: dx = -R*sin(theta)*dtheta; dy = R*cos(theta)*dtheta
    for (uint32_t i=0; i<N; i++) {
        float dx_mm = -R_mm * sinf(theta) * dtheta;
        float dy_mm =  R_mm * cosf(theta) * dtheta;

        acc_x += dx_mm * spm;
        acc_y += dy_mm * spm;

        uint8_t x_step=0, x_dir_pos=0, y_step=0, y_dir_pos=0;

        if (acc_x >= 1.0f) { x_step=1; x_dir_pos=1; acc_x -= 1.0f; }
        else if (acc_x <= -1.0f) { x_step=1; x_dir_pos=0; acc_x += 1.0f; }

        if (acc_y >= 1.0f) { y_step=1; y_dir_pos=1; acc_y -= 1.0f; }
        else if (acc_y <= -1.0f) { y_step=1; y_dir_pos=0; acc_y += 1.0f; }

        buf[in_buf++] = make_tick_byte(x_step, x_dir_pos, y_step, y_dir_pos);

        if (in_buf == STREAM_DATA_CHUNK) {
            stream_data(buf, (uint16_t)in_buf);
            sent_total += in_buf;
            in_buf = 0;

            // Если уже отправили START — держим среднюю скорость отправки ~ потреблению (чтобы не забить 4К-буфер STM32)
            if (sent_total > STREAM_PRELOAD_BYTES) {
                // 500 байт при ~4900 Б/с => ~102 мс
                uint32_t ms = (STREAM_DATA_CHUNK * 1000u) / ((STREAM_BYTES_PER_SEC * STREAM_RATE_SAFETY_PC)/100u);
                vTaskDelay(pdMS_TO_TICKS(ms));
            }
        }

        theta += dtheta;
        if (theta >= two_pi) theta -= two_pi; // не обязательно, но полезно для численной устойчивости

        // Отправляем START ровно один раз, когда накопили preload
        if (sent_total >= STREAM_PRELOAD_BYTES && sent_total - in_buf < STREAM_PRELOAD_BYTES) {
            // START будет вызван после первой отправки, которая преодолеет порог preload;
            // фактически — при следующем stream_data (см. выше)
        }
    }

    // добросить «хвост»
    if (in_buf) {
        stream_data(buf, (uint16_t)in_buf);
        sent_total += in_buf;
        in_buf = 0;
    }

    // 3) Теперь жмём START (если ещё не жали): простой способ — нажать после первой отправки >0 байт
    // Чтобы начать воспроизведение сразу после прелоада — жмём START здесь, если sent_total >= preload
    stream_start();

    // 4) Досылаем остаток с пейсингом уже отправили выше; здесь остаётся просто подождать теоретическое время T_s
    // плюс небольшой запас, чтобы STM32 доиграл последние байты
    uint32_t wait_ms = (uint32_t)(T_s * 1000.0f) + 150;
    vTaskDelay(pdMS_TO_TICKS(wait_ms));
}
// =============================== TEST STREAMING END 2 ===================================

// =============================== TEST STREAMING BEGIN 2 ===================================
static void stream_circle_task(void *pv)
{
    // 1) Получить лимиты (если ещё не знаем)
    if (plotter.x_max == 0.0f || plotter.y_max == 0.0f) {
        send_to_stm32_cmd(CMD_GET_LIMITS, NULL);
        uint32_t t0 = xTaskGetTickCount();
        while ((plotter.x_max == 0.0f || plotter.y_max == 0.0f) &&
               (xTaskGetTickCount() - t0) < pdMS_TO_TICKS(3000)) {
            vTaskDelay(pdMS_TO_TICKS(50));
        }
        if (plotter.x_max == 0.0f || plotter.y_max == 0.0f) {
            printf("STREAM: no limits -> abort\n");
            vTaskDelete(NULL);
            return;
        }
    }

    const float margin = 2.0f;
    const float x_max = plotter.x_max, y_max = plotter.y_max;
    const float shorter = (x_max < y_max) ? x_max : y_max;
    const float D = shorter - 2*margin;        // базовый диаметр
    const float R = D * 0.5f;                   // радиус
    const float cx = x_max * 0.5f;              // центр
    const float cy = y_max * 0.5f;
    const float speed = 40.0f;                  // мм/с по окружности (консервативно)

    // 2) Перо вверх
    uint32_t rid = send_to_stm32_cmd(CMD_PEN_UP, NULL);
    wait_command_completion(rid, "PEN_UP", 8000);

    // 3) В центр (G0), затем к старту (угол 0 => (cx+R, cy))
    char p[64];
    sprintf(p, "X%.2f,Y%.2f", cx, cy);
    rid = send_to_stm32_cmd(CMD_G0_RAPID, p);
    if (!wait_command_completion(rid, "G0 center", 15000)) {
        vTaskDelete(NULL); return;
    }

    sprintf(p, "X%.2f,Y%.2f", cx + R, cy);
    rid = send_to_stm32_cmd(CMD_G0_RAPID, p);
    if (!wait_command_completion(rid, "G0 start", 15000)) {
        vTaskDelete(NULL); return;
    }

    // 4) Перо вниз (не обязательно, можно и без контакта для отладки)
    rid = send_to_stm32_cmd(CMD_PEN_DOWN, NULL);
    wait_command_completion(rid, "PEN_DOWN", 8000);
    vTaskDelay(pdMS_TO_TICKS(150));

    // 5) Генерация и отправка круга по стриму
    printf("STREAM: circle R=%.2f mm, center=(%.2f,%.2f)\n", R, cx, cy);
    stream_generate_circle_and_send(cx, cy, R, speed);

    // 6) Перо вверх
    rid = send_to_stm32_cmd(CMD_PEN_UP, NULL);
    wait_command_completion(rid, "PEN_UP end", 8000);

    vTaskDelete(NULL);
}
// =============================== STREAMING FNCs ===================================

// Initialize continuous streaming
static void continuous_stream_init(void) {
    if (cont_stream.buffer) {
        free(cont_stream.buffer);
    }
    
    cont_stream.buffer = malloc(CONTINUOUS_BUFFER_SIZE);
    cont_stream.buffer_size = CONTINUOUS_BUFFER_SIZE;
    cont_stream.write_pos = 0;
    cont_stream.read_pos = 0;
    cont_stream.total_sent = 0;
    cont_stream.total_generated = 0;
    cont_stream.generation_complete = 0;
    cont_stream.active = 1;
    
    if (!cont_stream.buffer_mutex) {
        cont_stream.buffer_mutex = xSemaphoreCreateMutex();
    }
    if (!cont_stream.data_ready_sem) {
        cont_stream.data_ready_sem = xSemaphoreCreateBinary();
    }
}

// Clean up continuous streaming
static void continuous_stream_cleanup(void) {
    cont_stream.active = 0;
    
    if (cont_stream.generator_task) {
        vTaskDelete(cont_stream.generator_task);
        cont_stream.generator_task = NULL;
    }
    if (cont_stream.sender_task) {
        vTaskDelete(cont_stream.sender_task);
        cont_stream.sender_task = NULL;
    }
    
    if (cont_stream.buffer) {
        free(cont_stream.buffer);
        cont_stream.buffer = NULL;
    }
}

// Generate continuous pattern (example: spiral)
static void continuous_spiral_generator(void *pvParams) {
    float cx_mm = plotter.x_max / 2.0f;
    float cy_mm = plotter.y_max / 2.0f;
    float max_r = fminf(plotter.x_max, plotter.y_max) / 2.0f - 5.0f;
    float r = 5.0f;  // Start radius
    float theta = 0.0f;
    float r_increment = 0.5f;  // Spiral tightness
    float theta_increment = 0.1f;  // Angular resolution
    
    float acc_x = 0.0f, acc_y = 0.0f;
    float prev_x = cx_mm + r;
    float prev_y = cy_mm;
    
    printf("SPIRAL: Generating continuous spiral pattern\n");
    
    while (cont_stream.active && r < max_r) {
        // Calculate next point
        float x = cx_mm + r * cosf(theta);
        float y = cy_mm + r * sinf(theta);
        
        // Calculate steps
        float dx_mm = x - prev_x;
        float dy_mm = y - prev_y;
        
        acc_x += dx_mm * STEPS_PER_MM;
        acc_y += dy_mm * STEPS_PER_MM;
        
        uint8_t x_step = 0, x_dir = 0, y_step = 0, y_dir = 0;
        
        if (acc_x >= 1.0f) {
            x_step = 1; x_dir = 1;
            acc_x -= 1.0f;
        } else if (acc_x <= -1.0f) {
            x_step = 1; x_dir = 0;
            acc_x += 1.0f;
        }
        
        if (acc_y >= 1.0f) {
            y_step = 1; y_dir = 1;
            acc_y -= 1.0f;
        } else if (acc_y <= -1.0f) {
            y_step = 1; y_dir = 0;
            acc_y += 1.0f;
        }
        
        // Create nibble
        uint8_t nib = 0;
        if (x_step) nib |= 0x01;
        if (x_dir) nib |= 0x02;
        if (y_step) nib |= 0x04;
        if (y_dir) nib |= 0x08;
        
        // Wait for space in buffer
        while (1) {
            xSemaphoreTake(cont_stream.buffer_mutex, portMAX_DELAY);
            size_t used = (cont_stream.write_pos >= cont_stream.read_pos) ?
                         (cont_stream.write_pos - cont_stream.read_pos) :
                         (CONTINUOUS_BUFFER_SIZE - cont_stream.read_pos + cont_stream.write_pos);
            
            if (used < CONTINUOUS_BUFFER_SIZE - 2) {
                // Pack two nibbles per byte
                static uint8_t byte_buffer = 0;
                static int nibble_count = 0;
                
                if (nibble_count == 0) {
                    byte_buffer = nib;
                    nibble_count = 1;
                } else {
                    byte_buffer |= (nib << 4);
                    cont_stream.buffer[cont_stream.write_pos] = byte_buffer;
                    cont_stream.write_pos = (cont_stream.write_pos + 1) % CONTINUOUS_BUFFER_SIZE;
                    cont_stream.total_generated++;
                    nibble_count = 0;
                    
                    // Signal data available
                    xSemaphoreGive(cont_stream.data_ready_sem);
                }
                
                xSemaphoreGive(cont_stream.buffer_mutex);
                break;
            }
            xSemaphoreGive(cont_stream.buffer_mutex);
            vTaskDelay(pdMS_TO_TICKS(10));
        }
        
        prev_x = x;
        prev_y = y;
        
        // Update spiral
        theta += theta_increment;
        if (theta >= 2 * M_PI) {
            theta -= 2 * M_PI;
            r += r_increment;
        }
    }
    
    cont_stream.generation_complete = 1;
    printf("SPIRAL: Generation complete, total bytes: %zu\n", cont_stream.total_generated);
    vTaskDelete(NULL);
}

// Handle data request from STM32
static void handle_stream_request(uint16_t requested_bytes) {
    if (!cont_stream.active) return;
    
    printf("STREAM_REQ: STM32 requested %u bytes\n", requested_bytes);
    
    xSemaphoreTake(cont_stream.buffer_mutex, portMAX_DELAY);
    
    size_t available = (cont_stream.write_pos >= cont_stream.read_pos) ?
                      (cont_stream.write_pos - cont_stream.read_pos) :
                      (CONTINUOUS_BUFFER_SIZE - cont_stream.read_pos + cont_stream.write_pos);
    
    size_t to_send = (available < requested_bytes) ? available : requested_bytes;
    
    if (to_send > 0) {
        // Send data in chunks
        uint8_t chunk[CONTINUOUS_CHUNK_SIZE];
        size_t sent = 0;
        
        while (sent < to_send) {
            size_t chunk_size = (to_send - sent < CONTINUOUS_CHUNK_SIZE) ? 
                               (to_send - sent) : CONTINUOUS_CHUNK_SIZE;
            
            // Copy from circular buffer
            for (size_t i = 0; i < chunk_size; i++) {
                chunk[i] = cont_stream.buffer[cont_stream.read_pos];
                cont_stream.read_pos = (cont_stream.read_pos + 1) % CONTINUOUS_BUFFER_SIZE;
            }
            
            // Send DATA frame
            stream_send_frame(STREAM_CMD_DATA, chunk, chunk_size);
            sent += chunk_size;
            cont_stream.total_sent += chunk_size;
        }
        
        printf("STREAM_REQ: Sent %zu bytes (total: %zu)\n", sent, cont_stream.total_sent);
    } else if (cont_stream.generation_complete) {
        // No more data and generation complete - send END
        stream_send_frame(STREAM_CMD_END, NULL, 0);
        printf("STREAM_REQ: Sent END signal\n");
        continuous_stream_cleanup();
    } else {
        printf("STREAM_REQ: No data available yet\n");
    }
    
    xSemaphoreGive(cont_stream.buffer_mutex);
}

// =============================== TEST STREAMING BEGIN 2 ===================================

// =============================== TEST STREAMING END 2 ===================================

// =============================== TEST STREAMING BEGIN 2 ===================================

// =============================== TEST STREAMING END 2 ===================================

// =============================== TEST STREAMING BEGIN 2 ===================================

// =============================== TEST STREAMING END 2 ===================================


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

void draw_circle_from_current(float diameter_mm) {
    if (plotter.x_pos < 0 || plotter.y_pos < 0) {
        printf("ERROR: Unknown current position; request M114/GET_POSITION first.\n");
        return;
    }

    float current_x_mm = plotter.x_pos * MM_PER_STEP;
    float current_y_mm = plotter.y_pos * MM_PER_STEP;
    
    float radius = diameter_mm / 2.0;
    const int segments = 36;
    char params[64];
    
    printf("Drawing circle D=%.1fmm from (%.2f, %.2f)\n", 
           diameter_mm, current_x_mm, current_y_mm);
    
    // Поднимаем перо
    send_to_stm32_cmd(CMD_PEN_UP, NULL);
    vTaskDelay(pdMS_TO_TICKS(1000));
    
    // Движение к начальной точке круга
    sprintf(params, "X%.2f,Y%.2f", current_x_mm + radius, current_y_mm);
    send_to_stm32_cmd(CMD_G0_RAPID, params);
    vTaskDelay(pdMS_TO_TICKS(2000));
    
    // Опускаем перо
    send_to_stm32_cmd(CMD_PEN_DOWN, NULL);
    vTaskDelay(pdMS_TO_TICKS(1000));
    
    // Рисуем круг сегментами
    for (int i = 1; i <= segments; i++) {
        float theta = 2.0 * M_PI * i / segments;
        float x = current_x_mm + radius * cosf(theta);
        float y = current_y_mm + radius * sinf(theta);
        
        sprintf(params, "X%.2f,Y%.2f", x, y);
        send_to_stm32_cmd(CMD_G1_LINEAR, params);
        vTaskDelay(pdMS_TO_TICKS(500)); // Задержка между сегментами
    }
    
    // Поднимаем перо
    send_to_stm32_cmd(CMD_PEN_UP, NULL);
    printf("Circle completed\n");
}

void draw_concentric_circles_task(void *pvParameters);

// Build ticks for a single line segment at fixed sample rate.
// Each tick = 4 bits: [Y_DIR][Y_STEP][X_DIR][X_STEP]
static size_t build_ticks_line(float x0_mm, float y0_mm,
                               float x1_mm, float y1_mm,
                               float v_mm_s,   // requested linear speed
                               uint32_t f_hz,  // sample rate
                               uint8_t *out, size_t out_cap_bytes)
{
    // Convert to steps
    float dx_mm = x1_mm - x0_mm;
    float dy_mm = y1_mm - y0_mm;
    float dist_mm = sqrtf(dx_mm*dx_mm + dy_mm*dy_mm);
    if (dist_mm <= 1e-6f) return 0;

    float dirx = (dx_mm >= 0) ? 1.f : -1.f;
    float diry = (dy_mm >= 0) ? 1.f : -1.f;

    float dx_steps_abs = fabsf(dx_mm) * STEPS_PER_MM;
    float dy_steps_abs = fabsf(dy_mm) * STEPS_PER_MM;

    // time needed with requested speed
    float t_sec = dist_mm / fmaxf(v_mm_s, 1e-3f);
    // number of ticks for this segment
    uint32_t ticks = (uint32_t)lroundf(t_sec * (float)f_hz);
    if (ticks == 0) ticks = 1;

    // per-tick fractional step rates
    float sx = dx_steps_abs / (float)ticks;
    float sy = dy_steps_abs / (float)ticks;

    float ax = 0.f, ay = 0.f;
    size_t produced_ticks = 0;

    // Pack two nibbles per byte
    uint8_t cur_byte = 0;
    int nib_idx = 0;

    for (uint32_t i=0; i<ticks; ++i) {
        uint8_t t = 0;
        // set DIR bits
        if (dirx > 0) t |= (1u<<1); // X_DIR=1 => +X
        if (diry > 0) t |= (1u<<3); // Y_DIR=1 => +Y

        ax += sx;
        ay += sy;

        if (ax >= 1.0f) { t |= (1u<<0); ax -= 1.0f; }
        if (ay >= 1.0f) { t |= (1u<<2); ay -= 1.0f; }

        // pack nibble
        if (nib_idx == 0) {
            cur_byte = (t & 0x0F);
            nib_idx = 1;
        } else {
            cur_byte |= (uint8_t)((t & 0x0F) << 4);
            if (produced_ticks/2 < out_cap_bytes) {
                out[produced_ticks/2] = cur_byte;
            }
            nib_idx = 0;
        }
        produced_ticks++;
        if ((produced_ticks+1)/2 >= out_cap_bytes) break; // avoid overflow
    }

    // flush last half-filled byte
    if (nib_idx == 1 && (produced_ticks+1)/2 <= out_cap_bytes) {
        out[produced_ticks/2] = cur_byte;
    }

    return produced_ticks; // number of ticks (nibbles), not bytes
}

static void stream_send_circle(float cx_mm, float cy_mm, float R_mm,
                               float v_mm_s, uint32_t f_hz)
{
    // 1) send meta (must match STM32 Stream_Init defaults)
    stream_meta_t meta = { .f_hz = f_hz, .step_us = 6, .guard_us = 6, .inv_mask = 0 };
    stream_send_frame(STREAM_CMD_BEGIN, (const uint8_t*)&meta, sizeof(meta));

    // 2) build polyline
    const int segments = 144; // smoother than 36 for pen plotter

    float x_prev = cx_mm + R_mm;
    float y_prev = cy_mm;

    for (int i=1; i<=segments; ++i) {
        float th = (2.f * (float)M_PI * i) / (float)segments;
        float x = cx_mm + R_mm * cosf(th);
        float y = cy_mm + R_mm * sinf(th);

        size_t n_ticks = build_ticks_line(x_prev, y_prev, x, y, v_mm_s, f_hz, stream_pkt, sizeof(stream_pkt));
        size_t n_bytes = (n_ticks + 1) / 2;

        // send chunk
        if (n_bytes > 0) stream_send_frame(STREAM_CMD_DATA, stream_pkt, (uint16_t)n_bytes);

        x_prev = x; y_prev = y;
        // optional: small yield to keep WiFi/BLE alive
        taskYIELD();
    }

    // 3) start playback
    stream_send_frame(STREAM_CMD_START, NULL, 0);
}

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
    // Сначала пытаемся распарсить как команду
    uint32_t received_id;
    uint32_t cmd_id;
    char data[256] = {0};
    
    int parsed = sscanf(response, "%lu:%lu:%255s", &received_id, &cmd_id, data);
    
    // Если это валидный формат команды - обрабатываем
    if (parsed >= 2 && received_id > 0 && cmd_id < CMD_COUNT) {
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
            case CMD_SET_COLOR: {
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
        // Это отладочное сообщение
        printf("------- %s\n", response);
    }
    plotter.is_connected = true;
}

// UART communication task
void stm32_uart_task(void *pvParameters) {
    char buffer[512];
    int pos = 0;
    uint8_t binary_state = 0;
    
    printf("STM32 UART task started (with stream request support)\n");
    uart_flush(STM32_UART_PORT);
    
    while (1) {
        uint8_t data;
        int len = uart_read_bytes(STM32_UART_PORT, &data, 1, pdMS_TO_TICKS(10));
        
        if (len > 0) {
            // Check for binary stream request
            if (data == STREAM_SYNC0 && binary_state == 0) {
                binary_state = 1;
                continue;
            } else if (binary_state == 1) {
                if (data == STREAM_SYNC1) {
                    // Read command byte
                    uint8_t cmd;
                    uart_read_bytes(STM32_UART_PORT, &cmd, 1, pdMS_TO_TICKS(100));
                    
                    if (cmd == STREAM_CMD_REQUEST_MORE) {
                        // Read payload length (2 bytes)
                        uint8_t len_bytes[2];
                        uart_read_bytes(STM32_UART_PORT, len_bytes, 2, pdMS_TO_TICKS(100));
                        uint16_t payload_len = len_bytes[0] | (len_bytes[1] << 8);
                        
                        // Read requested size
                        if (payload_len == 2) {
                            uint8_t size_bytes[2];
                            uart_read_bytes(STM32_UART_PORT, size_bytes, 2, pdMS_TO_TICKS(100));
                            uint16_t requested = size_bytes[0] | (size_bytes[1] << 8);
                            
                            // Skip CRC byte
                            uint8_t dummy;
                            uart_read_bytes(STM32_UART_PORT, &dummy, 1, pdMS_TO_TICKS(100));
                            
                            // Handle request
                            handle_stream_request(requested);
                        }
                    }
                    binary_state = 0;
                    continue;
                } else {
                    binary_state = 0;
                    // Process the SYNC0 and current byte as normal text
                }
            }
            
            // Normal text processing
            if (data == '\n' || data == '\r') {
                if (pos > 0) {
                    buffer[pos] = '\0';
                    process_stm32_response(buffer);
                    pos = 0;
                }
            } else if (pos < sizeof(buffer) - 1) {
                if (data >= 32 || data == '\t') {
                    buffer[pos++] = data;
                }
            } else {
                pos = 0;
            }
        }
        
        taskYIELD();
    }
}

void start_continuous_spiral(void) {
    printf("Starting continuous spiral streaming\n");
    
    // Initialize
    continuous_stream_init();
    
    // Send BEGIN with continuous flag
    uint8_t begin_payload[10];
    uint32_t f_hz = STREAM_F_HZ;
    begin_payload[0] = (uint8_t)(f_hz & 0xFF);
    begin_payload[1] = (uint8_t)((f_hz >> 8) & 0xFF);
    begin_payload[2] = (uint8_t)((f_hz >> 16) & 0xFF);
    begin_payload[3] = (uint8_t)((f_hz >> 24) & 0xFF);
    begin_payload[4] = (uint8_t)(STREAM_STEP_US & 0xFF);
    begin_payload[5] = (uint8_t)(STREAM_STEP_US >> 8);
    begin_payload[6] = (uint8_t)(STREAM_GUARD_US & 0xFF);
    begin_payload[7] = (uint8_t)(STREAM_GUARD_US >> 8);
    begin_payload[8] = STREAM_INV_MASK;
    begin_payload[9] = 0xFF;  // Magic byte for continuous mode
    
    stream_send_frame(STREAM_CMD_BEGIN, begin_payload, 10);
    
    // Start generator task
    xTaskCreate(continuous_spiral_generator, "spiral_gen", 4096, NULL, 5, &cont_stream.generator_task);
    
    // Wait a bit for initial data generation
    vTaskDelay(pdMS_TO_TICKS(100));
    
    // Send initial chunk
    handle_stream_request(CONTINUOUS_CHUNK_SIZE);
    
    // Send START command
    stream_send_frame(STREAM_CMD_START, NULL, 0);
    
    printf("Continuous streaming started\n");
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

void draw_concentric_circles_task(void *pvParameters) {
    const float margin_mm = 2.0f;
    const int   segments  = 36;
    char buf[64];
    uint32_t request_id;

    printf("CIRCLES: Task started, waiting for system ready...\n");

    // Получаем лимиты
    printf("CIRCLES: System ready, requesting limits...\n");
    send_to_stm32_cmd(CMD_GET_LIMITS, NULL);
    
    // Ждем лимиты (только это критично)
    uint32_t limits_timeout = xTaskGetTickCount();
    while ((plotter.x_max == 0.0f || plotter.y_max == 0.0f) && 
           (xTaskGetTickCount() - limits_timeout) < pdMS_TO_TICKS(5000)) {
        vTaskDelay(pdMS_TO_TICKS(50));
    }

    if (plotter.x_max == 0.0f || plotter.y_max == 0.0f) {
        printf("CIRCLES: ERROR - Failed to get limits!\n");
        send_to_stm32_cmd(CMD_GET_LIMITS, NULL);
        vTaskDelay(pdMS_TO_TICKS(500));  // Сократили с 1000
        
        if (plotter.x_max == 0.0f || plotter.y_max == 0.0f) {
            printf("CIRCLES: ERROR - Still no limits, aborting!\n");
            vTaskDelete(NULL);
            return;
        }
    }
    
    printf("CIRCLES: Limits: X=%.2f, Y=%.2f\n", plotter.x_max, plotter.y_max);
    
    float x_max_mm = plotter.x_max;
    float y_max_mm = plotter.y_max;
    float shorter  = (x_max_mm < y_max_mm) ? x_max_mm : y_max_mm;
    float base_D   = shorter - 2 * margin_mm;
    float cx       = x_max_mm / 2.0f;
    float cy       = y_max_mm / 2.0f;
    printf("CIRCLES: Center=(%.2f,%.2f), base_D=%.2f\n", cx, cy, base_D);

    send_to_stm32_cmd(CMD_G90_ABSOLUTE, NULL);
    
    // Рисуем 4 концентрических круга
    for (int color = 1; color <= 4; color++) {
        printf("\nCIRCLES: === Color %d ===\n", color);

        float D = base_D * (1.0f - 0.2f * (float)(color - 1));
        float R = D / 2.0f;
        
        if (R <= 0) {
            printf("CIRCLES: Radius too small, skipping color %d\n", color);
            continue;
        }
        
        printf("CIRCLES: Diameter=%.2f, Radius=%.2f\n", D, R);
            
        // Шаг 1: Поднять перо
        printf("CIRCLES: Step 1 - Lifting pen\n");
        request_id = send_to_stm32_cmd(CMD_PEN_UP, NULL);
        if (!wait_command_completion(request_id, "PEN_UP", 10000)) {
            printf("CIRCLES: Pen up failed, continuing anyway\n");
        }
        // УБРАЛИ vTaskDelay - сразу идем дальше

        // Шаг 2: Движение к центру
        printf("CIRCLES: Step 2 - Moving to center (%.2f, %.2f)\n", cx, cy);
        sprintf(buf, "X%.2f,Y%.2f", cx, cy);
        request_id = send_to_stm32_cmd(CMD_G0_RAPID, buf);
        if (!wait_command_completion(request_id, "Move to center", 15000)) {
            printf("CIRCLES: Move to center failed, skipping color %d\n", color);
            continue;
        }
        // УБРАЛИ vTaskDelay

        // Шаг 3: УБРАЛИ паузу в центре - не нужна

        // Шаг 4: Смена цвета
        printf("CIRCLES: Step 4 - Setting color %d\n", color);
        sprintf(buf, "C%hhu", (uint8_t)(color - 1));
        request_id = send_to_stm32_cmd(CMD_SET_COLOR, buf);
        if (!wait_command_completion(request_id, "Color change", 30000)) {
            printf("CIRCLES: Color change failed, continuing with current color\n");
        }
        // УБРАЛИ vTaskDelay

        // Шаг 5: Движение к стартовой точке окружности
        float start_x = cx + R;
        float start_y = cy;
        printf("CIRCLES: Step 5 - Moving to start point (%.2f, %.2f)\n", start_x, start_y);
        sprintf(buf, "X%.2f,Y%.2f", start_x, start_y);
        request_id = send_to_stm32_cmd(CMD_G0_RAPID, buf);
        if (!wait_command_completion(request_id, "Move to start", 15000)) {
            printf("CIRCLES: Move to start failed, skipping color %d\n", color);
            continue;
        }
        // УБРАЛИ vTaskDelay

        // Шаг 6: Опустить перо
        printf("CIRCLES: Step 6 - Lowering pen\n");
        request_id = send_to_stm32_cmd(CMD_PEN_DOWN, NULL);
        if (!wait_command_completion(request_id, "PEN_DOWN", 10000)) {
            printf("CIRCLES: Pen down failed, drawing without pen contact\n");
        }
        vTaskDelay(pdMS_TO_TICKS(200));  // Оставляем МИНИМАЛЬНУЮ задержку только после опускания пера

        // Шаг 7: Рисование окружности сегментами БЕЗ ЗАДЕРЖЕК
        printf("CIRCLES: Step 7 - Drawing circle (%d segments)\n", segments);
        bool circle_completed = true;
        
        for (int i = 1; i <= segments; i++) {
            float theta = 2.0f * M_PI * i / segments;
            float x_point = cx + R * cosf(theta);
            float y_point = cy + R * sinf(theta);
            
            sprintf(buf, "X%.2f,Y%.2f", x_point, y_point);
            request_id = send_to_stm32_cmd(CMD_G1_LINEAR, buf);
            
            // Используем более короткий таймаут для сегментов
            char segment_name[32];
            sprintf(segment_name, "Segment %d", i);
            if (!wait_command_completion(request_id, segment_name, 5000)) {  // Сократили с 8000
                printf("CIRCLES: Segment %d failed, stopping circle\n", i);
                circle_completed = false;
                break;
            }
            
            // Прогресс выводим реже
            if (i % 9 == 0) {  // Изменили с 6 на 9
                printf("CIRCLES: Progress %d/%d segments\n", i, segments);
            }
        }

        // Шаг 8: Поднять перо после завершения круга
        printf("CIRCLES: Step 8 - Lifting pen after circle\n");
        request_id = send_to_stm32_cmd(CMD_PEN_UP, NULL);
        wait_command_completion(request_id, "Final PEN_UP", 5000);

        printf("CIRCLES: Color %d %s\n", color, circle_completed ? "completed successfully" : "failed");
        
        // УБРАЛИ финальную задержку между кругами
    }

    printf("CIRCLES: All circles finished! Returning to center\n");
    
    // Финальное позиционирование в центр
    sprintf(buf, "X%.2f,Y%.2f", cx, cy);
    send_to_stm32_cmd(CMD_G0_RAPID, buf);
    
    vTaskDelay(pdMS_TO_TICKS(500));  // Сократили с 2000
    
    printf("CIRCLES: Task completed, deleting task\n");
    vTaskDelete(NULL);
}

void test_simple_move(void) {
    printf("\n=== TEST: Simple manual movement ===\n");
    
    // Отправляем простые текстовые команды для проверки
    send_to_stm32_cmd(CMD_HOME, NULL);
    vTaskDelay(pdMS_TO_TICKS(5000));
    
    send_to_stm32_cmd(CMD_PEN_UP, NULL);
    vTaskDelay(pdMS_TO_TICKS(2000));
    
    // Простое движение
    send_to_stm32_cmd(CMD_G0_RAPID, "X10.0,Y10.0");
    vTaskDelay(pdMS_TO_TICKS(3000));
    
    send_to_stm32_cmd(CMD_G0_RAPID, "X20.0,Y20.0");
    vTaskDelay(pdMS_TO_TICKS(3000));
    
    printf("TEST: Complete\n");
}

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
            
        case '3':
            printf("KEYPAD: Pen UP\n");
            send_to_stm32_cmd(CMD_PEN_UP, NULL);
            break;
            
        case '4':
            printf("KEYPAD: Pen DOWN\n");
            send_to_stm32_cmd(CMD_PEN_DOWN, NULL);
            break;
            
        case '5':
            printf("KEYPAD: Select color 0\n");
            send_to_stm32_cmd(CMD_SET_COLOR, "C0");
            break;
            
        case '6':
            printf("KEYPAD: Select color 1\n");
            send_to_stm32_cmd(CMD_SET_COLOR, "C1");
            break;
            
        case '7':
            printf("KEYPAD: Select color 2\n");
            send_to_stm32_cmd(CMD_SET_COLOR, "C2");
            break;
            
        case '8':
            printf("KEYPAD: Select color 3\n");
            send_to_stm32_cmd(CMD_SET_COLOR, "C3");
            break;
            
        case '9':
            printf("KEYPAD: Draw 10mm circle from current position\n");
            draw_circle_from_current(10.0); // Функция ниже
            break;
            
        case '0':
        {
            // Example: concentric circles by streaming ticks
            // Make sure STM32 stream parser is ready before using this
            const float margin_mm = 2.0f;
            const float x_max = plotter.x_max;
            const float y_max = plotter.y_max;
            if (x_max <= 0 || y_max <= 0) {
                send_to_stm32_cmd(CMD_GET_LIMITS, NULL);
                printf("STREAM: no limits yet\n");
                break;
            }

            float shorter = (x_max < y_max) ? x_max : y_max;
            float base_D  = shorter - 2*margin_mm;
            float cx = x_max * 0.5f, cy = y_max * 0.5f;

            // pen-up, move to start of outer circle (text protocol still)
            send_to_stm32_cmd(CMD_PEN_UP, NULL);
            char p[64];
            sprintf(p, "X%.2f,Y%.2f", cx + base_D*0.5f, cy);
            send_to_stm32_cmd(CMD_G0_RAPID, p);

            // Now stream 4 circles with D, 0.8D, 0.6D, 0.4D
            const uint32_t f_hz = 10000;
            const float v_mm_s  = 20.0f; // ~1200 mm/min

            for (int i=0;i<4;i++) {
                float D = base_D * (1.0f - 0.2f * i);
                float R = 0.5f * D;

                // set color via text protocol
                char cbuf[8];
                sprintf(cbuf, "C%u", (unsigned)i);
                send_to_stm32_cmd(CMD_SET_COLOR, cbuf);
                // lower pen
                send_to_stm32_cmd(CMD_PEN_DOWN, NULL);

                // stream circle
                stream_send_circle(cx, cy, R, v_mm_s, f_hz);

                // lift pen after playback (можно по завершению, когда добавим ACK)
                send_to_stm32_cmd(CMD_PEN_UP, NULL);
            }
            break;
        }

        case 'A':  // Start continuous spiral
        {
            printf("KEYPAD: Starting continuous spiral stream\n");
            
            // Ensure we're homed first
            if (!plotter.is_homed) {
                printf("ERROR: Not homed, please home first\n");
                break;
            }
            
            // Get limits if needed
            if (plotter.x_max == 0.0f || plotter.y_max == 0.0f) {
                send_to_stm32_cmd(CMD_GET_LIMITS, NULL);
                vTaskDelay(pdMS_TO_TICKS(1000));
            }
            
            // Start continuous streaming
            start_continuous_spiral();
            break;
        }
        
        case 'B':  // Stop continuous streaming
        {
            printf("KEYPAD: Stopping continuous stream\n");
            
            // Send END signal
            stream_send_frame(STREAM_CMD_END, NULL, 0);
            
            // Clean up
            continuous_stream_cleanup();
            
            // Send ABORT just to be safe
            stream_send_frame(STREAM_CMD_ABORT, NULL, 0);
            
            printf("Continuous streaming stopped\n");
            break;
        }
                    
        case 'C':
            printf("KEYPAD: Move to (20,20)\n");
            sprintf(params, "X20.00,Y20.00");
            send_to_stm32_cmd(CMD_G0_RAPID, params);
            break;
            
        case 'D':
            printf("KEYPAD: Move to (10,20)\n");
            sprintf(params, "X10.00,Y20.00");
            send_to_stm32_cmd(CMD_G0_RAPID, params);
            break;
            
        case '*':
            printf("KEYPAD: Emergency STOP!\n");
            send_to_stm32_cmd(CMD_EMERGENCY_STOP, NULL);
            break;
            
        case '#':
            printf("KEYPAD: Get status\n");
            send_to_stm32_cmd(CMD_GET_STATUS, NULL);
            send_to_stm32_cmd(CMD_GET_POSITION, NULL);
            break;
            
        default:
            printf("KEYPAD: Unknown key\n");
            break;
    }
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
    
    spi_bus_initialize(SPI2_HOST, &buscfg, SPI_DMA_CH_AUTO);
    spi_bus_add_device(SPI2_HOST, &devcfg, &spi);
    
    // Initialize OLED display
    u8g2_Setup_ssd1309_128x64_noname0_f(&u8g2, U8G2_R0, u8x8_byte_hw_spi, u8x8_gpio_and_delay);
    u8g2_InitDisplay(&u8g2);
    u8g2_SetPowerSave(&u8g2, 0);
    
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
    xTaskCreate(keypad_task, "keypad", 2048, NULL, 3, NULL);
        // xTaskCreate(
    //     draw_concentric_circles_task,   // task function
    //     "draw_circles",                 // task name
    //     8192,                           // stack size
    //     NULL,                           // params
    //     3,                              // priority
    //     NULL                            // task handle
    // );
    
    // Optional: enable test movement task
    // xTaskCreate(test_movement_task, "test_movement", 2048, NULL, 2, NULL);
    
    // Main display loop - reduced delay for more responsive display
    while (1) {
        update_display();
        vTaskDelay(pdMS_TO_TICKS(100));
    }
}

// idf.py -p /dev/ttyUSB0 -b 115200 flash monitor