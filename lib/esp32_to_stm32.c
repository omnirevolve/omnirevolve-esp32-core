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
#include "driver/spi_common.h"

#include "u8g2.h"

#if __has_include("esp_rom_sys.h")
  #include "esp_rom_sys.h"   // ets_delay_us() on newer IDF
#else
  #include "esp32/rom/ets_sys.h"   // ets_delay_us() on older IDF
#endif

// Wi-Fi state polling is now done inside the OLED task
#include "esp_wifi.h"
#include "esp_netif.h"

// Choose a DMA channel that works across IDF versions
#if defined(SPI_DMA_CH_AUTO)
  #define OLED_DMA_CH SPI_DMA_CH_AUTO
#else
  // Older IDF doesn't define SPI_DMA_CH_AUTO; 1 is a safe DMA channel for VSPI.
  #define OLED_DMA_CH 1
#endif

// SPI configuration for STM32 communication
#define STM32_SPI_HOST    HSPI_HOST
#define STM32_SPI_MOSI    13   // GPIO13
#define STM32_SPI_MISO    12   // GPIO12 (not used but connected)
#define STM32_SPI_CLK     14   // GPIO14
#define STM32_READY_PIN   15   // GPIO15 - READY signal from STM32

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

// Simple 3x3 keypad
#define KEYPAD_ROWS 3
#define KEYPAD_COLS 3

static const int row_pins[KEYPAD_ROWS] = {32, 33, 27};
static const int col_pins[KEYPAD_COLS] = {22, 4, 19};

typedef enum {
    CURRENT_CMD_IDLE = 0,
    CURRENT_CMD_CALIBRATING,
    CURRENT_CMD_HOMING,
    CURRENT_CMD_DRAWING,
    CURRENT_CMD_ERROR
} CurrentCmd_t;

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
    TickType_t last_position_request;
    TickType_t last_cmd_status_request;
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
    .state.current_cmd = CURRENT_CMD_IDLE,

    .last_heartbeat_sent = 0,
    .last_status_request = 0,
    .last_position_request = 0,
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

// =========================== KEYPAD =============================
void keypad_init(void) {
    for (int i = 0; i < KEYPAD_ROWS; i++) {
        gpio_set_direction(row_pins[i], GPIO_MODE_OUTPUT);
        gpio_set_level(row_pins[i], 1); // default HIGH
    }
    for (int i = 0; i < KEYPAD_COLS; i++) {
        gpio_set_direction(col_pins[i], GPIO_MODE_INPUT);
        gpio_set_pull_mode(col_pins[i], GPIO_PULLUP_ONLY);
    }
    printf("Keypad initialized\n");
}

char keypad_scan(void) {
    for (int row = 0; row < KEYPAD_ROWS; row++) {
        gpio_set_level(row_pins[row], 0); // row -> LOW
        vTaskDelay(pdMS_TO_TICKS(1));     // small settle delay
        for (int col = 0; col < KEYPAD_COLS; col++) {
            if (gpio_get_level(col_pins[col]) == 0) {
                gpio_set_level(row_pins[row], 1);
                return keypad_keys[row][col];
            }
        }
        gpio_set_level(row_pins[row], 1);
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
            // 200 ms debounce
            uint32_t now = xTaskGetTickCount();
            if (key != keypad_state.last_key ||
                (now - keypad_state.last_key_time) > pdMS_TO_TICKS(200)) {
                process_keypad_command(key);
                keypad_state.last_key = key;
                keypad_state.last_key_time = now;
            }
        } else {
            keypad_state.last_key = 0;
        }
        vTaskDelay(pdMS_TO_TICKS(50));
    }
}

// ======================= OLED DISPLAY (U8g2 over HW SPI / VSPI) =======================
typedef enum {
    WIFI_STATE_DISCONNECTED = 0,
    WIFI_STATE_CONNECTING,
    WIFI_STATE_CONNECTED,
    WIFI_STATE_AP_MODE,
    WIFI_STATE_ERROR
} wifi_state_t;

typedef struct {
    wifi_state_t wifi_state;
    char wifi_ssid[32];
    char ip_address[16];
    int8_t wifi_rssi;
} oled_wifi_info_t;

// ======================= END OLED SECTION =======================


#define OLED_SPI_HOST   VSPI_HOST
#define OLED_PIN_MOSI   23
#define OLED_PIN_SCK    18
#define OLED_PIN_CS     5
#define OLED_PIN_DC     17
#define OLED_PIN_RST    16

#define OLED_WIDTH      128
#define OLED_HEIGHT     64

static u8g2_t u8g2;
static oled_wifi_info_t wifi_info = {0};
static char oled_temp_message[64] = {0};
static TickType_t oled_message_timeout = 0;
static volatile uint8_t oled_initialized = 0;

static spi_device_handle_t s_oled_spi = NULL;

// --- U8g2 byte callback: driver controls CS; we only set DC and send bytes ---
static uint8_t u8x8_byte_hw_spi(u8x8_t *u8x8, uint8_t msg, uint8_t arg_int, void *arg_ptr)
{
    (void)u8x8;
    switch (msg) {
    case U8X8_MSG_BYTE_INIT:
        return 1;

    case U8X8_MSG_BYTE_SET_DC:
        if (OLED_PIN_DC >= 0) gpio_set_level(OLED_PIN_DC, arg_int); // 0=command, 1=data
        return 1;

    case U8X8_MSG_BYTE_SEND: {
        spi_transaction_t t = {
            .length    = 8 * arg_int,
            .tx_buffer = arg_ptr
        };
        return (spi_device_polling_transmit(s_oled_spi, &t) == ESP_OK) ? 1 : 0;
    }

    default:
        return 0;
    }
}

static uint8_t u8x8_gpio_and_delay(u8x8_t *u8x8, uint8_t msg, uint8_t arg_int, void *arg_ptr)
{
    (void)u8x8; (void)arg_ptr;
    switch (msg) {
    case U8X8_MSG_GPIO_AND_DELAY_INIT:
        if (OLED_PIN_DC  >= 0) { gpio_set_direction(OLED_PIN_DC,  GPIO_MODE_OUTPUT); }
        if (OLED_PIN_RST >= 0) { gpio_set_direction(OLED_PIN_RST, GPIO_MODE_OUTPUT); gpio_set_level(OLED_PIN_RST, 1); }
        return 1;

    case U8X8_MSG_DELAY_MILLI:
        vTaskDelay(pdMS_TO_TICKS(arg_int));
        return 1;

    case U8X8_MSG_DELAY_10MICRO:
        ets_delay_us(10);
        return 1;

    case U8X8_MSG_GPIO_RESET:
        if (OLED_PIN_RST >= 0) gpio_set_level(OLED_PIN_RST, arg_int);
        return 1;

    default:
        return 1;
    }
}

static void oled_spi_bus_init(void)
{
    spi_bus_config_t buscfg = {
        .mosi_io_num     = OLED_PIN_MOSI,
        .miso_io_num     = -1,
        .sclk_io_num     = OLED_PIN_SCK,
        .quadwp_io_num   = -1,
        .quadhd_io_num   = -1,
        .max_transfer_sz = 1024
    };

    // VSPI uses DMA channel 2 (HSPI for STM32 can use DMA 1)
    ESP_ERROR_CHECK(spi_bus_initialize(OLED_SPI_HOST, &buscfg, 2));

    spi_device_interface_config_t devcfg = {
        .mode            = 0,
        .clock_speed_hz  = 8000000,   // 8 MHz; adjust if needed
        .spics_io_num    = OLED_PIN_CS, // driver toggles CS
        .queue_size      = 7
    };
    ESP_ERROR_CHECK(spi_bus_add_device(OLED_SPI_HOST, &devcfg, &s_oled_spi));
}

static void oled_hw_reset(void)
{
    if (OLED_PIN_RST < 0) return;
    gpio_set_level(OLED_PIN_RST, 1);
    vTaskDelay(pdMS_TO_TICKS(1));
    gpio_set_level(OLED_PIN_RST, 0);
    vTaskDelay(pdMS_TO_TICKS(10));
    gpio_set_level(OLED_PIN_RST, 1);
    vTaskDelay(pdMS_TO_TICKS(10));
}

void oled_init(void)
{
    if (oled_initialized) return;

    oled_spi_bus_init();
    oled_hw_reset();

    // SSD1309 via SPI, full frame buffer (same behavior as legacy code)
    u8g2_Setup_ssd1309_128x64_noname0_f(
        &u8g2, U8G2_R0,
        u8x8_byte_hw_spi,
        u8x8_gpio_and_delay
    );

    u8g2_InitDisplay(&u8g2);
    u8g2_SetPowerSave(&u8g2, 0);
    u8g2_ClearBuffer(&u8g2);
    u8g2_SetFont(&u8g2, u8g2_font_4x6_mf);

    oled_initialized = 1;
    printf("[OLED] ready (VSPI: MOSI=%d SCK=%d CS=%d DC=%d RST=%d)\n",
           OLED_PIN_MOSI, OLED_PIN_SCK, OLED_PIN_CS, OLED_PIN_DC, OLED_PIN_RST);

    u8g2_DrawStr(&u8g2, 0, 10, "Start uROS-agent on PC-node");
    u8g2_DrawStr(&u8g2, 0, 18, "and restart me (ESP32)");
    u8g2_SendBuffer(&u8g2);
}

// --- Public status update APIs (kept for external use if needed) ---
void oled_update_wifi_status(wifi_state_t state, const char *ssid, const char *ip, int8_t rssi)
{
    if (!oled_initialized) return;
    wifi_info.wifi_state = state;
    if (ssid) strncpy(wifi_info.wifi_ssid, ssid, sizeof(wifi_info.wifi_ssid) - 1);
    if (ip)   strncpy(wifi_info.ip_address, ip, sizeof(wifi_info.ip_address) - 1);
    wifi_info.wifi_rssi = rssi;
}

void oled_show_message(const char *message, uint32_t duration_ms)
{
    if (!oled_initialized) return;

    if (message) {
        strncpy(oled_temp_message, message, sizeof(oled_temp_message) - 1);
        oled_message_timeout = xTaskGetTickCount() + pdMS_TO_TICKS(duration_ms);
    } else {
        oled_temp_message[0] = '\0';
        oled_message_timeout = 0;
    }
}

// --- Internal Wi-Fi poller used by OLED task ---
static void oled_poll_wifi_and_update(void)
{
    wifi_state_t state = WIFI_STATE_DISCONNECTED;
    char ssid[33] = {0};
    char ip_str[16] = {0};
    int8_t rssi = -100;

    wifi_ap_record_t ap_info;
    esp_netif_ip_info_t ip_info;

    // STA interface
    esp_netif_t *sta = esp_netif_get_handle_from_ifkey("WIFI_STA_DEF");
    // AP interface (used only if AP mode is active)
    esp_netif_t *ap  = esp_netif_get_handle_from_ifkey("WIFI_AP_DEF");

    if (esp_wifi_sta_get_ap_info(&ap_info) == ESP_OK) {
        // Connected to AP
        state = WIFI_STATE_CONNECTED;
        memcpy(ssid, ap_info.ssid, sizeof(ap_info.ssid));
        ssid[32] = '\0';
        rssi = ap_info.rssi;

        if (sta && esp_netif_get_ip_info(sta, &ip_info) == ESP_OK) {
            sprintf(ip_str, IPSTR, IP2STR(&ip_info.ip));
        }
    } else {
        // Not connected — check current Wi-Fi mode
        wifi_mode_t mode;
        if (esp_wifi_get_mode(&mode) == ESP_OK) {
            if (mode == WIFI_MODE_AP) {
                state = WIFI_STATE_AP_MODE;
                strcpy(ssid, "AP Mode");
                if (ap && esp_netif_get_ip_info(ap, &ip_info) == ESP_OK) {
                    sprintf(ip_str, IPSTR, IP2STR(&ip_info.ip));
                }
            } else if (mode == WIFI_MODE_STA || mode == WIFI_MODE_APSTA) {
                state = WIFI_STATE_CONNECTING;
                strcpy(ssid, "Scanning...");
            } else {
                state = WIFI_STATE_DISCONNECTED;
            }
        } else {
            state = WIFI_STATE_ERROR;
        }
    }

    // Push result into shared struct (mutex inside)
    oled_update_wifi_status(state, ssid, ip_str, rssi);
}

static char* get_current_plotter_task() {
    switch (plotter.state.current_cmd) {
        case CURRENT_CMD_IDLE: return "IDL";
        case CURRENT_CMD_CALIBRATING: return "CAL";
        case CURRENT_CMD_DRAWING: return "DRW";
        case CURRENT_CMD_HOMING: return "HOM";
        case CURRENT_CMD_ERROR: return "ERR";
    }
    return "?";
}

// --- Rendering ---
static void oled_draw(void)
{
    if (!oled_initialized) return;

    u8g2_ClearBuffer(&u8g2);

    if (oled_message_timeout > xTaskGetTickCount() && oled_temp_message[0] != '\0') {
        //u8g2_SetFont(&u8g2, u8g2_font_7x13_tf);
        int w = u8g2_GetStrWidth(&u8g2, oled_temp_message);
        u8g2_DrawStr(&u8g2, (OLED_WIDTH - w) / 2, 32, oled_temp_message);
        u8g2_SendBuffer(&u8g2);
        return;
    }

    //u8g2_SetFont(&u8g2, u8g2_font_6x10_tf);
    int y = 7;
    const int delta_y = 8;
    char buf[64];

    if (wifi_info.wifi_state != WIFI_STATE_CONNECTED) {
        const char *wifi_str = "WiFi: -";
        switch (wifi_info.wifi_state) {
            case WIFI_STATE_DISCONNECTED: wifi_str = "WiFi: d/c"; break;
            case WIFI_STATE_CONNECTING:   wifi_str = "WiFi: ..."; break;
            case WIFI_STATE_AP_MODE:      wifi_str = "WiFi: AP"; break;
            case WIFI_STATE_ERROR:        wifi_str = "WiFi: ER"; break;
            default: wifi_str = "WiFi: ?"; break;
        }
        u8g2_DrawStr(&u8g2, 0, y, wifi_str); y += delta_y;
    } else {
        snprintf(buf, sizeof(buf), "WiFi: %d dBm", wifi_info.wifi_rssi);
        u8g2_DrawStr(&u8g2, 0, y, buf); y += delta_y;
    }

    //u8g2_DrawHLine(&u8g2, 0, y + 1, OLED_WIDTH); y += 5;

    if (!plotter.state.is_connected) {
        snprintf(buf, sizeof(buf), "STM32: Disconnected");
        u8g2_DrawStr(&u8g2, 0, y, buf); y += delta_y;
    } else {
        snprintf(buf, sizeof(buf), "STM32: CAL:%d HOM:%d CMD:%s",
                 plotter.state.is_calibrated ? 1 : 0,
                 plotter.state.is_homed ? 1 : 0,
                 get_current_plotter_task());
        u8g2_DrawStr(&u8g2, 0, y, buf); y += delta_y;

        snprintf(buf, sizeof(buf), "Pos: X%ld Y%ld", (long)plotter.state.x_pos, (long)plotter.state.y_pos);
        u8g2_DrawStr(&u8g2, 0, y, buf); y += delta_y;

        if (plotter.state.current_cmd == CURRENT_CMD_DRAWING) {
            snprintf(buf, sizeof(buf), "DRW: %u", plotter.state.bytes_processed);
            u8g2_DrawStr(&u8g2, 0, y, buf); y += delta_y;
        }
    }

    u8g2_SendBuffer(&u8g2);
}

void oled_task(void *pvParameters)
{
    printf("[OLED] task started\n");

    const TickType_t draw_period  = pdMS_TO_TICKS(200);   // 5 Гц отрисовка
    const TickType_t wifi_period  = pdMS_TO_TICKS(2000);  // 0.5 Гц опрос Wi-Fi
    TickType_t last_draw = xTaskGetTickCount();
    TickType_t last_wifi = last_draw;

    for (;;) {
        if (!oled_initialized) {
            vTaskDelay(pdMS_TO_TICKS(200));
            continue;
        }

        TickType_t now = xTaskGetTickCount();

        if ((now - last_wifi) >= wifi_period) {
            oled_poll_wifi_and_update();
            last_wifi = now;
        }

        if ((now - last_draw) >= draw_period) {
            oled_draw();
            last_draw = now;
        }

        vTaskDelay(pdMS_TO_TICKS(200));
    }
}

// ======================= END OLED DISPLAY =======================


// ========================= UART binary protocol (ESP32 side) =========================
// ---- Framing constants ----
// ---- CRC16-CCITT (0xFFFF, poly 0x1021) over payload only ----
static inline uint16_t crc16_ccitt_step(uint16_t crc, uint8_t b) {
    crc ^= (uint16_t)b << 8;
    for (int i=0;i<8;i++) crc = (crc & 0x8000) ? (uint16_t)((crc<<1) ^ 0x1021) : (uint16_t)(crc<<1);
    return crc;
}

static uint16_t crc16_ccitt_buf(const uint8_t *p, uint16_t n) {
    uint16_t crc = 0xFFFF;
    while (n--) crc = crc16_ccitt_step(crc, *p++);
    return crc;
}

static inline void wr_u16le(uint8_t *p, uint16_t v){ p[0]=(uint8_t)(v&0xFF); p[1]=(uint8_t)(v>>8); }
static inline void wr_u32le(uint8_t *p, uint32_t v){ p[0]=(uint8_t)(v&0xFF); p[1]=(uint8_t)((v>>8)&0xFF); p[2]=(uint8_t)((v>>16)&0xFF); p[3]=(uint8_t)((v>>24)&0xFF); }
static inline uint16_t rd_u16le(const uint8_t *p){ return (uint16_t)(p[0] | ((uint16_t)p[1]<<8)); }
static inline uint32_t rd_u32le(const uint8_t *p){ return (uint32_t)p[0] | ((uint32_t)p[1]<<8) | ((uint32_t)p[2]<<16) | ((uint32_t)p[3]<<24); }

// ---- Low-level: send one framed packet over UART ----
static void uart_send_frame(uint8_t type, const uint8_t *payload, uint16_t len)
{
    uint8_t hdr[5];
    hdr[0] = UFR_SYNC0;
    hdr[1] = UFR_SYNC1;
    hdr[2] = type;
    hdr[3] = (uint8_t)(len & 0xFF);
    hdr[4] = (uint8_t)(len >> 8);

    uint16_t crc = crc16_ccitt_buf(payload, len);
    uint8_t tail[2] = { (uint8_t)(crc>>8), (uint8_t)(crc & 0xFF) };

    uart_write_bytes(STM32_UART_PORT, (const char*)hdr, sizeof(hdr));
    if (len) uart_write_bytes(STM32_UART_PORT, (const char*)payload, len);
    uart_write_bytes(STM32_UART_PORT, (const char*)tail, sizeof(tail));
    // binary: no newline
}

// ---- High-level: build CTRL_CMD payload and send ----
uint32_t send_to_stm32_cmd(command_id_t cmd_id, const char* params)
{
    // Build CTRL_CMD payload: [req_id(4)][cmd(1)][flags(1)=0][plen(2)][p...]
    uint8_t pl[64]; // small because our commands are tiny
    uint16_t off = 0;

    wr_u32le(pl + off, (uint32_t)plotter.request_counter); off += 4;
    pl[off++] = (uint8_t)cmd_id;
    pl[off++] = 0; // flags

    // Encode params in binary per-cmd
    uint16_t p_len = 0;
    uint8_t pbuf[16];

    switch (cmd_id) {
        case CMD_SET_COLOR: {
            // params expected like "C<n>" -> keep only index
            uint8_t idx = 0;
            if (params && (params[0]=='C' || params[0]=='c')) {
                int v = atoi(params+1);
                if (v < 0) v = 0;
                if (v > 255) v = 255;
                idx = (uint8_t)v;
            }
            pbuf[0] = idx; p_len = 1;
            break;
        }
        case CMD_GET_CMD_STATUS: {
            // params was "<req_id>" or empty -> 0 means "last known"
            uint32_t q = 0;
            if (params && *params) q = (uint32_t)strtoul(params, NULL, 10);
            wr_u32le(pbuf, q); p_len = 4;
            break;
        }
        default:
            // no params
            p_len = 0;
            break;
    }

    wr_u16le(pl + off, p_len); off += 2;
    if (p_len) { memcpy(pl + off, pbuf, p_len); off += p_len; }

    // Track current command state for active commands
    if (cmd_id < CMD_DUMMY_ACTIVE_COMMANDS_DELIMITER) {
        plotter.current_cmd_status.request_id = plotter.request_counter;
        plotter.current_cmd_status.cmd_state  = PLOTTER_CMD_STATE__UNDEFINED;
    }

    // Keep last command name (optional)
    if (cmd_id < CMD_COUNT) {
        snprintf(plotter.last_command, sizeof(plotter.last_command), "%s", command_names[cmd_id]);
    }

    uart_send_frame(UFR_TYPE_CMD, pl, off);
    return plotter.request_counter++;
}

// ---- RX state machine for framed UART ----
static rx_ctx_t srx;

static void rx_reset(void){ memset(&srx, 0, sizeof(srx)); srx.st = RX_SYNC0; }

static void process_cmd_response(uint8_t status, uint32_t req_id) {
    if (plotter.current_cmd_status.request_id == req_id) {
        if (status == RESP_ACK) {
            plotter.current_cmd_status.cmd_state = PLOTTER_CMD_STATE__RECEIVED;
        } else if (status == RESP_DONE) {
            plotter.current_cmd_status.cmd_state = PLOTTER_CMD_STATE__FINISHED;
            plotter.state.current_cmd = CURRENT_CMD_IDLE;
        } else if (status == RESP_BUSY) {
            // leave as is, UI can poll again
        } else if (status == RESP_ERR) {
            plotter.state.current_cmd = CURRENT_CMD_ERROR;
            plotter.current_cmd_status.cmd_state = PLOTTER_CMD_STATE__FAILED;
        }
    }
}
// ---- Handle CTRL_RESP payload (STM32 -> ESP32) ----
static void handle_ctrl_resp(const uint8_t *p, uint16_t n)
{
    if (n < 8) return; // need at least header
    uint32_t req_id = rd_u32le(p+0);
    uint8_t  cmd_id = p[4];
    uint8_t  status = p[5];
    uint16_t dlen   = rd_u16le(p+6);
    if (8u + dlen != n) return;
    const uint8_t *d = p + 8;

    TickType_t now = xTaskGetTickCount();
    plotter.last_rx_any = now;
    if (cmd_id == CMD_HEARTBEAT) plotter.last_rx_heartbeat = now;

    switch (cmd_id) {
        case CMD_GET_STATUS:
            if (dlen >= 5) {
                uint8_t flags = d[0];
                uint32_t bytes = rd_u32le(d+1);
                bool new_is_cal        = (flags & 0x80u) != 0;
                bool new_is_home       = (flags & 0x40u) != 0;
                bool new_is_terminal   = (flags & 0x20u) != 0;
                bool new_stream_active = (flags & 0x10u) != 0;

                plotter.state.is_calibrated   = new_is_cal ? 1 : 0;
                plotter.state.is_homed        = new_is_home ? 1 : 0;
                plotter.state.bytes_processed = bytes;
                plotter.state.is_processing_cmd = new_stream_active ? 1 : 0;
                plotter.state.is_idle           = (!new_stream_active && new_is_terminal) ? 1 : 0;
            }
            break;
            
        case CMD_GET_CMD_STATUS:
            // Payload format: [query_req_id(4)][cmd_state(1)]
            if (dlen >= 5) {
                uint32_t qid = rd_u32le(d + 0);
                uint8_t  st  = d[4]; // values from CmdState_t enum

                // Update only if it is about our current command
                if (plotter.current_cmd_status.request_id == qid) {
                    plotter.current_cmd_status.cmd_state = (CmdState_t)st;

                    if (st == PLOTTER_CMD_STATE__FINISHED) {
                        plotter.state.current_cmd = CURRENT_CMD_IDLE;
                    } else if (st == PLOTTER_CMD_STATE__FAILED) {
                        plotter.state.current_cmd = CURRENT_CMD_ERROR;
                    }
                }
            }
            break;
        case CMD_GET_POSITION:
            if (dlen >= 8) {
                int32_t xs = (int32_t)rd_u32le(d+0);
                int32_t ys = (int32_t)rd_u32le(d+4);
                plotter.state.x_pos = xs;
                plotter.state.y_pos = ys;
            } else {
                plotter.state.x_pos = -1; plotter.state.y_pos = -1;
            }
            break;

        case CMD_GET_LIMITS:
            if (dlen >= 8) {
                // both MCUs are little-endian; memcpy is fine
                float xmm, ymm;
                memcpy(&xmm, d+0, 4);
                memcpy(&ymm, d+4, 4);
                plotter.state.x_max = xmm;
                plotter.state.y_max = ymm;
            }
            break;

        case CMD_GET_COLOR:
            if (dlen >= 1) {
                plotter.state.current_color = d[0];
            }
            break;

        case CMD_CALIBRATE:
            process_cmd_response(status, req_id);
            if (status == RESP_ACK) {
                plotter.state.current_cmd = CURRENT_CMD_CALIBRATING;
            }
            if (status == RESP_DONE) {
                send_to_stm32_cmd(CMD_GET_LIMITS, NULL);
            }
            break;

        case CMD_HOME:
            process_cmd_response(status, req_id);
            if (status == RESP_ACK) {
                plotter.state.current_cmd = CURRENT_CMD_HOMING;
            }
            break;

        case CMD_DRAW_BEGIN:
            process_cmd_response(status, req_id);
            if (status == RESP_ACK) {
                plotter.state.current_cmd = CURRENT_CMD_DRAWING;
            }
            break;

        case CMD_PEN_UP:
        case CMD_PEN_DOWN:
        case CMD_SET_COLOR:
        case CMD_EMERGENCY_STOP:
            process_cmd_response(status, req_id);
            // translate status to old cmd_state
            break;

        default:
            break;
    }

    plotter.state.is_connected = 1;
}

static void handle_debug_text(const uint8_t *p, uint16_t n)
{
    // ensure printable
    char line[200];
    uint16_t m = (n < sizeof(line)-1) ? n : (sizeof(line)-1);
    memcpy(line, p, m); line[m] = 0;
    printf("STM32[DBG]: %s\n", line);
}

// ---- UART RX task (binary) ----
void stm32_uart_task(void *pvParameters)
{
    printf("UART: framed RX loop\n");
    rx_reset();

    uint8_t b;
    for (;;) {
        int got = uart_read_bytes(STM32_UART_PORT, &b, 1, pdMS_TO_TICKS(100));
        if (got == 1) {
            switch (srx.st) {
                case RX_SYNC0: srx.st = (b==UFR_SYNC0) ? RX_SYNC1 : RX_SYNC0; break;
                case RX_SYNC1: srx.st = (b==UFR_SYNC1) ? RX_TYPE  : RX_SYNC0; break;
                case RX_TYPE:  srx.type = b; srx.st = RX_LEN0; break;
                case RX_LEN0:  srx.len = b; srx.st = RX_LEN1; break;
                case RX_LEN1:  srx.len |= (uint16_t)b<<8; srx.idx = 0; srx.crc_calc = 0xFFFF; srx.st = (srx.len ? RX_PAYLOAD : RX_CRC_HI); break;
                case RX_PAYLOAD:
                    if (srx.idx < sizeof(srx.payload)) {
                        srx.payload[srx.idx++] = b;
                        srx.crc_calc = crc16_ccitt_step(srx.crc_calc, b);
                        if (srx.idx >= srx.len) srx.st = RX_CRC_HI;
                    } else {
                        // overflow -> resync
                        rx_reset();
                    }
                    break;
                case RX_CRC_HI:
                    srx.crc_hi = b;
                    srx.st = RX_CRC_LO;
                    break;
                case RX_CRC_LO: {
                    uint16_t crc_recv = ((uint16_t)srx.crc_hi<<8) | b;
                    if (crc_recv == srx.crc_calc) {
                        if (srx.type == UFR_TYPE_RESP)      handle_ctrl_resp(srx.payload, srx.len);
                        else if (srx.type == UFR_TYPE_DEBUG)handle_debug_text(srx.payload, srx.len);
                        // ignore unknown types
                    }
                    rx_reset();
                    break;
                }
            }
        }
        taskYIELD();
    }
}
// ========================= END UART binary protocol (ESP32 side) =========================

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
        if ((now - plotter.last_position_request) >= pdMS_TO_TICKS(500)) {
            send_to_stm32_cmd(CMD_GET_POSITION, NULL);
            plotter.last_position_request = now;
        }

        if ((now - plotter.last_cmd_status_request) >= pdMS_TO_TICKS(500)) {
            plotter.last_cmd_status_request = now;
            const CmdState_t cs = plotter.current_cmd_status.cmd_state;
            const bool cmd_active = (cs == PLOTTER_CMD_STATE__RECEIVED || cs == PLOTTER_CMD_STATE__EXECUTING);

            if (cmd_active) {
                char p[16];
                snprintf(p, sizeof(p), "%u", (uint32_t)plotter.current_cmd_status.request_id);
                send_to_stm32_cmd(CMD_GET_CMD_STATUS, p);
                plotter.last_cmd_status_request = now;
            }
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
            printf("KEYPAD: Set color #3\n");
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
    oled_init();
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
    xTaskCreatePinnedToCore(oled_task, "oled_display", 4096, NULL, 1, NULL, 1);
}
