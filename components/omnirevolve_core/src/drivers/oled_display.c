// SSD1309 OLED via u8g2 (VSPI). Renders Wi‑Fi and plotter status.
#include <stdio.h>
#include <string.h>
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "driver/spi_master.h"
#include "driver/gpio.h"
#include "esp_wifi.h"
#include "esp_netif.h"
#include "u8g2.h"
#include "omnirevolve/drivers/oled_display.h"

#include <inttypes.h>
#if __has_include("esp_rom_sys.h")
  #include "esp_rom_sys.h"
  #define udelay_us(x) esp_rom_delay_us(x)
#else
  #include "esp32/rom/ets_sys.h"
  #define udelay_us(x) ets_delay_us(x)
#endif

// VSPI pins for OLED
#ifndef OLED_PIN_MOSI
#define OLED_PIN_MOSI   23
#endif
#ifndef OLED_PIN_SCK
#define OLED_PIN_SCK    18
#endif
#ifndef OLED_PIN_CS
#define OLED_PIN_CS     5
#endif
#ifndef OLED_PIN_DC
#define OLED_PIN_DC     17
#endif
#ifndef OLED_PIN_RST
#define OLED_PIN_RST    16
#endif

#define OLED_SPI_HOST   VSPI_HOST
#define OLED_WIDTH      128
#define OLED_HEIGHT     64

static u8g2_t u8g2;
static spi_device_handle_t s_oled_spi = NULL;
static volatile uint8_t s_inited = 0;

static wifi_state_t s_wifi_state = WIFI_STATE_DISCONNECTED;
static char s_wifi_ssid[32] = {0};
static char s_wifi_ip[16] = {0};
static int8_t s_wifi_rssi = -100;

static char s_temp_msg[64] = {0};
static TickType_t s_msg_deadline = 0;

static plotter_state_reader_t s_state_reader = NULL;

void oled_set_state_reader(plotter_state_reader_t reader) { s_state_reader = reader; }

// --- u8g2 callbacks (driver toggles CS) ---
static uint8_t u8x8_byte_hw_spi(u8x8_t *u8x8, uint8_t msg, uint8_t arg_int, void *arg_ptr) {
    (void)u8x8;
    switch (msg) {
        case U8X8_MSG_BYTE_INIT: return 1;
        case U8X8_MSG_BYTE_SET_DC:
            if (OLED_PIN_DC >= 0) gpio_set_level(OLED_PIN_DC, arg_int);
            return 1;
        case U8X8_MSG_BYTE_SEND: {
            spi_transaction_t t = { .length = 8 * arg_int, .tx_buffer = arg_ptr };
            return (spi_device_polling_transmit(s_oled_spi, &t) == ESP_OK) ? 1 : 0;
        }
        default: return 0;
    }
}

static uint8_t u8x8_gpio_and_delay(u8x8_t *u8x8, uint8_t msg, uint8_t arg_int, void *arg_ptr) {
    (void)u8x8; (void)arg_ptr;
    switch (msg) {
        case U8X8_MSG_GPIO_AND_DELAY_INIT:
            if (OLED_PIN_DC  >= 0) { gpio_set_direction(OLED_PIN_DC,  GPIO_MODE_OUTPUT); }
            if (OLED_PIN_RST >= 0) { gpio_set_direction(OLED_PIN_RST, GPIO_MODE_OUTPUT); gpio_set_level(OLED_PIN_RST, 1); }
            return 1;
        case U8X8_MSG_DELAY_MILLI: vTaskDelay(pdMS_TO_TICKS(arg_int)); return 1;
        case U8X8_MSG_DELAY_10MICRO: udelay_us(10); return 1;
        case U8X8_MSG_GPIO_RESET:
            if (OLED_PIN_RST >= 0) gpio_set_level(OLED_PIN_RST, arg_int);
            return 1;
        default: return 1;
    }
}

static void oled_spi_init(void) {
    spi_bus_config_t buscfg = {
        .mosi_io_num = OLED_PIN_MOSI,
        .miso_io_num = -1,
        .sclk_io_num = OLED_PIN_SCK,
        .quadwp_io_num = -1,
        .quadhd_io_num = -1,
        .max_transfer_sz = 1024
    };
    ESP_ERROR_CHECK(spi_bus_initialize(OLED_SPI_HOST, &buscfg, 2));
    spi_device_interface_config_t devcfg = {
        .mode = 0,
        .clock_speed_hz = 8000000,
        .spics_io_num = OLED_PIN_CS,
        .queue_size = 7
    };
    ESP_ERROR_CHECK(spi_bus_add_device(OLED_SPI_HOST, &devcfg, &s_oled_spi));
}

static void oled_hw_reset(void) {
    if (OLED_PIN_RST < 0) return;
    gpio_set_level(OLED_PIN_RST, 1);
    vTaskDelay(pdMS_TO_TICKS(1));
    gpio_set_level(OLED_PIN_RST, 0);
    vTaskDelay(pdMS_TO_TICKS(10));
    gpio_set_level(OLED_PIN_RST, 1);
    vTaskDelay(pdMS_TO_TICKS(10));
}

void oled_init(void) {
    if (s_inited) return;
    oled_spi_init();
    oled_hw_reset();
    u8g2_Setup_ssd1309_128x64_noname0_f(&u8g2, U8G2_R0, u8x8_byte_hw_spi, u8x8_gpio_and_delay);
    u8g2_InitDisplay(&u8g2);
    u8g2_SetPowerSave(&u8g2, 0);
    u8g2_ClearBuffer(&u8g2);
    u8g2_SetFont(&u8g2, u8g2_font_4x6_mf);
    s_inited = 1;
    printf("[oled] ready\n");
    u8g2_DrawStr(&u8g2, 0, 10, "Start uROS-agent on PC-node");
    u8g2_DrawStr(&u8g2, 0, 18, "and restart me (ESP32)");
    u8g2_SendBuffer(&u8g2);
}

void oled_update_wifi_status(wifi_state_t state, const char *ssid, const char *ip, int8_t rssi) {
    if (!s_inited) return;
    s_wifi_state = state;
    if (ssid) strncpy(s_wifi_ssid, ssid, sizeof(s_wifi_ssid) - 1);
    if (ip)   strncpy(s_wifi_ip, ip, sizeof(s_wifi_ip) - 1);
    s_wifi_rssi = rssi;
}

void oled_show_message(const char *message, uint32_t duration_ms) {
    if (!s_inited) return;
    if (message) {
        strncpy(s_temp_msg, message, sizeof(s_temp_msg) - 1);
        s_msg_deadline = xTaskGetTickCount() + pdMS_TO_TICKS(duration_ms);
    } else {
        s_temp_msg[0] = '\0';
        s_msg_deadline = 0;
    }
}

static void poll_wifi(void) {
    wifi_state_t state = WIFI_STATE_DISCONNECTED;
    char ssid[33] = {0};
    char ip_str[16] = {0};
    int8_t rssi = -100;

    wifi_ap_record_t ap_info;
    esp_netif_ip_info_t ip_info;
    esp_netif_t *sta = esp_netif_get_handle_from_ifkey("WIFI_STA_DEF");
    esp_netif_t *ap  = esp_netif_get_handle_from_ifkey("WIFI_AP_DEF");

    if (esp_wifi_sta_get_ap_info(&ap_info) == ESP_OK) {
        state = WIFI_STATE_CONNECTED;
        memcpy(ssid, ap_info.ssid, sizeof(ap_info.ssid));
        ssid[32] = '\0';
        rssi = ap_info.rssi;
        if (sta && esp_netif_get_ip_info(sta, &ip_info) == ESP_OK) {
            sprintf(ip_str, IPSTR, IP2STR(&ip_info.ip));
        }
    } else {
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
    oled_update_wifi_status(state, ssid, ip_str, rssi);
}

static const char* task_code(uint8_t current_cmd) {
    switch (current_cmd) {
        case 0: return "IDL";
        case 1: return "CAL";
        case 2: return "HOM";
        case 3: return "DRW";
        case 4: return "ERR";
        default: return "?";
    }
}

static void draw_ui(void) {
    if (!s_inited) return;
    u8g2_ClearBuffer(&u8g2);

    if (s_msg_deadline > xTaskGetTickCount() && s_temp_msg[0] != '\0') {
        int w = u8g2_GetStrWidth(&u8g2, s_temp_msg);
        u8g2_DrawStr(&u8g2, (OLED_WIDTH - w) / 2, 32, s_temp_msg);
        u8g2_SendBuffer(&u8g2);
        return;
    }

    int y = 7;
    const int dy = 8;
    char buf[64];

    if (s_wifi_state != WIFI_STATE_CONNECTED) {
        const char *wifi_str = "WiFi: -";
        switch (s_wifi_state) {
            case WIFI_STATE_DISCONNECTED: wifi_str = "WiFi: d/c"; break;
            case WIFI_STATE_CONNECTING:   wifi_str = "WiFi: ..."; break;
            case WIFI_STATE_AP_MODE:      wifi_str = "WiFi: AP"; break;
            case WIFI_STATE_ERROR:        wifi_str = "WiFi: ER"; break;
            default: wifi_str = "WiFi: ?"; break;
        }
        u8g2_DrawStr(&u8g2, 0, y, wifi_str); y += dy;
    } else {
        snprintf(buf, sizeof(buf), "WiFi: %d dBm", s_wifi_rssi);
        u8g2_DrawStr(&u8g2, 0, y, buf); y += dy;
    }

    plotter_state_t ps = {0};
    if (s_state_reader) s_state_reader(&ps);

    if (!ps.is_connected) {
        snprintf(buf, sizeof(buf), "STM32: Disconnected");
        u8g2_DrawStr(&u8g2, 0, y, buf); y += dy;
    } else {
        snprintf(buf, sizeof(buf), "STM32: CAL:%d HOM:%d CMD:%s",
                 ps.is_calibrated ? 1 : 0,
                 ps.is_homed ? 1 : 0,
                 task_code(ps.current_cmd));
        u8g2_DrawStr(&u8g2, 0, y, buf); y += dy;

        snprintf(buf, sizeof(buf), "Pos: X%ld Y%ld", (long)ps.x_pos, (long)ps.y_pos);
        u8g2_DrawStr(&u8g2, 0, y, buf); y += dy;

        if (ps.current_cmd == 3) {
            snprintf(buf, sizeof(buf), "DRW: %" PRIu32, (uint32_t)ps.bytes_processed);
            u8g2_DrawStr(&u8g2, 0, y, buf); y += dy;
        }
    }
    u8g2_SendBuffer(&u8g2);
}

static void oled_task(void *arg) {
    (void)arg;
    printf("[oled] task\n");
    const TickType_t draw_period = pdMS_TO_TICKS(200);
    const TickType_t wifi_period = pdMS_TO_TICKS(2000);
    TickType_t last_draw = xTaskGetTickCount();
    TickType_t last_wifi = last_draw;

    for (;;) {
        if (!s_inited) { vTaskDelay(pdMS_TO_TICKS(200)); continue; }
        TickType_t now = xTaskGetTickCount();
        if ((now - last_wifi) >= wifi_period) { poll_wifi(); last_wifi = now; }
        if ((now - last_draw) >= draw_period) { draw_ui(); last_draw = now; }
        vTaskDelay(pdMS_TO_TICKS(200));
    }
}

void oled_start_task(void) {
    xTaskCreatePinnedToCore(oled_task, "oled_display", 4096, NULL, 1, NULL, 1);
}
