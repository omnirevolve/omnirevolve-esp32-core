#include <stdio.h>
#include <string.h>
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "driver/spi_master.h"
#include "driver/gpio.h"
#include "esp_wifi.h"
#include "esp_netif.h"
#include "u8g2.h"

#include "omnirevolve/core/state.h"
#include "omnirevolve/hal/pins.h"
#include "omnirevolve/drivers/oled_display.h"

static u8g2_t u8g2;
static oled_wifi_info_t wifi_info = {0};
static char temp_msg[64] = {0};
static TickType_t msg_deadline = 0;
static uint8_t s_inited = 0;
static spi_device_handle_t s_oled_spi = NULL;

static uint8_t u8x8_byte_hw_spi(u8x8_t *u8x8, uint8_t msg, uint8_t arg_int, void *arg_ptr) {
    (void)u8x8;
    switch(msg){
        case U8X8_MSG_BYTE_INIT: return 1;
        case U8X8_MSG_BYTE_SET_DC:
            if (OLED_PIN_DC>=0) gpio_set_level(OLED_PIN_DC, arg_int);
            return 1;
        case U8X8_MSG_BYTE_SEND: {
            spi_transaction_t t = { .length = 8*arg_int, .tx_buffer = arg_ptr };
            return (spi_device_polling_transmit(s_oled_spi, &t) == ESP_OK) ? 1 : 0;
        }
        default: return 0;
    }
}

static uint8_t u8x8_gpio_and_delay(u8x8_t *u8x8, uint8_t msg, uint8_t arg_int, void *arg_ptr){
    (void)u8x8; (void)arg_ptr;
    switch(msg){
        case U8X8_MSG_GPIO_AND_DELAY_INIT:
            if (OLED_PIN_DC>=0) { gpio_set_direction(OLED_PIN_DC, GPIO_MODE_OUTPUT); }
            if (OLED_PIN_RST>=0){ gpio_set_direction(OLED_PIN_RST,GPIO_MODE_OUTPUT); gpio_set_level(OLED_PIN_RST,1); }
            return 1;
        case U8X8_MSG_DELAY_MILLI: vTaskDelay(pdMS_TO_TICKS(arg_int)); return 1;
        case U8X8_MSG_DELAY_10MICRO: ets_delay_us(10); return 1;
        case U8X8_MSG_GPIO_RESET:
            if (OLED_PIN_RST>=0) gpio_set_level(OLED_PIN_RST, arg_int);
            return 1;
        default: return 1;
    }
}

static void oled_spi_bus_init(void){
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

static void oled_hw_reset(void){
    if (OLED_PIN_RST<0) return;
    gpio_set_level(OLED_PIN_RST,1);
    vTaskDelay(pdMS_TO_TICKS(1));
    gpio_set_level(OLED_PIN_RST,0);
    vTaskDelay(pdMS_TO_TICKS(10));
    gpio_set_level(OLED_PIN_RST,1);
    vTaskDelay(pdMS_TO_TICKS(10));
}

void or_oled_init(void){
    if (s_inited) return;
    oled_spi_bus_init();
    oled_hw_reset();
    u8g2_Setup_ssd1309_128x64_noname0_f(&u8g2, U8G2_R0, u8x8_byte_hw_spi, u8x8_gpio_and_delay);
    u8g2_InitDisplay(&u8g2);
    u8g2_SetPowerSave(&u8g2, 0);
    u8g2_ClearBuffer(&u8g2);
    u8g2_SetFont(&u8g2, u8g2_font_4x6_mf);
    s_inited = 1;
    u8g2_DrawStr(&u8g2, 0, 10, "Start uROS-agent on PC-node");
    u8g2_DrawStr(&u8g2, 0, 18, "and restart me (ESP32)");
    u8g2_SendBuffer(&u8g2);
}

void or_oled_update_wifi(wifi_state_t state, const char *ssid, const char *ip, int8_t rssi){
    if (!s_inited) return;
    wifi_info.wifi_state = state;
    if (ssid) strncpy(wifi_info.wifi_ssid, ssid, sizeof(wifi_info.wifi_ssid)-1);
    if (ip)   strncpy(wifi_info.ip_address, ip, sizeof(wifi_info.ip_address)-1);
    wifi_info.wifi_rssi = rssi;
}

void or_oled_show_message(const char *message, uint32_t duration_ms){
    if (!s_inited) return;
    if (message){
        strncpy(temp_msg, message, sizeof(temp_msg)-1);
        msg_deadline = xTaskGetTickCount() + pdMS_TO_TICKS(duration_ms);
    } else {
        temp_msg[0] = 0;
        msg_deadline = 0;
    }
}

static void poll_wifi(void){
    wifi_state_t state = WIFI_STATE_DISCONNECTED;
    char ssid[33]={0}; char ip[16]={0}; int8_t rssi=-100;
    wifi_ap_record_t ap_info; esp_netif_ip_info_t ip_info;
    esp_netif_t *sta = esp_netif_get_handle_from_ifkey("WIFI_STA_DEF");
    esp_netif_t *ap  = esp_netif_get_handle_from_ifkey("WIFI_AP_DEF");
    if (esp_wifi_sta_get_ap_info(&ap_info)==ESP_OK){
        state = WIFI_STATE_CONNECTED;
        memcpy(ssid, ap_info.ssid, sizeof(ap_info.ssid));
        ssid[32]='\0'; rssi=ap_info.rssi;
        if (sta && esp_netif_get_ip_info(sta,&ip_info)==ESP_OK){
            sprintf(ip, IPSTR, IP2STR(&ip_info.ip));
        }
    } else {
        wifi_mode_t mode;
        if (esp_wifi_get_mode(&mode)==ESP_OK){
            if (mode==WIFI_MODE_AP){
                state=WIFI_STATE_AP_MODE; strcpy(ssid, "AP Mode");
                if (ap && esp_netif_get_ip_info(ap,&ip_info)==ESP_OK){
                    sprintf(ip, IPSTR, IP2STR(&ip_info.ip));
                }
            } else if (mode==WIFI_MODE_STA || mode==WIFI_MODE_APSTA){
                state=WIFI_STATE_CONNECTING; strcpy(ssid, "Scanning...");
            } else {
                state=WIFI_STATE_DISCONNECTED;
            }
        } else state=WIFI_STATE_ERROR;
    }
    or_oled_update_wifi(state, ssid, ip, rssi);
}

static const char* cur_task_str(uint8_t cur){
    switch(cur){
        case 0: return "IDL";
        case 1: return "CAL";
        case 2: return "HOM";
        case 3: return "DRW";
        case 4: return "ERR";
        default: return "?";
    }
}

static void draw_screen(void){
    if (!s_inited) return;
    u8g2_ClearBuffer(&u8g2);

    if (msg_deadline > xTaskGetTickCount() && temp_msg[0]){
        int w = u8g2_GetStrWidth(&u8g2, temp_msg);
        u8g2_DrawStr(&u8g2, (OLED_WIDTH - w)/2, 32, temp_msg);
        u8g2_SendBuffer(&u8g2);
        return;
    }

    int y=7; const int dy=8; char buf[64];
    if (wifi_info.wifi_state != WIFI_STATE_CONNECTED){
        const char *s = "WiFi: -";
        if      (wifi_info.wifi_state==WIFI_STATE_DISCONNECTED) s="WiFi: d/c";
        else if (wifi_info.wifi_state==WIFI_STATE_CONNECTING)   s="WiFi: ...";
        else if (wifi_info.wifi_state==WIFI_STATE_AP_MODE)      s="WiFi: AP";
        else if (wifi_info.wifi_state==WIFI_STATE_ERROR)        s="WiFi: ER";
        u8g2_DrawStr(&u8g2, 0, y, s); y+=dy;
    } else {
        snprintf(buf,sizeof(buf), "WiFi: %d dBm", wifi_info.wifi_rssi);
        u8g2_DrawStr(&u8g2, 0, y, buf); y+=dy;
    }

    if (!g_or_plotter.state.is_connected){
        u8g2_DrawStr(&u8g2, 0, y, "STM32: Disconnected"); y+=dy;
    } else {
        snprintf(buf,sizeof(buf), "STM32: CAL:%d HOM:%d CMD:%s",
                g_or_plotter.state.is_calibrated?1:0,
                g_or_plotter.state.is_homed?1:0,
                cur_task_str(g_or_plotter.state.current_cmd));
        u8g2_DrawStr(&u8g2, 0, y, buf); y+=dy;

        snprintf(buf,sizeof(buf), "Pos: X%ld Y%ld", (long)g_or_plotter.state.x_pos, (long)g_or_plotter.state.y_pos);
        u8g2_DrawStr(&u8g2, 0, y, buf); y+=dy;

        if (g_or_plotter.state.current_cmd == 3 /*DRW*/){
            snprintf(buf,sizeof(buf), "DRW: %u", (unsigned)g_or_plotter.state.bytes_processed);
            u8g2_DrawStr(&u8g2, 0, y, buf); y+=dy;
        }
    }
    u8g2_SendBuffer(&u8g2);
}

void or_oled_task(void *pv){
    (void)pv;
    printf("[OLED] task started\n");
    const TickType_t draw_period = pdMS_TO_TICKS(200);
    const TickType_t wifi_period = pdMS_TO_TICKS(2000);
    TickType_t last_draw = xTaskGetTickCount();
    TickType_t last_wifi = last_draw;
    for(;;){
        if (!s_inited){ vTaskDelay(pdMS_TO_TICKS(200)); continue; }
        TickType_t now = xTaskGetTickCount();
        if ((now-last_wifi) >= wifi_period){ poll_wifi(); last_wifi = now; }
        if ((now-last_draw) >= draw_period){ draw_screen(); last_draw = now; }
        vTaskDelay(pdMS_TO_TICKS(50));
    }
}
