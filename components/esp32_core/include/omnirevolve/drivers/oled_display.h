#pragma once
#include <stdint.h>

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

void or_oled_init(void);
void or_oled_task(void *pvParameters);
void or_oled_update_wifi(wifi_state_t state, const char *ssid, const char *ip, int8_t rssi);
void or_oled_show_message(const char *message, uint32_t duration_ms);
