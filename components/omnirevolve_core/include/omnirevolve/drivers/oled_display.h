#pragma once
#include <stdint.h>
#include "omnirevolve/esp32_core/esp32_core.h"

#ifdef __cplusplus
extern "C" {
#endif

typedef enum {
    WIFI_STATE_DISCONNECTED = 0,
    WIFI_STATE_CONNECTING,
    WIFI_STATE_CONNECTED,
    WIFI_STATE_AP_MODE,
    WIFI_STATE_ERROR
} wifi_state_t;

void oled_init(void);
void oled_start_task(void);

void oled_update_wifi_status(wifi_state_t state, const char *ssid, const char *ip, int8_t rssi);
void oled_show_message(const char *message, uint32_t duration_ms);

// Provided by core; set a reader before starting task to render plotter status.
void oled_set_state_reader(plotter_state_reader_t reader);

#ifdef __cplusplus
}
#endif
