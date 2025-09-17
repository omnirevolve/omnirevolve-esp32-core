#pragma once
#include <stdint.h>
#include "omnirevolve/esp32_core/esp32_core.h"

#ifdef __cplusplus
extern "C" {
#endif

void stm_link_init(void);
void stm_link_config_ready_pin(ready_signal_isr_callback cb);
void stm_link_send_draw_data(const uint8_t* data, uint32_t len);

#ifdef __cplusplus
}
#endif
