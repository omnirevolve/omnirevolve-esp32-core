#pragma once
#include <stdint.h>
#include "omnirevolve/protocol/cmd_ids.h"

typedef void (*ready_signal_isr_callback)(void *arg);

void or_stm_spi_init(void);
void or_stm_config_ready_irq(ready_signal_isr_callback isr_cb);
void or_stm_send_stream(const uint8_t* data, uint32_t len);
