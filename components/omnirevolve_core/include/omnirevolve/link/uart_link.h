#pragma once
#include <stdint.h>
#include "driver/uart.h"
#include "omnirevolve/protocol/cmd_ids.h"

#ifdef __cplusplus
extern "C" {
#endif

typedef void (*uart_resp_cb_t)(const uint8_t *payload, uint16_t len);
typedef void (*uart_debug_cb_t)(const char *line);

void uart_link_init(uart_port_t port, int tx_pin, int rx_pin, int baud);
void uart_link_start(void);
void uart_link_set_callbacks(uart_resp_cb_t resp_cb, uart_debug_cb_t dbg_cb);

uint32_t uart_link_send_cmd(command_id_t cmd_id, const char* params);

#ifdef __cplusplus
}
#endif
