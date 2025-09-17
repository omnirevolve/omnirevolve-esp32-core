#pragma once
#include <stdint.h>
#include "omnirevolve/protocol/cmd_ids.h"

void or_uart_init(void);
uint32_t or_send_cmd(command_id_t cmd_id, const char* params);
void or_uart_task(void *pvParameters);
