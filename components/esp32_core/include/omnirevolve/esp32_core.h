#pragma once
#include <stdint.h>
#include "omnirevolve/protocol/cmd_ids.h"
#include "omnirevolve/core/state.h"

#ifdef __cplusplus
extern "C" {
#endif

typedef void (*ready_signal_isr_callback)(void *arg);

void or_plotter_init_sync(ready_signal_isr_callback cb);
void or_plotter_start_tasks(void);

uint32_t or_send_cmd(command_id_t cmd_id, const char* params);
void or_send_stream_chunk(const uint8_t* data, uint32_t len);
void or_get_state(plotter_state_t *ps);

#ifdef __cplusplus
}
#endif
