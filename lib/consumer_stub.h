#pragma once
#include <stdint.h>
#include "ring_buffer.h"

void     consumer_stub_start(rb_t *rb);
uint32_t consumer_stub_consumed_bytes(void);
