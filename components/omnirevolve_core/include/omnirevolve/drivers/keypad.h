#pragma once
#include <stdint.h>

#ifdef __cplusplus
extern "C" {
#endif

typedef void (*keypad_key_cb_t)(char key);

void keypad_init(void);
void keypad_start_task(void);
void keypad_set_callback(keypad_key_cb_t cb);

#ifdef __cplusplus
}
#endif
