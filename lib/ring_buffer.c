#include "ring_buffer.h"
#include <string.h>
#include "freertos/FreeRTOS.h"
#include "freertos/portmacro.h"

static portMUX_TYPE s_mux = portMUX_INITIALIZER_UNLOCKED;

static inline size_t _min(size_t a, size_t b){ return a < b ? a : b; }

void rb_init(rb_t *rb, uint8_t *mem, size_t size){
    rb->mem = mem;
    rb->size = size;
    rb->head = 0;
    rb->tail = 0;
}

size_t rb_used(const rb_t *rb){
    size_t head = rb->head, tail = rb->tail, size = rb->size;
    return (head >= tail) ? (head - tail) : (size - (tail - head));
}

size_t rb_free_space(const rb_t *rb){
    return rb->size - rb_used(rb);
}

size_t rb_write_all_or_drop(rb_t *rb, const uint8_t *src, size_t len){
    if (len == 0) return 0;
    size_t written = 0;

    portENTER_CRITICAL(&s_mux);
    size_t free = rb_free_space(rb);
    if (free >= len){
        size_t head = rb->head;
        size_t size = rb->size;

        size_t first = _min(len, size - head);
        memcpy(rb->mem + head, src, first);
        size_t remain = len - first;
        if (remain){
            memcpy(rb->mem, src + first, remain);
        }
        rb->head = (head + len) % size;
        written = len;
    }
    portEXIT_CRITICAL(&s_mux);

    return written;
}

size_t rb_read_exact(rb_t *rb, uint8_t *dst, size_t len){
    if (len == 0) return 0;
    size_t read = 0;

    portENTER_CRITICAL(&s_mux);
    size_t used = rb_used(rb);
    if (used >= len){
        size_t tail = rb->tail;
        size_t size = rb->size;

        size_t first = _min(len, size - tail);
        memcpy(dst, rb->mem + tail, first);
        size_t remain = len - first;
        if (remain){
            memcpy(dst + first, rb->mem, remain);
        }
        rb->tail = (tail + len) % size;
        read = len;
    }
    portEXIT_CRITICAL(&s_mux);

    return read;
}
