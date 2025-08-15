#pragma once
#include <stddef.h>
#include <stdint.h>
#include <stdbool.h>

typedef struct {
    uint8_t *mem;
    size_t   size;
    size_t   head; // write
    size_t   tail; // read
} rb_t;

// Инициализация поверх внешнего массива mem[size]
void   rb_init(rb_t *rb, uint8_t *mem, size_t size);

// Свободно/занято
size_t rb_free_space(const rb_t *rb);
size_t rb_used(const rb_t *rb);

// Запись: либо весь пакет, либо 0 (ничего не пишет, если места мало)
size_t rb_write_all_or_drop(rb_t *rb, const uint8_t *src, size_t len);

// Чтение: читает ровно len; если недостаточно данных — вернёт 0 и ничего не читает
size_t rb_read_exact(rb_t *rb, uint8_t *dst, size_t len);
