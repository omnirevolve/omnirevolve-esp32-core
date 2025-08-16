#ifndef SPI_FEEDER_H
#define SPI_FEEDER_H

#include <stdint.h>
#include <stdbool.h>
#include "ring_buffer.h"

// Запуск SPI-feeder задачи
void     spi_feeder_start(rb_t* rb);

// Сообщить фидеру о завершении сессии:
// он перестанет ждать новых данных, дожмёт остаток (пэддинг до 2048 B) и перейдёт в idle.
void     spi_feeder_request_finish(void);

// true, когда буфер опустошён и последний (неполный) блок отправлен с пэддингом.
bool     spi_feeder_is_flushed(void);

// Статистика
uint32_t spi_feeder_get_total_sent(void);
uint32_t spi_feeder_get_chunks_sent(void);

#endif // SPI_FEEDER_H
