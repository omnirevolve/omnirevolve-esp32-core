#ifndef SPI_FEEDER_H
#define SPI_FEEDER_H

#include <stdint.h>
#include "ring_buffer.h"

// Запуск SPI feeder задачи
void spi_feeder_start(rb_t* rb);

// Статистика
uint32_t spi_feeder_get_total_sent(void);
uint32_t spi_feeder_get_chunks_sent(void);

#endif // SPI_FEEDER_H
