#pragma once

// === Буферы / порции ===
#define RB_SIZE_BYTES        (16 * 1024)   // кольцевой буфер ESP32
#define SPI_PORTION_SIZE     512           // внутренняя "порция" вычитки

// === Ниббл-биты (должны совпадать с STM32) ===
#define NIBBLE_X_STEP   (1u << 0)
#define NIBBLE_X_DIR    (1u << 1)   // 0 = to MIN, 1 = to MAX
#define NIBBLE_Y_STEP   (1u << 2)
#define NIBBLE_Y_DIR    (1u << 3)
