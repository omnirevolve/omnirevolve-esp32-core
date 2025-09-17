#pragma once
#include "driver/uart.h"
#include "driver/spi_master.h"

// STM32 SPI (HSPI)
#define STM32_SPI_HOST    HSPI_HOST
#define STM32_SPI_MOSI    13
#define STM32_SPI_MISO    12 // not used
#define STM32_SPI_CLK     14
#define STM32_READY_PIN   15

// STM32 UART2
#define STM32_UART_PORT   UART_NUM_2
#define STM32_TX_PIN      25
#define STM32_RX_PIN      26

// OLED (VSPI)
#define OLED_SPI_HOST     VSPI_HOST
#define OLED_PIN_MOSI     23
#define OLED_PIN_SCK      18
#define OLED_PIN_CS       5
#define OLED_PIN_DC       17
#define OLED_PIN_RST      16
#define OLED_WIDTH        128
#define OLED_HEIGHT       64

// 3x3 keypad
#define KEYPAD_ROWS 3
#define KEYPAD_COLS 3
static const int or_keypad_row_pins[KEYPAD_ROWS] = {32, 33, 27};
static const int or_keypad_col_pins[KEYPAD_COLS] = {22, 4, 19};
