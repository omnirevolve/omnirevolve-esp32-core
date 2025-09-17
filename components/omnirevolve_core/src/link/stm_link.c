// SPI stream to STM32 (HSPI), plus READY pin ISR hook.
#include <stdio.h>
#include "driver/spi_master.h"
#include "driver/gpio.h"
#include "esp_err.h"
#include "omnirevolve/protocol/cmd_ids.h"
#include "omnirevolve/esp32_core/esp32_core.h"
#include "omnirevolve/link/stm_link.h"

#define STM32_SPI_HOST    HSPI_HOST
#define STM32_SPI_MOSI    13
#define STM32_SPI_MISO    -1
#define STM32_SPI_CLK     14
#define STM32_READY_PIN   15

static spi_device_handle_t s_stm32 = NULL;

static void stm32_spi_init(void) {
    spi_bus_config_t buscfg = {
        .mosi_io_num = STM32_SPI_MOSI,
        .miso_io_num = STM32_SPI_MISO,
        .sclk_io_num = STM32_SPI_CLK,
        .quadwp_io_num = -1,
        .quadhd_io_num = -1,
        .max_transfer_sz = SPI_CHUNK_SIZE,
        .flags = SPICOMMON_BUSFLAG_MASTER
    };
    esp_err_t ret = spi_bus_initialize(STM32_SPI_HOST, &buscfg, 1);
    if (ret != ESP_OK) {
        printf("[stm_link] spi_bus_initialize failed: %s\n", esp_err_to_name(ret));
        return;
    }
    spi_device_interface_config_t devcfg = {
        .mode = 0,
        .clock_speed_hz = 8 * 1000 * 1000,
        .spics_io_num = -1,
        .queue_size = 8,
    };
    ret = spi_bus_add_device(STM32_SPI_HOST, &devcfg, &s_stm32);
    if (ret != ESP_OK) {
        printf("[stm_link] spi_bus_add_device failed: %s\n", esp_err_to_name(ret));
        return;
    }
    printf("[stm_link] HSPI ready (MOSI=%d CLK=%d READY=%d)\n", STM32_SPI_MOSI, STM32_SPI_CLK, STM32_READY_PIN);
}

void stm_link_init(void) { stm32_spi_init(); }

void stm_link_config_ready_pin(ready_signal_isr_callback cb) {
    gpio_config_t io = {
        .pin_bit_mask = 1ULL << STM32_READY_PIN,
        .mode = GPIO_MODE_INPUT,
        .pull_up_en = GPIO_PULLUP_DISABLE,
        .pull_down_en = GPIO_PULLDOWN_DISABLE,
        .intr_type = GPIO_INTR_ANYEDGE
    };
    gpio_config(&io);
    esp_err_t err = gpio_install_isr_service(ESP_INTR_FLAG_IRAM);
    if (err != ESP_OK && err != ESP_ERR_INVALID_STATE) { ESP_ERROR_CHECK(err); }
    ESP_ERROR_CHECK(gpio_isr_handler_add(STM32_READY_PIN, cb, NULL));
    ESP_ERROR_CHECK(gpio_intr_enable(STM32_READY_PIN));
}

void stm_link_send_draw_data(const uint8_t* data, uint32_t len) {
    if (!s_stm32 || !data || len == 0) {
        printf("[stm_link] invalid draw data\n");
        return;
    }
    spi_transaction_t t = { .length = len * 8, .tx_buffer = data };
    esp_err_t ret = spi_device_transmit(s_stm32, &t);
    if (ret != ESP_OK) {
        printf("[stm_link] transmit failed: %s\n", esp_err_to_name(ret));
    }
}
