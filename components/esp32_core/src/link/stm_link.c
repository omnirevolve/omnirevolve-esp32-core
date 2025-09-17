#include <stdio.h>
#include "driver/spi_master.h"
#include "driver/gpio.h"
#include "freertos/FreeRTOS.h"
#include "omnirevolve/hal/pins.h"
#include "omnirevolve/protocol/cmd_ids.h"

static spi_device_handle_t s_stm_spi = NULL;

void or_stm_spi_init(void){
    printf("\n=== STM32 SPI INITIALIZATION ===\n");
    spi_bus_config_t buscfg = {
        .mosi_io_num = STM32_SPI_MOSI,
        .miso_io_num = -1,
        .sclk_io_num = STM32_SPI_CLK,
        .quadwp_io_num = -1,
        .quadhd_io_num = -1,
        .max_transfer_sz = SPI_CHUNK_SIZE,
        .flags = SPICOMMON_BUSFLAG_MASTER
    };
    esp_err_t ret = spi_bus_initialize(STM32_SPI_HOST, &buscfg, 1);
    if (ret != ESP_OK){ printf("ERROR: SPI bus init failed: %s\n", esp_err_to_name(ret)); return; }
    spi_device_interface_config_t devcfg = {
        .mode = 0,
        .clock_speed_hz = 8*1000*1000,
        .spics_io_num = -1,
        .queue_size = 8
    };
    ret = spi_bus_add_device(STM32_SPI_HOST, &devcfg, &s_stm_spi);
    if (ret != ESP_OK){ printf("ERROR: add SPI dev: %s\n", esp_err_to_name(ret)); return; }
    printf("STM32 SPI ready (HSPI)\n");
}

void or_stm_config_ready_irq(ready_signal_isr_callback isr_cb){
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
    ESP_ERROR_CHECK(gpio_isr_handler_add(STM32_READY_PIN, isr_cb, NULL));
    ESP_ERROR_CHECK(gpio_intr_enable(STM32_READY_PIN));
}

void or_stm_send_stream(const uint8_t* data, uint32_t len){
    if (!s_stm_spi || !data || !len){ printf("SPI stream: invalid args\n"); return; }
    spi_transaction_t t = { .length = len*8, .tx_buffer = data };
    esp_err_t ret = spi_device_transmit(s_stm_spi, &t);
    if (ret != ESP_OK) printf("SPI transmit failed: %s\n", esp_err_to_name(ret));
}
