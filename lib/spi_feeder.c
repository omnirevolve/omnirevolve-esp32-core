#include <string.h>
#include "spi_feeder.h"
#include "esp32_to_stm32.h"
#include "ring_buffer.h"
#include "plotter_config.h"
#include "esp_log.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"

#define SPI_CHUNK_SIZE   2048u   // размер блока для STM32 DMA
#define FEEDER_PERIOD_MS 10      // период опроса

static const char* TAG = "spi_feeder";

static rb_t*    s_rb            = NULL;
static uint32_t s_total_sent    = 0;
static uint32_t s_chunks_sent   = 0;

static volatile bool s_finishing = false;   // получили запрос завершения (finish)
static volatile bool s_flushed   = false;   // остаток отправлен, можно считать фидер idle

// избежать 2К локального массива на стеке задачи
static uint8_t s_chunk_buf[SPI_CHUNK_SIZE];

static void spi_feeder_task(void* arg) {
    rb_t* rb = (rb_t*)arg;

    ESP_LOGI(TAG, "SPI feeder task started (chunk=%u)", (unsigned)SPI_CHUNK_SIZE);

    for (;;) {
        // Если уже всё дожали — просто «дремлем»
        if (s_finishing && s_flushed) {
            vTaskDelay(pdMS_TO_TICKS(FEEDER_PERIOD_MS));
            continue;
        }

        // Проверяем готовность STM32
        if (!plotter_is_ready_to_receive_draw_stream_data()) {
            static uint32_t not_ready_ticks = 0;
            if ((++not_ready_ticks % (1000/FEEDER_PERIOD_MS)) == 0) { // ~раз в секунду
                ESP_LOGW(TAG, "STM32 not ready, rb_used=%u", (unsigned)rb_used(rb));
            }
            vTaskDelay(pdMS_TO_TICKS(FEEDER_PERIOD_MS));
            continue;
        }

        size_t available = rb_used(rb);

        // Обычный режим: шлём только полные 2048-байтные чанки
        if (!s_finishing) {
            if (available >= SPI_CHUNK_SIZE) {
                size_t read = rb_read_exact(rb, s_chunk_buf, SPI_CHUNK_SIZE);
                if (read == SPI_CHUNK_SIZE) {
                    plotter_send_draw_stream_data(s_chunk_buf, SPI_CHUNK_SIZE);
                    s_chunks_sent++;
                    s_total_sent += SPI_CHUNK_SIZE;
                    ESP_LOGD(TAG, "sent chunk #%u, total=%u",
                             (unsigned)s_chunks_sent, (unsigned)s_total_sent);
                }
            } else {
                // ждём накопления
                vTaskDelay(pdMS_TO_TICKS(FEEDER_PERIOD_MS));
            }
            continue;
        }

        // Режим завершения: дожать всё, что осталось.
        if (available >= SPI_CHUNK_SIZE) {
            // Ещё есть полные чанки — отправляем их как обычно
            size_t read = rb_read_exact(rb, s_chunk_buf, SPI_CHUNK_SIZE);
            if (read == SPI_CHUNK_SIZE) {
                plotter_send_draw_stream_data(s_chunk_buf, SPI_CHUNK_SIZE);
                s_chunks_sent++;
                s_total_sent += SPI_CHUNK_SIZE;
                ESP_LOGD(TAG, "finish mode: sent full chunk, left=%u",
                         (unsigned)rb_used(rb));
            }
            continue;
        }

        // Осталось < 2048 байт — финальный пэддинг-блок
        if (available > 0) {
            size_t remain = available;
            size_t read = rb_read_exact(rb, s_chunk_buf, remain);
            if (read == remain) {
                memset(s_chunk_buf + remain, 0, SPI_CHUNK_SIZE - remain);
                plotter_send_draw_stream_data(s_chunk_buf, SPI_CHUNK_SIZE);
                s_chunks_sent++;
                s_total_sent += SPI_CHUNK_SIZE;
                ESP_LOGI(TAG, "flushed tail: %u+pad -> %u bytes, total=%u",
                         (unsigned)remain, (unsigned)SPI_CHUNK_SIZE, (unsigned)s_total_sent);
            } else {
                vTaskDelay(pdMS_TO_TICKS(FEEDER_PERIOD_MS));
                continue;
            }
        }

        // Если сюда дошли, буфер пуст — считаем, что дожали
        if (rb_used(rb) == 0) {
            s_flushed = true;
            ESP_LOGI(TAG, "flush complete");
        }

        vTaskDelay(pdMS_TO_TICKS(FEEDER_PERIOD_MS));
    }
}

void spi_feeder_start(rb_t* rb) {
    if (rb == NULL) {
        ESP_LOGE(TAG, "Cannot start feeder with NULL ring buffer");
        return;
    }
    s_rb = rb;
    s_total_sent  = 0;
    s_chunks_sent = 0;
    s_finishing   = false;
    s_flushed     = false;

    xTaskCreatePinnedToCore(
        spi_feeder_task,
        "spi_feeder",
        6144,     // было 4096: подняли, т.к. много логов + varargs
        rb,
        6,
        NULL,
        1
    );

    ESP_LOGI(TAG, "SPI feeder started");
}

void spi_feeder_request_finish(void) {
    s_finishing = true;
    ESP_LOGI(TAG, "finish requested");
}

bool spi_feeder_is_flushed(void) {
    return s_flushed;
}

uint32_t spi_feeder_get_total_sent(void) {
    return s_total_sent;
}

uint32_t spi_feeder_get_chunks_sent(void) {
    return s_chunks_sent;
}
