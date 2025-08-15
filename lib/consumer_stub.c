#include <string.h>
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "esp_log.h"
#include "ring_buffer.h"
#include "plotter_config.h"

static const char *TAG = "consumer_stub";

static rb_t *s_rb = NULL;
static TaskHandle_t s_task = NULL;
static uint32_t s_consumed_bytes = 0;

static void consumer_task(void *arg){
    uint8_t local[SPI_PORTION_SIZE];

    for(;;){
        if (!s_rb){
            vTaskDelay(pdMS_TO_TICKS(10));
            continue;
        }
        size_t used = rb_used(s_rb);
        if (used >= SPI_PORTION_SIZE){
            size_t got = rb_read_exact(s_rb, local, SPI_PORTION_SIZE);
            if (got == SPI_PORTION_SIZE){
                s_consumed_bytes += got;
                // MVP: никуда не отправляем — имитация выгрузки
            }
        } else {
            vTaskDelay(pdMS_TO_TICKS(2));
        }
    }
}

void consumer_stub_start(rb_t *rb){
    s_rb = rb;
    if (!s_task){
        xTaskCreatePinnedToCore(consumer_task, "consumer", 4096, NULL, 5, &s_task, tskNO_AFFINITY);
        ESP_LOGI(TAG, "started, portion=%d bytes", (int)SPI_PORTION_SIZE);
    }
}

uint32_t consumer_stub_consumed_bytes(void){
    return s_consumed_bytes;
}
