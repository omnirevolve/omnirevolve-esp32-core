#include "spi_feeder.h"
#include "plotter_hal.h"
#include "ring_buffer.h"
#include "plotter_config.h"
#include "esp_log.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"

#define SPI_CHUNK_SIZE 2048  // Размер блока для STM32 DMA

static const char* TAG = "spi_feeder";

static rb_t* s_rb = NULL;
static uint32_t s_total_sent = 0;
static uint32_t s_chunks_sent = 0;

static void spi_feeder_task(void* arg) {
    rb_t* rb = (rb_t*)arg;
    uint8_t chunk[SPI_CHUNK_SIZE];
    
    ESP_LOGI(TAG, "SPI feeder task started");
    
    // Ждём инициализации HAL
    while (g_plotter_hal == NULL) {
        vTaskDelay(pdMS_TO_TICKS(100));
    }
    
    ESP_LOGI(TAG, "HAL available, starting feeder loop");
    
    while (1) {
        // Проверяем готовность STM32
        if (g_plotter_hal->is_ready()) {
            size_t available = rb_used(rb);
            
            // Отправляем только полные чанки по 2048 байт
            if (available >= SPI_CHUNK_SIZE) {
                size_t read = rb_read(rb, chunk, SPI_CHUNK_SIZE);
                
                if (read == SPI_CHUNK_SIZE) {
                    ESP_LOGI(TAG, "Sending chunk %lu (2048 bytes) to STM32", s_chunks_sent);
                    
                    // Отправляем через HAL
                    g_plotter_hal->send_nibbles(chunk, SPI_CHUNK_SIZE);
                    
                    s_chunks_sent++;
                    s_total_sent += SPI_CHUNK_SIZE;
                    
                    ESP_LOGD(TAG, "Total sent: %lu bytes in %lu chunks", 
                            s_total_sent, s_chunks_sent);
                }
            } else if (available > 0 && available < 100) {
                // Если осталось мало данных и сессия завершается
                // TODO: проверить флаг finish_requested из microros_bridge
                ESP_LOGD(TAG, "Buffer has %zu bytes (waiting for more)", available);
            }
        } else {
            // STM32 не готов
            static int not_ready_count = 0;
            if (++not_ready_count % 100 == 0) {  // Логируем раз в секунду
                ESP_LOGW(TAG, "STM32 not ready, buffer: %zu bytes", rb_used(rb));
            }
        }
        
        vTaskDelay(pdMS_TO_TICKS(10));  // 100Hz опрос
    }
}

void spi_feeder_start(rb_t* rb) {
    if (rb == NULL) {
        ESP_LOGE(TAG, "Cannot start feeder with NULL ring buffer");
        return;
    }
    
    s_rb = rb;
    s_total_sent = 0;
    s_chunks_sent = 0;
    
    xTaskCreatePinnedToCore(
        spi_feeder_task, 
        "spi_feeder", 
        4096, 
        rb, 
        6,  // Высокий приоритет для real-time передачи
        NULL, 
        tskNO_AFFINITY
    );
    
    ESP_LOGI(TAG, "SPI feeder started with %d byte chunks", SPI_CHUNK_SIZE);
}

uint32_t spi_feeder_get_total_sent(void) {
    return s_total_sent;
}

uint32_t spi_feeder_get_chunks_sent(void) {
    return s_chunks_sent;
}