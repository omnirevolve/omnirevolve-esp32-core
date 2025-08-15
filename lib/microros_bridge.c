#include "microros_bridge.h"
#include "plotter_config.h"

#include <string.h>
#include <inttypes.h>
#include "esp_log.h"

#include <rcl/rcl.h>
#include <rcl/error_handling.h>
#include <rclc/rclc.h>
#include <rclc/executor.h>
#include <std_msgs/msg/u_int8_multi_array.h>

#ifdef CONFIG_MICRO_ROS_ESP_XRCE_DDS_MIDDLEWARE
#include <rmw_microros/rmw_microros.h>
#endif

#include "freertos/FreeRTOS.h"
#include "freertos/task.h"

#include "consumer_stub.h"

// Макрос для проверки результатов
#define RCCHECK(fn) { rcl_ret_t temp_rc = fn; if((temp_rc != RCL_RET_OK)){ESP_LOGE(TAG, "Failed status on line %d: %d", __LINE__, (int)temp_rc); return;}}

static const char *TAG = "microros_bridge";

static rcl_allocator_t allocator;
static rclc_support_t support;
static rcl_node_t node;
static rcl_subscription_t sub;
static rclc_executor_t executor;

static rb_t *s_rb = NULL;
static microros_bridge_stats_t s_stats = {0};

static std_msgs__msg__UInt8MultiArray s_msg;

static void sub_cb(const void * msgin)
{
    ESP_LOGI(TAG, "Callback triggered!"); // Добавляем лог для диагностики
    
    const std_msgs__msg__UInt8MultiArray *m = (const std_msgs__msg__UInt8MultiArray*)msgin;
    const uint8_t *data = m->data.data;
    size_t len = m->data.size;
    
    ESP_LOGI(TAG, "Received %zu bytes", len); // Лог размера

#if SEQ_ID_ENABLED
    if (len < 1){
        s_stats.drops_total++;
        ESP_LOGW(TAG, "drop: empty msg with seq enabled");
        return;
    }
    uint8_t seq = data[0];
    if (s_stats.last_seq >= 0){
        uint8_t expect = ((uint8_t)s_stats.last_seq + 1) & 0xFF;
        if (seq != expect){
            s_stats.seq_missed++;
            ESP_LOGW(TAG, "seq miss: expected %u, got %u", expect, seq);
        }
    }
    s_stats.last_seq = seq;
    data += 1;
    len  -= 1;
#endif

    if (len == 0){
        s_stats.drops_total++;
        return;
    }

    size_t w = rb_write_all_or_drop(s_rb, data, len);
    if (w == 0){
        s_stats.drops_total++;
        ESP_LOGW(TAG, "Buffer full, dropped %zu bytes", len);
        return;
    }

    s_stats.pkts_in++;
    s_stats.bytes_in += (uint32_t)w;
    
    ESP_LOGI(TAG, "Buffered %zu bytes successfully", w);
}

static void log_task(void *arg)
{
    (void)arg;
    for(;;){
        size_t used = rb_used(s_rb);
        uint32_t consumed = consumer_stub_consumed_bytes();

        ESP_LOGI(TAG,
                 "in: pkts=%" PRIu32 " bytes=%" PRIu32 " drops=%" PRIu32
                 " seqmiss=%" PRIu32 " rb_used=%zu/%u consumed=%" PRIu32,
                 s_stats.pkts_in,
                 s_stats.bytes_in,
                 s_stats.drops_total,
                 s_stats.seq_missed,
                 used,
                 (unsigned)RB_SIZE_BYTES,
                 consumed);

        vTaskDelay(pdMS_TO_TICKS(LOG_PERIOD_MS));
    }
}

static void executor_task(void *arg)
{
    (void)arg;
    
    ESP_LOGI(TAG, "Executor task started");
    
    int spin_counter = 0;
    for(;;){
        rcl_ret_t rc = rclc_executor_spin_some(&executor, RCL_MS_TO_NS(10));
        
        // Периодический лог для проверки, что executor работает
        if (++spin_counter % 1000 == 0) { // Каждую секунду при частоте 1ms
            ESP_LOGD(TAG, "Executor spinning, counter=%d, rc=%d", spin_counter, (int)rc);
        }
        
        vTaskDelay(pdMS_TO_TICKS(1));
    }
}

static void microros_init_task(void *arg)
{
    rb_t *rb = (rb_t *)arg;
    
    ESP_LOGI(TAG, "Starting micro-ROS initialization");
    
    // Ждём стабилизации WiFi
    vTaskDelay(pdMS_TO_TICKS(2000));
    
    allocator = rcl_get_default_allocator();
    
    // Создаём init_options с UDP транспортом
    rcl_init_options_t init_options = rcl_get_zero_initialized_init_options();
    RCCHECK(rcl_init_options_init(&init_options, allocator));
    
#ifdef CONFIG_MICRO_ROS_ESP_XRCE_DDS_MIDDLEWARE
    rmw_init_options_t* rmw_options = rcl_init_options_get_rmw_init_options(&init_options);
    
    // Настраиваем UDP транспорт
    RCCHECK(rmw_uros_options_set_udp_address(
        CONFIG_MICRO_ROS_AGENT_IP, 
        CONFIG_MICRO_ROS_AGENT_PORT, 
        rmw_options));
    
    // ВАЖНО: Ждём подключения к агенту
    ESP_LOGI(TAG, "Waiting for agent at %s:%s...", 
             CONFIG_MICRO_ROS_AGENT_IP, CONFIG_MICRO_ROS_AGENT_PORT);
    
    while (rmw_uros_ping_agent(1000, 1) != RMW_RET_OK) {
        ESP_LOGW(TAG, "Agent not available, retrying...");
        vTaskDelay(pdMS_TO_TICKS(1000));
    }
    ESP_LOGI(TAG, "Agent connected!");
#endif
    
    // Инициализируем support с опциями
    RCCHECK(rclc_support_init_with_options(&support, 0, NULL, &init_options, &allocator));
    ESP_LOGI(TAG, "Support initialized");
    
    // Создаём ноду
    RCCHECK(rclc_node_init_default(&node, "esp32_plotter_node", "", &support));
    ESP_LOGI(TAG, "Node created");
    
    // Создаём подписчика с BEST_EFFORT QoS
    RCCHECK(rclc_subscription_init_best_effort(
        &sub, &node,
        ROSIDL_GET_MSG_TYPE_SUPPORT(std_msgs, msg, UInt8MultiArray),
        "/plotter/byte_stream"
    ));
    ESP_LOGI(TAG, "Subscription created for /plotter/byte_stream");
    
    // Инициализируем сообщение
    std_msgs__msg__UInt8MultiArray__init(&s_msg);
    // ВАЖНО: Выделяем память для данных
    s_msg.data.capacity = 256;
    s_msg.data.size = 0;
    s_msg.data.data = (uint8_t*) malloc(s_msg.data.capacity * sizeof(uint8_t));
    
    // Создаём executor
    RCCHECK(rclc_executor_init(&executor, &support.context, 1, &allocator));
    
    // Добавляем подписчика в executor
    RCCHECK(rclc_executor_add_subscription(&executor, &sub, &s_msg, &sub_cb, ON_NEW_DATA));
    ESP_LOGI(TAG, "Executor configured");
    
    // Запускаем executor в отдельной задаче
    xTaskCreatePinnedToCore(executor_task, "mr_executor", 8192, NULL, 6, NULL, tskNO_AFFINITY);
    
    // Запускаем задачу логирования
    xTaskCreatePinnedToCore(log_task, "mr_log", 4096, NULL, 4, NULL, tskNO_AFFINITY);
    
    ESP_LOGI(TAG, "micro-ROS fully initialized");
    
    // Эта задача завершается
    vTaskDelete(NULL);
}

void microros_bridge_init(rb_t *rb)
{
    s_rb = rb;
    s_stats.last_seq = -1;
    
    // Запускаем инициализацию в отдельной задаче
    xTaskCreatePinnedToCore(microros_init_task, "mr_init", 16384, rb, 5, NULL, tskNO_AFFINITY);
    
    ESP_LOGI(TAG, "Bridge initialization started");
}

const microros_bridge_stats_t* microros_bridge_stats(void)
{
    return &s_stats;
}