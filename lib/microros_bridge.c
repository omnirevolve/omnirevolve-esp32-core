#include "microros_bridge.h"
#include "plotter_config.h"

#include <string.h>
#include <inttypes.h>                 // PRIu32
#include "esp_log.h"

#include <rcl/rcl.h>
#include <rcl/error_handling.h>
#include <rclc/rclc.h>
#include <rclc/executor.h>
#include <std_msgs/msg/u_int8_multi_array.h>

#include "freertos/FreeRTOS.h"        // vTaskDelay, pdMS_TO_TICKS
#include "freertos/task.h"            // xTaskCreatePinnedToCore, tskNO_AFFINITY

#include "consumer_stub.h"            // consumer_stub_consumed_bytes()

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
    const std_msgs__msg__UInt8MultiArray *m = (const std_msgs__msg__UInt8MultiArray*)msgin;
    const uint8_t *data = m->data.data;
    size_t len = m->data.size;

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
        return;
    }

    s_stats.pkts_in++;
    s_stats.bytes_in += (uint32_t)w;
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
    for(;;){
        rclc_executor_spin_some(&executor, RCL_MS_TO_NS(5));
        vTaskDelay(pdMS_TO_TICKS(1));
    }
}

void microros_bridge_init(rb_t *rb)
{
    s_rb = rb;
    s_stats.last_seq = -1;

    allocator = rcl_get_default_allocator();

    rclc_support_init(&support, 0, NULL, &allocator);

    rclc_node_init_default(&node, "esp32_plotter_node", "", &support);

    rclc_subscription_init_best_effort(
        &sub, &node,
        ROSIDL_GET_MSG_TYPE_SUPPORT(std_msgs, msg, UInt8MultiArray),
        "/plotter/byte_stream"
    );

    std_msgs__msg__UInt8MultiArray__init(&s_msg);

    rclc_executor_init(&executor, &support.context, 1, &allocator);
    rclc_executor_add_subscription(&executor, &sub, &s_msg, &sub_cb, ON_NEW_DATA);

    xTaskCreatePinnedToCore(executor_task, "mr_executor", 4096, NULL, 6, NULL, tskNO_AFFINITY);
    xTaskCreatePinnedToCore(log_task,      "mr_log",      4096, NULL, 4, NULL, tskNO_AFFINITY);

    ESP_LOGI(TAG, "initialized (BestEffort, KeepLast(1), seq_id=%d)", (int)SEQ_ID_ENABLED);
}

const microros_bridge_stats_t* microros_bridge_stats(void)
{
    return &s_stats;
}
