#include "presence_detector.h"
#include "time_utils.h"
#include "esp_log.h"
#include "esp_timer.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include <math.h>
#include <stdio.h>
#include <string.h>

static const char *TAG = "presence";

#define AGG_BUF_SIZE 100

typedef struct {
    csi_snapshot_t window[WINDOW_SIZE];
    int head;
    int count;
} ring_buffer_t;

typedef struct {
    ring_buffer_t rb;

    /* 初始化阶段 */
    float baseline_sum[MAX_SUBCARRIERS];
    float baseline[MAX_SUBCARRIERS];
    int num_subcarriers;
    int init_count;
    bool initialized;

    /* 1秒聚合缓冲 */
    bool raw_decisions[AGG_BUF_SIZE];
    float metrics[AGG_BUF_SIZE];
    int agg_count;
    int64_t agg_start_us;

    /* 最终输出状态 */
    bool current_state;
    int warmup_counter;
} detector_state_t;

static detector_state_t det;

static float compute_variance_metric(detector_state_t *d)
{
    ring_buffer_t *rb = &d->rb;
    int n_sc = d->num_subcarriers;
    if (n_sc == 0 || rb->count == 0) return 0.0f;

    float total_var = 0.0f;
    for (int k = 0; k < n_sc; k++) {
        float sum = 0.0f, sum_sq = 0.0f;
        for (int i = 0; i < rb->count; i++) {
            float v = rb->window[i].amplitudes[k];
            sum += v;
            sum_sq += v * v;
        }
        float mean = sum / rb->count;
        float var = (sum_sq / rb->count) - (mean * mean);
        total_var += var;
    }
    return total_var / n_sc;
}

static float compute_amp_dev_metric(detector_state_t *d)
{
    ring_buffer_t *rb = &d->rb;
    int n_sc = d->num_subcarriers;
    if (n_sc == 0 || rb->count == 0) return 0.0f;

    float total_dev = 0.0f;
    for (int k = 0; k < n_sc; k++) {
        float sum = 0.0f;
        for (int i = 0; i < rb->count; i++) {
            sum += rb->window[i].amplitudes[k];
        }
        float mean_k = sum / rb->count;
        total_dev += fabsf(mean_k - d->baseline[k]);
    }
    return total_dev / n_sc;
}

static void ring_buffer_add(ring_buffer_t *rb, csi_snapshot_t *snap)
{
    rb->window[rb->head] = *snap;
    rb->head = (rb->head + 1) % WINDOW_SIZE;
    if (rb->count < WINDOW_SIZE) rb->count++;
}

static void update_baseline(detector_state_t *d, csi_snapshot_t *snap)
{
    int n_sc = d->num_subcarriers;
    for (int k = 0; k < n_sc; k++) {
        d->baseline[k] = d->baseline[k] * 0.999f + snap->amplitudes[k] * 0.001f;
    }
}

static presence_state_t detect_presence(csi_snapshot_t *snapshot)
{
    ring_buffer_add(&det.rb, snapshot);

    /* Phase 1: 初始化 - 采集空房间基准 */
    if (!det.initialized) {
        for (int k = 0; k < snapshot->num_subcarriers; k++) {
            det.baseline_sum[k] += snapshot->amplitudes[k];
        }
        det.num_subcarriers = snapshot->num_subcarriers;
        det.init_count++;

        if (det.init_count < INIT_SAMPLES) {
            if (det.init_count % 50 == 0) {
                ESP_LOGI(TAG, "初始化中... %d/%d", det.init_count, INIT_SAMPLES);
            }
            return PRESENCE_UNKNOWN;
        }

        for (int k = 0; k < det.num_subcarriers; k++) {
            det.baseline[k] = det.baseline_sum[k] / det.init_count;
        }
        det.initialized = true;
        det.agg_start_us = esp_timer_get_time();
        ESP_LOGI(TAG, "===== 初始化完成! %d个子载波基准已建立 =====", det.num_subcarriers);
        return PRESENCE_UNKNOWN;
    }

    /* Phase 2: 预热 - 填满滑动窗口 */
    if (det.rb.count < WINDOW_SIZE) {
        det.warmup_counter++;
        if (det.warmup_counter % 20 == 0) {
            ESP_LOGI(TAG, "预热中... %d/%d", det.rb.count, WINDOW_SIZE);
        }
        return PRESENCE_UNKNOWN;
    }

    /* Phase 3: 逐包计算原始判定 */
    float var_metric = compute_variance_metric(&det);
    float amp_metric = compute_amp_dev_metric(&det);
    float metric = (var_metric > amp_metric) ? var_metric : amp_metric;

    bool raw_decision = (metric >= THRESHOLD_HIGH);

    /* 存入1秒聚合缓冲 */
    if (det.agg_count < AGG_BUF_SIZE) {
        det.raw_decisions[det.agg_count] = raw_decision;
        det.metrics[det.agg_count] = metric;
        det.agg_count++;
    }

    /* 检查1秒窗口是否到期 */
    int64_t now_us = esp_timer_get_time();
    if ((now_us - det.agg_start_us) < (int64_t)AGG_WINDOW_MS * 1000) {
        return det.current_state ? PRESENCE_DETECTED : PRESENCE_EMPTY;
    }

    /* === 1秒聚合：统计有人比例 === */
    int present_count = 0;
    float metric_sum = 0.0f;
    for (int i = 0; i < det.agg_count; i++) {
        if (det.raw_decisions[i]) present_count++;
        metric_sum += det.metrics[i];
    }
    float ratio = (det.agg_count > 0) ? (float)present_count / det.agg_count : 0.0f;
    float avg_metric = (det.agg_count > 0) ? metric_sum / det.agg_count : 0.0f;

    det.current_state = (ratio >= PRESENCE_RATIO);

    /* "没人"时缓慢更新baseline */
    if (!det.current_state) {
        update_baseline(&det, snapshot);
    }

    char time_buf[32];
    get_beijing_time_string(time_buf, sizeof(time_buf));
    const char *status_str = det.current_state ? "有人" : "没人";
    printf("%s, %s, %.4f, %d/%d\n", time_buf, status_str, avg_metric,
           present_count, det.agg_count);

    /* 重置聚合窗口 */
    det.agg_count = 0;
    det.agg_start_us = now_us;

    return det.current_state ? PRESENCE_DETECTED : PRESENCE_EMPTY;
}

static void presence_detector_task(void *arg)
{
    csi_snapshot_t snapshot;

    while (1) {
        if (xQueueReceive(csi_snapshot_queue, &snapshot, portMAX_DELAY) != pdTRUE) {
            continue;
        }
        detect_presence(&snapshot);
    }
}

void presence_detector_init(void)
{
    memset(&det, 0, sizeof(det));
    det.current_state = false;
    det.num_subcarriers = MAX_SUBCARRIERS;
    ESP_LOGI(TAG, "Presence detector initialized. Threshold=%.2f, Ratio=%.0f%%, Window=%d, Init=%d",
             THRESHOLD_HIGH, PRESENCE_RATIO * 100, WINDOW_SIZE, INIT_SAMPLES);
    xTaskCreatePinnedToCore(presence_detector_task, "presence_det", 8192, NULL, 5, NULL, 1);
}
