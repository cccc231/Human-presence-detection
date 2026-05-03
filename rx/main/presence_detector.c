#include "presence_detector.h"
#include "time_utils.h"
#include "esp_log.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include <math.h>
#include <stdio.h>
#include <string.h>

static const char *TAG = "presence";

typedef struct {
    csi_snapshot_t window[WINDOW_SIZE];
    int head;
    int count;
} ring_buffer_t;

typedef struct {
    ring_buffer_t rb;
    bool current_state;
    int stable_count;
    int warmup_counter;
} detector_state_t;

static detector_state_t det;

static float compute_variance_metric(detector_state_t *d)
{
    ring_buffer_t *rb = &d->rb;
    int n_sc = MAX_SUBCARRIERS;

    for (int i = 0; i < rb->count; i++) {
        if (rb->window[i].num_subcarriers < n_sc) {
            n_sc = rb->window[i].num_subcarriers;
        }
    }
    if (n_sc == 0) return 0.0f;

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

static void ring_buffer_add(ring_buffer_t *rb, csi_snapshot_t *snap)
{
    rb->window[rb->head] = *snap;
    rb->head = (rb->head + 1) % WINDOW_SIZE;
    if (rb->count < WINDOW_SIZE) rb->count++;
}

static presence_state_t detect_presence(csi_snapshot_t *snapshot)
{
    ring_buffer_add(&det.rb, snapshot);

    if (det.rb.count < WINDOW_SIZE) {
        det.warmup_counter++;
        if (det.warmup_counter % 20 == 0) {
            ESP_LOGI(TAG, "预热中... %d/%d", det.rb.count, WINDOW_SIZE);
        }
        return PRESENCE_UNKNOWN;
    }

    float metric = compute_variance_metric(&det);

    bool raw_decision;
    if (det.current_state) {
        raw_decision = (metric >= THRESHOLD_LOW);
    } else {
        raw_decision = (metric >= THRESHOLD_HIGH);
    }

    if (raw_decision == det.current_state) {
        det.stable_count = 0;
    } else {
        det.stable_count++;
        if (det.stable_count >= DEBOUNCE_COUNT) {
            det.current_state = raw_decision;
            det.stable_count = 0;
            ESP_LOGI(TAG, "状态切换: %s (metric=%.4f)",
                     det.current_state ? "有人" : "没人", metric);
        }
    }

    char time_buf[32];
    get_beijing_time_string(time_buf, sizeof(time_buf));
    const char *status_str = det.current_state ? "有人" : "没人";
    printf("%s, %s, %.4f\n", time_buf, status_str, metric);

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
    ESP_LOGI(TAG, "Presence detector initialized. Threshold: %.2f/%.2f, Window=%d",
             THRESHOLD_HIGH, THRESHOLD_LOW, WINDOW_SIZE);
    xTaskCreatePinnedToCore(presence_detector_task, "presence_det", 8192, NULL, 5, NULL, 1);
}
