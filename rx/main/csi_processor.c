#include "csi_processor.h"
#include "esp_log.h"
#include "esp_timer.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include <math.h>
#include <string.h>

static const char *TAG = "csi_proc";

QueueHandle_t csi_raw_queue = NULL;
QueueHandle_t csi_snapshot_queue = NULL;

static TaskHandle_t csi_task_handle = NULL;

void csi_rx_cb(void *ctx, wifi_csi_info_t *info)
{
    if (!info || !info->buf || info->len == 0) {
        return;
    }

    csi_raw_t raw;
    memset(&raw, 0, sizeof(raw));

    uint16_t copy_len = info->len;
    if (copy_len > MAX_CSI_LEN) {
        copy_len = MAX_CSI_LEN;
    }

    memcpy(raw.buf, info->buf, copy_len);
    raw.len = copy_len;
    raw.rssi = info->rx_ctrl.rssi;
    raw.timestamp = esp_timer_get_time();
    raw.first_word_invalid = info->first_word_invalid;

    if (xQueueSend(csi_raw_queue, &raw, 0) != pdTRUE) {
        // Queue full, drop packet
    }
}

static void csi_processor_task(void *arg)
{
    csi_raw_t raw;
    csi_snapshot_t snapshot;

    while (1) {
        if (xQueueReceive(csi_raw_queue, &raw, portMAX_DELAY) != pdTRUE) {
            continue;
        }

        uint16_t offset = 0;
        if (raw.first_word_invalid) {
            offset = 4; // Skip first 4 bytes (2 I/Q pairs)
        }

        if (raw.len <= offset) {
            continue;
        }

        uint16_t data_len = raw.len - offset;
        uint16_t num_subcarriers = data_len / 2;
        if (num_subcarriers > MAX_SUBCARRIERS) {
            num_subcarriers = MAX_SUBCARRIERS;
        }

        memset(&snapshot, 0, sizeof(snapshot));
        snapshot.num_subcarriers = num_subcarriers;
        snapshot.timestamp = raw.timestamp;

        float sum = 0.0f;
        for (uint16_t i = 0; i < num_subcarriers; i++) {
            int8_t imag = raw.buf[offset + 2 * i];
            int8_t real = raw.buf[offset + 2 * i + 1];
            snapshot.amplitudes[i] = sqrtf((float)(real * real + imag * imag));
            sum += snapshot.amplitudes[i];
        }

        snapshot.mean_amp = sum / num_subcarriers;

        if (xQueueSend(csi_snapshot_queue, &snapshot, 0) != pdTRUE) {
            // Queue full, drop
        }
    }
}

void csi_processor_init(void)
{
    csi_raw_queue = xQueueCreate(CSI_QUEUE_SIZE, sizeof(csi_raw_t));
    csi_snapshot_queue = xQueueCreate(DETECTOR_QUEUE_SIZE, sizeof(csi_snapshot_t));

    configASSERT(csi_raw_queue);
    configASSERT(csi_snapshot_queue);

    xTaskCreatePinnedToCore(csi_processor_task, "csi_proc", 4096, NULL, 5, &csi_task_handle, 0);

    ESP_LOGI(TAG, "CSI processor initialized");
}
