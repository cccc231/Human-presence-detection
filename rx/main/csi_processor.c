#include "csi_processor.h"
#include "esp_log.h"
#include "esp_timer.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include <stdio.h>
#include <string.h>

static const char *TAG = "csi_proc";

QueueHandle_t csi_raw_queue = NULL;

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
    int pkt_count = 0;
    /* 预分配行缓冲，一次写入避免 printf 被看门狗中断截断 */
    /* CSI,<20位ts>,<4位rssi>,<3位nsub> + 64对IQ各6字节 + 换行 + 尾零 */
    char line_buf[MAX_CSI_LEN * 6 + 64];

    while (1) {
        if (xQueueReceive(csi_raw_queue, &raw, portMAX_DELAY) != pdTRUE) {
            continue;
        }

        pkt_count++;

        uint16_t offset = 0;
        if (raw.first_word_invalid) {
            offset = 4;
        }

        if (raw.len <= offset) {
            continue;
        }

        uint16_t data_len = raw.len - offset;
        uint16_t num_subcarriers = data_len / 2;
        if (num_subcarriers > MAX_SUBCARRIERS) {
            num_subcarriers = MAX_SUBCARRIERS;
        }

        /* 每 4 包输出 1 包: 120/4 = 30Hz, 30*200=6KB/s < 115200/10=11.5KB/s */
        if (pkt_count % 4 == 0) {
            int pos = snprintf(line_buf, sizeof(line_buf),
                               "CSI,%lld,%d,%d",
                               (long long)raw.timestamp,
                               (int)raw.rssi,
                               (int)num_subcarriers);
            for (uint16_t i = 0; i < num_subcarriers; i++) {
                int8_t imag = raw.buf[offset + 2 * i];
                int8_t real = raw.buf[offset + 2 * i + 1];
                pos += snprintf(line_buf + pos, sizeof(line_buf) - pos,
                                ",%d,%d", (int)real, (int)imag);
            }
            line_buf[pos++] = '\n';
            line_buf[pos] = '\0';
            /* 单次 write，不会被看门狗截断 */
            fputs(line_buf, stdout);
        }

        /* 每包让出 CPU */
        taskYIELD();
    }
}

void csi_processor_init(void)
{
    csi_raw_queue = xQueueCreate(CSI_QUEUE_SIZE, sizeof(csi_raw_t));
    configASSERT(csi_raw_queue);

    xTaskCreatePinnedToCore(csi_processor_task, "csi_proc", 8192, NULL, 5, &csi_task_handle, 0);

    ESP_LOGI(TAG, "CSI processor initialized (raw I/Q output mode)");
}
