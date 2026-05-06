#pragma once

#include <stdint.h>
#include "esp_wifi_types.h"
#include "freertos/FreeRTOS.h"
#include "freertos/queue.h"

#define MAX_CSI_LEN       256
#define MAX_SUBCARRIERS   64
#define CSI_QUEUE_SIZE    40

typedef struct {
    int8_t buf[MAX_CSI_LEN];
    uint16_t len;
    int8_t rssi;
    int64_t timestamp;
    bool first_word_invalid;
} csi_raw_t;

extern QueueHandle_t csi_raw_queue;

void csi_processor_init(void);
void csi_rx_cb(void *ctx, wifi_csi_info_t *info);
