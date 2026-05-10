#include <string.h>
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "esp_system.h"
#include "esp_wifi.h"
#include "esp_event.h"
#include "esp_log.h"
#include "nvs_flash.h"
#include "esp_netif.h"
#include "esp_now.h"
#include "esp_mac.h"

#include "csi_processor.h"
#include "bed_detector.h"

static const char *TAG = "csi_rx";

#define CONFIG_CSI_SEND_MAC  {0xff, 0xff, 0xff, 0xff, 0xff, 0xff}
#define CONFIG_ESPNOW_CHANNEL 0

static uint8_t s_csi_send_mac[6] = CONFIG_CSI_SEND_MAC;

static void esp_now_recv_cb(const esp_now_recv_info_t *info, const uint8_t *data, int len)
{
    // Just receiving triggers CSI capture - no processing needed here
}

static void wifi_init(void)
{
    ESP_ERROR_CHECK(esp_event_loop_create_default());
    ESP_ERROR_CHECK(esp_netif_init());

    wifi_init_config_t cfg = WIFI_INIT_CONFIG_DEFAULT();
    cfg.csi_enable = 1;
    ESP_ERROR_CHECK(esp_wifi_init(&cfg));

    ESP_ERROR_CHECK(esp_wifi_set_mode(WIFI_MODE_STA));
    ESP_ERROR_CHECK(esp_wifi_set_storage(WIFI_STORAGE_RAM));
    ESP_ERROR_CHECK(esp_wifi_start());

    ESP_ERROR_CHECK(esp_wifi_set_band_mode(WIFI_BAND_MODE_2G_ONLY));

    wifi_protocols_t protocols = {
        .ghz_2g = WIFI_PROTOCOL_11N,
        .ghz_5g = 0,
    };
    ESP_ERROR_CHECK(esp_wifi_set_protocols(ESP_IF_WIFI_STA, &protocols));

    wifi_bandwidths_t bandwidth = {
        .ghz_2g = WIFI_BW_HT20,
        .ghz_5g = 0,
    };
    ESP_ERROR_CHECK(esp_wifi_set_bandwidths(ESP_IF_WIFI_STA, &bandwidth));
    ESP_ERROR_CHECK(esp_wifi_set_ps(WIFI_PS_NONE));
}

static void csi_init(void)
{
    wifi_csi_config_t csi_config = {
        .lltf_en = true,
        .htltf_en = true,
        .stbc_htltf2_en = false,
        .ltf_merge_en = true,
        .channel_filter_en = true,
        .manu_scale = false,
        .shift = false,
    };

    ESP_ERROR_CHECK(esp_wifi_set_csi_rx_cb(csi_rx_cb, NULL));
    ESP_ERROR_CHECK(esp_wifi_set_csi_config(&csi_config));
    ESP_ERROR_CHECK(esp_wifi_set_csi(true));
    ESP_ERROR_CHECK(esp_wifi_set_promiscuous(true));

    ESP_LOGI(TAG, "CSI initialized");
}

static void esp_now_init_rx(void)
{
    ESP_ERROR_CHECK(esp_now_init());
    ESP_ERROR_CHECK(esp_now_register_recv_cb(esp_now_recv_cb));

    esp_now_peer_info_t peer = {
        .channel = CONFIG_ESPNOW_CHANNEL,
        .ifidx = WIFI_IF_STA,
        .encrypt = false,
    };
    memcpy(peer.peer_addr, s_csi_send_mac, 6);
    ESP_ERROR_CHECK(esp_now_add_peer(&peer));

    ESP_LOGI(TAG, "ESP-NOW initialized, listening for: " MACSTR, MAC2STR(s_csi_send_mac));
}

void app_main(void)
{
    esp_err_t ret = nvs_flash_init();
    if (ret == ESP_ERR_NVS_NO_FREE_PAGES || ret == ESP_ERR_NVS_NEW_VERSION_FOUND) {
        ESP_ERROR_CHECK(nvs_flash_erase());
        ret = nvs_flash_init();
    }
    ESP_ERROR_CHECK(ret);

    csi_processor_init();
    bed_detector_init();
    wifi_init();
    csi_init();
    esp_now_init_rx();

    ESP_LOGI(TAG, "================ CSI RX - Vital Signs Mode ================");
    ESP_LOGI(TAG, "TX rate: 120 pkt/s, output: raw I/Q via serial");
    ESP_LOGI(TAG, "Format: CSI,<timestamp_us>,<rssi>,<num_sub>,<I0>,<Q0>,...");

    while (1) {
        vTaskDelay(pdMS_TO_TICKS(10000));
    }
}
