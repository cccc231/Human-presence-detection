#include <stdio.h>
#include <string.h>
#include <stdlib.h>
#include <stdint.h>
#include <unistd.h>

#include "nvs_flash.h"
#include "esp_mac.h"
#include "esp_log.h"
#include "esp_wifi.h"
#include "esp_netif.h"
#include "esp_now.h"
#include "esp_event.h"
#include "esp_timer.h"

#define CONFIG_ESPNOW_CHANNEL   0
#define CONFIG_WIFI_BAND_MODE   WIFI_BAND_MODE_2G_ONLY
#define CONFIG_ESPNOW_SECOND_CHAN WIFI_SECOND_CHAN_NONE
#define CONFIG_WIFI_2G_BANDWIDTHS WIFI_BW_HT20
#define CONFIG_WIFI_2G_PROTOCOL WIFI_PROTOCOL_11N
#define CONFIG_ESP_NOW_PHYMODE  WIFI_PHY_MODE_HT20
#define CONFIG_ESP_NOW_RATE     WIFI_PHY_RATE_MCS0_LGI
#define CONFIG_SEND_FREQUENCY   120

static const char *TAG = "csi_tx";
static uint8_t g_local_mac[6] = {0};

typedef struct __attribute__((packed)) {
    uint32_t seq;
    int64_t tx_us;
} csi_ping_t;

static void wifi_init(void)
{
    ESP_ERROR_CHECK(esp_event_loop_create_default());
    ESP_ERROR_CHECK(esp_netif_init());

    wifi_init_config_t cfg = WIFI_INIT_CONFIG_DEFAULT();
    ESP_ERROR_CHECK(esp_wifi_init(&cfg));

    ESP_ERROR_CHECK(esp_wifi_set_mode(WIFI_MODE_STA));
    ESP_ERROR_CHECK(esp_wifi_set_storage(WIFI_STORAGE_RAM));
    ESP_ERROR_CHECK(esp_wifi_start());

    ESP_ERROR_CHECK(esp_wifi_set_band_mode(CONFIG_WIFI_BAND_MODE));

    wifi_protocols_t protocols = {
        .ghz_2g = CONFIG_WIFI_2G_PROTOCOL,
        .ghz_5g = 0,
    };
    ESP_ERROR_CHECK(esp_wifi_set_protocols(ESP_IF_WIFI_STA, &protocols));

    wifi_bandwidths_t bandwidth = {
        .ghz_2g = CONFIG_WIFI_2G_BANDWIDTHS,
        .ghz_5g = 0,
    };
    ESP_ERROR_CHECK(esp_wifi_set_bandwidths(ESP_IF_WIFI_STA, &bandwidth));
    ESP_ERROR_CHECK(esp_wifi_set_ps(WIFI_PS_NONE));

    ESP_ERROR_CHECK(esp_wifi_get_mac(WIFI_IF_STA, g_local_mac));
}

static void wifi_esp_now_init(const esp_now_peer_info_t *peer)
{
    ESP_ERROR_CHECK(esp_now_init());
    ESP_ERROR_CHECK(esp_now_set_pmk((uint8_t *)"pmk1234567890123"));

    esp_now_rate_config_t rate_config = {
        .phymode = CONFIG_ESP_NOW_PHYMODE,
        .rate = CONFIG_ESP_NOW_RATE,
        .ersu = false,
        .dcm = false,
    };

    ESP_ERROR_CHECK(esp_now_add_peer(peer));
    ESP_ERROR_CHECK(esp_now_set_peer_rate_config(peer->peer_addr, &rate_config));
}

void app_main(void)
{
    esp_err_t ret = nvs_flash_init();
    if (ret == ESP_ERR_NVS_NO_FREE_PAGES || ret == ESP_ERR_NVS_NEW_VERSION_FOUND) {
        ESP_ERROR_CHECK(nvs_flash_erase());
        ret = nvs_flash_init();
    }
    ESP_ERROR_CHECK(ret);

    wifi_init();

    esp_now_peer_info_t peer = {
        .channel = 0,
        .ifidx = WIFI_IF_STA,
        .encrypt = false,
        .peer_addr = {0xff, 0xff, 0xff, 0xff, 0xff, 0xff},
    };
    wifi_esp_now_init(&peer);

    ESP_LOGI(TAG, "================ CSI TX (ESP-NOW) ================");
    ESP_LOGI(TAG, "channel: %d, frequency: %d Hz, mac: " MACSTR,
             CONFIG_ESPNOW_CHANNEL, CONFIG_SEND_FREQUENCY, MAC2STR(g_local_mac));

    uint32_t seq = 0;
    for (;; ++seq) {
        csi_ping_t pkt = {
            .seq = seq,
            .tx_us = esp_timer_get_time(),
        };
        esp_now_send(peer.peer_addr, (const uint8_t *)&pkt, sizeof(pkt));
        usleep(1000 * 1000 / CONFIG_SEND_FREQUENCY);
    }
}
