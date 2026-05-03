#include "time_utils.h"
#include "esp_log.h"
#include "esp_netif_sntp.h"
#include "esp_timer.h"
#include <time.h>
#include <sys/time.h>

static const char *TAG = "time_utils";
static bool time_synced = false;

static void time_sync_notification_cb(struct timeval *tv)
{
    ESP_LOGI(TAG, "NTP time synchronized");
    time_synced = true;
}

void time_utils_init(void)
{
    ESP_LOGI(TAG, "Initializing SNTP...");

    setenv("TZ", "CST-8", 1);
    tzset();

    esp_sntp_config_t config = ESP_NETIF_SNTP_DEFAULT_CONFIG("ntp.aliyun.com");
    config.sync_cb = time_sync_notification_cb;
    config.smooth_sync = false;
    config.server_from_dhcp = false;
    config.num_of_servers = 2;
    config.servers[0] = "ntp.aliyun.com";
    config.servers[1] = "ntp.tencent.com";

    esp_netif_sntp_init(&config);
}

void get_beijing_time_string(char *buf, size_t len)
{
    if (!time_synced) {
        int64_t us_since_boot = esp_timer_get_time();
        int64_t sec_since_boot = us_since_boot / 1000000LL;
        snprintf(buf, len, "SYNCING_%llds", (long long)sec_since_boot);
        return;
    }

    time_t now;
    struct tm timeinfo;
    time(&now);
    localtime_r(&now, &timeinfo);
    strftime(buf, len, "%Y-%m-%d %H:%M:%S", &timeinfo);
}
