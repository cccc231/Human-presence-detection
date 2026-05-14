idf_component_register(SRCS "rx_main.c" "csi_processor.c"
                       INCLUDE_DIRS "."
                       PRIV_REQUIRES nvs_flash esp_wifi esp_netif esp_event esp_timer)
