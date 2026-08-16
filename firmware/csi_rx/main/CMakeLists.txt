idf_component_register(SRCS "csi_rx_main.c"
                       INCLUDE_DIRS "." "../../common"
                       REQUIRES esp_wifi nvs_flash esp_timer driver esp_netif esp_event)
