idf_component_register(SRCS "csi_tx_main.c"
                       INCLUDE_DIRS "." "../../common"
                       REQUIRES esp_wifi nvs_flash esp_netif esp_event)
