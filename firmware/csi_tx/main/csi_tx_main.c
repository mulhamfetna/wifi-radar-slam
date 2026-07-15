/*
 * csi_tx -- ESP32-S3 illuminator for the paper-4 static bench.
 *
 * Emits HT (802.11n) data frames at 40 MHz, at a steady rate, from a fixed source MAC. The
 * RX board filters on this MAC and estimates CSI from these frames.
 *
 * THE HT40 TRAP (esp-csi #52): esp_wifi_set_bandwidth() only sets the *capability*. The CSI
 * bandwidth is set by the packet actually sent -- management, broadcast, and legacy-rate
 * frames are all 20 MHz. So we transmit an 802.11n DATA frame at an MCS rate via
 * esp_wifi_80211_tx(), which is what actually produces an HT40 record on the receiver.
 *
 * Build with ESP-IDF v5.x:  idf.py set-target esp32s3 && idf.py build flash monitor
 */
#include <string.h>

#include "freertos/FreeRTOS.h"
#include "freertos/task.h"

#include "esp_event.h"
#include "esp_log.h"
#include "esp_mac.h"
#include "esp_wifi.h"
#include "nvs_flash.h"
#include "rom/ets_sys.h"          /* ets_delay_us -- pace with usleep-style, not vTaskDelay */

static const char *TAG = "csi_tx";

#define CSI_CHANNEL   1
#define TX_RATE_HZ    100          /* ~100 frames/s */

/*
 * A minimal 802.11n data frame. The addresses must match what the RX filters on: addr2 is
 * the source (this board's MAC). A short payload is enough -- CSI is estimated from the
 * preamble/HT-LTF, not the payload.
 */
static uint8_t s_frame[] = {
    0x08, 0x01,                         /* frame control: data, to-DS                     */
    0x00, 0x00,                         /* duration                                       */
    0xff, 0xff, 0xff, 0xff, 0xff, 0xff, /* addr1: broadcast (receiver accepts in promisc) */
    0x00, 0x00, 0x00, 0x00, 0x00, 0x00, /* addr2: SOURCE (this board) -- filled at boot   */
    0x00, 0x00, 0x00, 0x00, 0x00, 0x00, /* addr3                                          */
    0x00, 0x00,                         /* sequence control                               */
    /* payload */
    'W', 'I', 'F', 'I', 'R', 'A', 'D', 'A', 'R', '4',
};

static void wifi_init(void)
{
    ESP_ERROR_CHECK(esp_netif_init());
    ESP_ERROR_CHECK(esp_event_loop_create_default());

    wifi_init_config_t cfg = WIFI_INIT_CONFIG_DEFAULT();
    ESP_ERROR_CHECK(esp_wifi_init(&cfg));
    ESP_ERROR_CHECK(esp_wifi_set_storage(WIFI_STORAGE_RAM));
    ESP_ERROR_CHECK(esp_wifi_set_mode(WIFI_MODE_STA));
    ESP_ERROR_CHECK(esp_wifi_start());

    ESP_ERROR_CHECK(esp_wifi_set_channel(CSI_CHANNEL, WIFI_SECOND_CHAN_BELOW));
    ESP_ERROR_CHECK(esp_wifi_set_bandwidth(WIFI_IF_STA, WIFI_BW_HT40));

    /* Fix the data rate to an HT (11n) MCS at 40 MHz -- this is what yields an HT40 CSI
     * record on the receiver. Without it the stack may fall back to a legacy 20 MHz rate. */
    esp_wifi_config_11b_rate(WIFI_IF_STA, false);
    ESP_ERROR_CHECK(esp_wifi_internal_set_fix_rate(WIFI_IF_STA, true, WIFI_PHY_RATE_MCS7_SGI));

    /* Put our own MAC into addr2 so the RX can filter on it -- print it so you can paste it
     * into the RX firmware's TX_MAC. */
    uint8_t mac[6];
    ESP_ERROR_CHECK(esp_wifi_get_mac(WIFI_IF_STA, mac));
    memcpy(&s_frame[10], mac, 6);
    ESP_LOGI(TAG, "TX MAC (paste into csi_rx TX_MAC): %02x:%02x:%02x:%02x:%02x:%02x",
             mac[0], mac[1], mac[2], mac[3], mac[4], mac[5]);
}

static void tx_task(void *arg)
{
    (void)arg;
    const uint32_t period_us = 1000000U / TX_RATE_HZ;
    for (;;) {
        /* en_sys_seq = true: let the stack assign the sequence number. */
        esp_err_t err = esp_wifi_80211_tx(WIFI_IF_STA, s_frame, sizeof(s_frame), true);
        if (err != ESP_OK) {
            ESP_LOGW(TAG, "80211_tx err %d", err);
        }
        ets_delay_us(period_us);   /* busy-wait pacing, per Espressif advice (esp-csi #114) */
    }
}

void app_main(void)
{
    ESP_ERROR_CHECK(nvs_flash_init());
    wifi_init();
    xTaskCreate(tx_task, "csi_tx", 4096, NULL, 5, NULL);
    ESP_LOGI(TAG, "csi_tx up: channel %d, HT40, MCS7, %d Hz", CSI_CHANNEL, TX_RATE_HZ);
}
