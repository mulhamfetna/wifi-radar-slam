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
#include "esp_private/wifi.h"     /* esp_wifi_internal_set_fix_rate (fix the TX MCS/BW) */
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

    ESP_ERROR_CHECK(esp_wifi_set_channel(CSI_CHANNEL, WIFI_SECOND_CHAN_ABOVE));
    ESP_ERROR_CHECK(esp_wifi_set_bandwidth(WIFI_IF_STA, WIFI_BW_HT40));

    /* Force an HT (802.11n) MCS rate on the 80211_tx path. This is what makes the transmitted
     * frame carry an HT-LTF -- without it the stack sends LEGACY (non-HT) frames that have no
     * HT-LTF, so a receiver with htltf_en/lltf_en=false generates NO CSI at all. Use the PUBLIC
     * esp_wifi_config_80211_tx_rate (the internal set_fix_rate aborts on the unassociated STA).
     * MCS7 LGI at HT40; the boards are inches apart so the high MCS decodes fine. */
    esp_wifi_config_11b_rate(WIFI_IF_STA, false);   /* disable legacy 11b rates */
    esp_err_t rerr = esp_wifi_config_80211_tx_rate(WIFI_IF_STA, WIFI_PHY_RATE_MCS7_LGI);
    if (rerr != ESP_OK) {
        ESP_LOGW(TAG, "config_80211_tx_rate failed (%d) -- frames may be legacy (no HT-LTF)", rerr);
    }

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
    uint32_t ok = 0, fail = 0, iter = 0;
    /* Pace with vTaskDelay so the task YIELDS (a busy-wait starves IDLE -> task watchdog).
     * 100 Hz = 10 ms; the default tick is coarse enough that vTaskDelay is perfect here. */
    const TickType_t period = pdMS_TO_TICKS(1000 / TX_RATE_HZ);
    for (;;) {
        esp_err_t err = esp_wifi_80211_tx(WIFI_IF_STA, s_frame, sizeof(s_frame), true);
        if (err == ESP_OK) ok++; else fail++;
        if ((++iter % 100) == 0) {
            ESP_LOGI(TAG, "tx ok=%lu fail=%lu (last err %d)",
                     (unsigned long)ok, (unsigned long)fail, err);
        }
        vTaskDelay(period > 0 ? period : 1);
    }
}

void app_main(void)
{
    ESP_ERROR_CHECK(nvs_flash_init());
    wifi_init();
    xTaskCreate(tx_task, "csi_tx", 4096, NULL, 5, NULL);
    ESP_LOGI(TAG, "csi_tx up: channel %d, HT40, MCS7, %d Hz", CSI_CHANNEL, TX_RATE_HZ);
}
