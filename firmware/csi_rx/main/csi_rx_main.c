/*
 * csi_rx -- ESP32-S3 CSI receiver for the paper-4 static bench.
 *
 * Promiscuous HT40 CSI capture from a fixed sender MAC, streamed to the host as binary
 * records (../../common/csi_wire.h) over UART0 -> the board's COM/FTDI port -> /dev/ttyUSB*.
 * ALL processing is offline on the laptop; the chip only captures, stamps, filters, ships.
 *
 * Verified on hardware (2026-07-17): with lltf_en+htltf_en (no merge) and the TX sending an
 * HT (MCS) rate, received packets are cwb=1 (40 MHz), sig=1 (HT), len=384 -- i.e. genuine
 * HT40 CSI (LLTF 64 + HT-LTF 128 = 192 complex). The parser skips the LLTF and uses the HT-LTF.
 *
 * Three load-bearing CSI config flags (docs/hardware-build.md Part 4.1):
 *   channel_filter_en = false  -- else a subcarrier smoother low-passes the delay domain
 *   ltf_merge_en      = false  -- else LLTF (half-band in HT40) is averaged into HT-LTF
 *   manu_scale        = true   -- else AGC rescales CSI between the two recordings
 *
 * Console + binary share UART0 at CSI_UART_BAUD; the 'CSI1' magic framing lets the Python
 * parser resync past the occasional log line, so mixing them is safe.
 */
#include <stdio.h>
#include <string.h>

#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "freertos/queue.h"

#include "esp_event.h"
#include "esp_log.h"
#include "esp_mac.h"
#include "esp_timer.h"
#include "esp_wifi.h"
#include "nvs_flash.h"

#include "csi_wire.h"

static const char *TAG = "csi_rx";

/* ------- user config (match the TX) -------------------------------------- */
#define CSI_CHANNEL   1                    /* 2.4 GHz channel; TX must match     */
/* Binary CSI shares the console UART. At the 115200 default this sustains ~28 HT40 records/s
 * (baud-limited: 384-byte records). Raising CONFIG_ESP_CONSOLE_UART_BAUDRATE lifts throughput
 * once a stable higher FTDI baud is dialed in -- see docs/results-paper4-first-light.md. */
/* TX board STA MAC (esptool read_mac): 28:84:85:48:40:20. Filter on it. */
static const uint8_t TX_MAC[6] = {0x28, 0x84, 0x85, 0x48, 0x40, 0x20};

/* One wire record = header + payload, contiguous, so it writes in a single call (atomic vs
 * an interleaving log line). */
typedef struct {
    csi_wire_header_t hdr;
    int8_t payload[CSI_HT40_PAYLOAD_BYTES];
} __attribute__((packed)) csi_msg_t;

static QueueHandle_t s_csi_queue;
static uint16_t s_seq;
static volatile uint32_t s_rx_ht40, s_dropped;

static void csi_rx_cb(void *ctx, wifi_csi_info_t *info)
{
    (void)ctx;
    if (info == NULL || info->buf == NULL) {
        return;
    }
    /* Filter on the TX MAC (all-zero TX_MAC accepts everything). */
    static const uint8_t zero[6] = {0};
    if (memcmp(TX_MAC, zero, 6) != 0 && memcmp(info->mac, TX_MAC, 6) != 0) {
        return;
    }

    const wifi_pkt_rx_ctrl_t *rx = &info->rx_ctrl;
    /* HT40 CSI with LLTF+HT-LTF enabled is 384 bytes (192 complex). Drop anything else. */
    if (!(rx->cwb == 1 && rx->sig_mode == 1 && info->len == CSI_HT40_PAYLOAD_BYTES)) {
        return;
    }
    s_rx_ht40++;

    csi_msg_t msg;
    csi_wire_header_t *h = &msg.hdr;
    h->magic[0] = CSI_WIRE_MAGIC0; h->magic[1] = CSI_WIRE_MAGIC1;
    h->magic[2] = CSI_WIRE_MAGIC2; h->magic[3] = CSI_WIRE_MAGIC3;
    h->seq          = s_seq++;
    h->timestamp_us = (uint32_t)esp_timer_get_time();
    h->rssi         = rx->rssi;
    h->agc_gain     = 0;   /* not in the public rx_ctrl; manu_scale pins the scale instead */
    h->fft_gain     = 0;
    h->sig_mode     = rx->sig_mode;
    h->cwb          = rx->cwb;
    h->n_sub        = CSI_HT40_N_SUB;
    h->valid        = 1;
    h->reserved     = (uint8_t)rx->noise_floor;
    memcpy(msg.payload, info->buf, CSI_HT40_PAYLOAD_BYTES);

    /* Non-blocking: never stall the Wi-Fi task. Drop on a full queue and count it. */
    if (xQueueSend(s_csi_queue, &msg, 0) != pdTRUE) {
        s_dropped++;
    }
}

static void writer_task(void *arg)
{
    (void)arg;
    csi_msg_t msg;
    for (;;) {
        if (xQueueReceive(s_csi_queue, &msg, portMAX_DELAY) == pdTRUE) {
            /* one contiguous write of the whole record (header+payload) to stdout=UART0 */
            fwrite(&msg, 1, sizeof(msg), stdout);
            fflush(stdout);
        }
    }
}

static void wifi_init(void)
{
    ESP_ERROR_CHECK(esp_netif_init());
    ESP_ERROR_CHECK(esp_event_loop_create_default());

    wifi_init_config_t cfg = WIFI_INIT_CONFIG_DEFAULT();
    ESP_ERROR_CHECK(esp_wifi_init(&cfg));
    ESP_ERROR_CHECK(esp_wifi_set_storage(WIFI_STORAGE_RAM));
    ESP_ERROR_CHECK(esp_wifi_set_mode(WIFI_MODE_STA));
    ESP_ERROR_CHECK(esp_wifi_start());

    /* HT40 on a fixed channel. Channel 1's secondary must be ABOVE (below would be ch -3). */
    ESP_ERROR_CHECK(esp_wifi_set_channel(CSI_CHANNEL, WIFI_SECOND_CHAN_ABOVE));
    ESP_ERROR_CHECK(esp_wifi_set_bandwidth(WIFI_IF_STA, WIFI_BW_HT40));
    ESP_ERROR_CHECK(esp_wifi_set_promiscuous(true));

    /* lltf_en=true -> full 384-byte buffer the parser expects; ltf_merge stays false. */
    wifi_csi_config_t csi_config = {
        .lltf_en           = true,
        .htltf_en          = true,
        .stbc_htltf2_en    = false,
        .ltf_merge_en      = false,   /* CRITICAL: default true corrupts HT40 */
        .channel_filter_en = false,   /* CRITICAL: default true windows the CIR */
        .manu_scale        = true,    /* fix scaling so AGC cannot rescale between recordings */
        .shift             = 8,
    };
    ESP_ERROR_CHECK(esp_wifi_set_csi_config(&csi_config));
    ESP_ERROR_CHECK(esp_wifi_set_csi_rx_cb(csi_rx_cb, NULL));
    ESP_ERROR_CHECK(esp_wifi_set_csi(true));
}

void app_main(void)
{
    ESP_ERROR_CHECK(nvs_flash_init());
    setvbuf(stdout, NULL, _IOFBF, 2048);   /* full buffering: fewer, larger UART writes */

    s_csi_queue = xQueueCreate(64, sizeof(csi_msg_t));
    if (s_csi_queue == NULL) {
        ESP_LOGE(TAG, "queue alloc failed");
        return;
    }
    xTaskCreate(writer_task, "csi_writer", 4096, NULL, 5, NULL);

    wifi_init();
    ESP_LOGI(TAG, "csi_rx up: ch %d HT40, filtering %02x:%02x:%02x:%02x:%02x:%02x",
             CSI_CHANNEL, TX_MAC[0], TX_MAC[1], TX_MAC[2], TX_MAC[3], TX_MAC[4], TX_MAC[5]);

    /* Rare health heartbeat -- the parser skips non-CSI1 text. */
    for (;;) {
        vTaskDelay(pdMS_TO_TICKS(5000));
        ESP_LOGI(TAG, "rx_ht40=%lu dropped=%lu",
                 (unsigned long)s_rx_ht40, (unsigned long)s_dropped);
    }
}
