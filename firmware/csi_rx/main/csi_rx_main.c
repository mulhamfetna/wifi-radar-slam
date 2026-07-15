/*
 * csi_rx -- ESP32-S3 CSI receiver for the paper-4 static bench.
 *
 * Sits in promiscuous mode, captures HT40 CSI from a fixed sender MAC, and streams each
 * record to UART in the binary wire format (../../common/csi_wire.h). ALL processing is
 * offline on the laptop -- the chip only captures, stamps, filters, and ships.
 *
 * The config below sets the three settings that would otherwise silently kill the
 * experiment (see docs/hardware-build.md Part 4.1):
 *   channel_filter_en = false  -- else a subcarrier smoother low-passes the delay domain
 *   ltf_merge_en      = false  -- else LLTF (half-band in HT40) is averaged into HT-LTF
 *   manu_scale        = true   -- else AGC rescales CSI between the two recordings
 *
 * Build with ESP-IDF v5.x:  idf.py set-target esp32s3 && idf.py build flash monitor
 */
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
#include "driver/uart.h"

#include "csi_wire.h"

static const char *TAG = "csi_rx";

/* ------- user config (match the TX) -------------------------------------- */
#define CSI_CHANNEL      1                 /* 2.4 GHz channel; TX must match  */
#define CSI_UART_NUM     UART_NUM_0
#define CSI_UART_BAUD    2000000           /* 2 Mbaud; binary keeps us inside it */
/* The TX board's STA MAC -- set this to your TX's printed MAC. Filter on it. */
static const uint8_t TX_MAC[6] = {0x00, 0x00, 0x00, 0x00, 0x00, 0x00};

/* ------- CSI -> UART handoff --------------------------------------------- */
/* The CSI callback runs in the Wi-Fi task: do NOTHING there but enqueue. A writer task
 * drains the queue to UART so a slow UART never blocks the Wi-Fi task. */
typedef struct {
    csi_wire_header_t hdr;
    int8_t payload[CSI_HT40_PAYLOAD_BYTES];
} csi_msg_t;

static QueueHandle_t s_csi_queue;
static uint16_t s_seq;

static void csi_rx_cb(void *ctx, wifi_csi_info_t *info)
{
    (void)ctx;
    if (info == NULL || info->buf == NULL) {
        return;
    }
    /* Filter on the TX MAC (skip when TX_MAC is unset / all zero -- accept all). */
    static const uint8_t zero[6] = {0};
    if (memcmp(TX_MAC, zero, 6) != 0 && memcmp(info->mac, TX_MAC, 6) != 0) {
        return;
    }

    const wifi_pkt_rx_ctrl_t *rx = &info->rx_ctrl;
    const bool ht40 = (rx->cwb == 1) && (rx->sig_mode == 1) &&
                      (info->len == CSI_HT40_PAYLOAD_BYTES);

    csi_msg_t msg;
    csi_wire_header_t *h = &msg.hdr;
    h->magic[0] = CSI_WIRE_MAGIC0; h->magic[1] = CSI_WIRE_MAGIC1;
    h->magic[2] = CSI_WIRE_MAGIC2; h->magic[3] = CSI_WIRE_MAGIC3;
    h->seq          = s_seq++;
    h->timestamp_us = (uint32_t)esp_timer_get_time();
    h->rssi         = rx->rssi;
    /* agc_gain/fft_gain are NOT exposed by the public wifi_pkt_rx_ctrl_t in ESP-IDF v5.x --
     * they exist only via a reverse-engineered path (pyespargos). We instead pin the CSI
     * scale with manu_scale=true in the config, so no per-packet gain compensation is needed;
     * these two fields stay zero and the Python gain_linear() returns 1.0. The spare byte
     * carries noise_floor (dBm), which IS available and useful for an SNR sanity check. */
    h->agc_gain     = 0;
    h->fft_gain     = 0;
    h->sig_mode     = rx->sig_mode;
    h->cwb          = rx->cwb;
    h->n_sub        = CSI_HT40_N_SUB;
    h->valid        = ht40 ? 1 : 0;
    h->reserved     = (uint8_t)rx->noise_floor;

    /* Ship the RAW buffer; the laptop extracts the HT-LTF and does the fftshift. Only copy
     * a full HT40 record; anything shorter is a 20 MHz frame we do not want. */
    if (!ht40) {
        return;
    }
    memcpy(msg.payload, info->buf, CSI_HT40_PAYLOAD_BYTES);

    /* Non-blocking: if the queue is full we DROP and count it, never block the Wi-Fi task.
     * The CSI callback runs in the Wi-Fi task context (not an ISR), so plain xQueueSend. */
    if (xQueueSend(s_csi_queue, &msg, 0) != pdTRUE) {
        static uint32_t dropped;
        if ((++dropped % 100) == 0) {
            ESP_LOGW(TAG, "queue full, dropped %u", (unsigned)dropped);
        }
    }
}

static void uart_writer_task(void *arg)
{
    (void)arg;
    csi_msg_t msg;
    for (;;) {
        if (xQueueReceive(s_csi_queue, &msg, portMAX_DELAY) == pdTRUE) {
            uart_write_bytes(CSI_UART_NUM, (const char *)&msg.hdr, sizeof(msg.hdr));
            uart_write_bytes(CSI_UART_NUM, (const char *)msg.payload,
                             CSI_HT40_PAYLOAD_BYTES);
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

    /* HT40 on a fixed channel. No association -- sniffer mode gets the most CSI. */
    ESP_ERROR_CHECK(esp_wifi_set_channel(CSI_CHANNEL, WIFI_SECOND_CHAN_BELOW));
    ESP_ERROR_CHECK(esp_wifi_set_bandwidth(WIFI_IF_STA, WIFI_BW_HT40));
    ESP_ERROR_CHECK(esp_wifi_set_promiscuous(true));

    /* The load-bearing CSI config -- see the file header. */
    wifi_csi_config_t csi_config = {
        .lltf_en           = false,
        .htltf_en          = true,
        .stbc_htltf2_en    = false,
        .ltf_merge_en      = false,   /* CRITICAL: default true corrupts HT40 */
        .channel_filter_en = false,   /* CRITICAL: default true windows the CIR */
        .manu_scale        = true,    /* fix scaling so AGC cannot rescale between recordings */
        .shift             = 8,       /* tune so |CSI| uses the int8 range without clipping */
    };
    ESP_ERROR_CHECK(esp_wifi_set_csi_config(&csi_config));
    ESP_ERROR_CHECK(esp_wifi_set_csi_rx_cb(csi_rx_cb, NULL));
    ESP_ERROR_CHECK(esp_wifi_set_csi(true));
}

static void uart_init(void)
{
    uart_config_t uart_cfg = {
        .baud_rate  = CSI_UART_BAUD,
        .data_bits  = UART_DATA_8_BITS,
        .parity     = UART_PARITY_DISABLE,
        .stop_bits  = UART_STOP_BITS_1,
        .flow_ctrl  = UART_HW_FLOWCTRL_DISABLE,
        .source_clk = UART_SCLK_DEFAULT,
    };
    ESP_ERROR_CHECK(uart_driver_install(CSI_UART_NUM, 4096, 8192, 0, NULL, 0));
    ESP_ERROR_CHECK(uart_param_config(CSI_UART_NUM, &uart_cfg));
}

void app_main(void)
{
    ESP_ERROR_CHECK(nvs_flash_init());
    uart_init();

    s_csi_queue = xQueueCreate(64, sizeof(csi_msg_t));
    if (s_csi_queue == NULL) {
        ESP_LOGE(TAG, "queue alloc failed");
        return;
    }
    /* Big stack: the writer copies 384-byte payloads. */
    xTaskCreate(uart_writer_task, "csi_uart", 4096, NULL, 5, NULL);

    wifi_init();
    ESP_LOGI(TAG, "csi_rx up: channel %d, HT40, promiscuous, binary UART @ %d baud",
             CSI_CHANNEL, CSI_UART_BAUD);
}
