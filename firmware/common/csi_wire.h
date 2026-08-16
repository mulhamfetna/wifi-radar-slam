/*
 * csi_wire.h -- the binary CSI wire format, shared by the ESP32 firmware and the Python
 * parser (src/wifi_radar_slam/hw/csi.py). ONE definition, two readers -- so the firmware and
 * the offline parser can never silently disagree (the hazard the design doc warns about).
 *
 * Binary, not ASCII: the stock esp-csi example prints CSI as ~2000 chars/record and drops
 * packets at 256 subcarriers (esp-csi #249). A packed binary record is ~75% smaller.
 *
 * The RX firmware ships the RAW ESP32 CSI buffer (LLTF + HT-LTF, imag-first int8 pairs). The
 * Python side extracts the HT-LTF and applies the fftshift, deriving its own index order --
 * because Espressif's own parser ordering is disputed and still open (esp-csi #224). Do NOT
 * reorder on the chip.
 *
 * All multi-byte fields are little-endian (ESP32 is little-endian; x86/ARM laptops are too).
 */
#ifndef CSI_WIRE_H
#define CSI_WIRE_H

#include <stdint.h>

#define CSI_WIRE_MAGIC0 'C'
#define CSI_WIRE_MAGIC1 'S'
#define CSI_WIRE_MAGIC2 'I'
#define CSI_WIRE_MAGIC3 '1'

/* HT40 raw CSI buffer: 192 complex subcarriers (64 LLTF + 128 HT-LTF) = 384 int8 bytes. */
#define CSI_HT40_N_SUB 192
#define CSI_HT40_PAYLOAD_BYTES (CSI_HT40_N_SUB * 2)

/*
 * 18-byte header, then payload = n_sub complex pairs (imag int8, real int8).
 * __attribute__((packed)) so the on-wire layout is exactly these 18 bytes with no padding;
 * the Python struct format "<4sHIbBBBBBBB" must match this field-for-field.
 */
typedef struct __attribute__((packed)) {
    uint8_t  magic[4];      /* 'C','S','I','1'                              */
    uint16_t seq;           /* rolling packet counter                       */
    uint32_t timestamp_us;  /* esp_timer_get_time() low 32 bits (microsec)  */
    int8_t   rssi;          /* rx_ctrl.rssi                                 */
    uint8_t  agc_gain;      /* rx_ctrl.agc_gain  (AGC compensation)         */
    uint8_t  fft_gain;      /* rx_ctrl.fft_gain                             */
    uint8_t  sig_mode;      /* rx_ctrl.sig_mode  (0 non-HT, 1 HT)           */
    uint8_t  cwb;           /* rx_ctrl.cwb       (0 = 20 MHz, 1 = 40 MHz)   */
    uint8_t  n_sub;         /* complex subcarriers in payload (192 for HT40)*/
    uint8_t  valid;         /* firmware sanity: cwb==1 && sig_mode==1 && len==384 */
    uint8_t  reserved;      /* pad to 18 bytes                              */
} csi_wire_header_t;

#endif /* CSI_WIRE_H */
