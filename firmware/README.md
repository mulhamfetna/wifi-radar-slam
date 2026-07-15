# Paper 4 firmware — ESP32-S3 CSI bench (ESP-IDF C on FreeRTOS)

Two minimal apps. The chip only **captures, stamps, filters, and ships**; all signal
processing is offline in Python (`src/wifi_radar_slam/hw/`). See
`docs/paper4-architecture-python-vs-c.md` for why the split is drawn here.

```
firmware/
  common/csi_wire.h   -- the binary wire format, shared with the Python parser (hw/csi.py)
  csi_tx/             -- illuminator: emits HT40 802.11n data frames at ~100 Hz
  csi_rx/             -- receiver: promiscuous CSI capture -> binary UART stream
```

## Requirements
- **ESP-IDF v5.x** installed and exported (`. $IDF_PATH/export.sh`).
- **2× ESP32-S3** boards (see `docs/hardware-build.md` for why S3 and not plain ESP32 / C5 / C6).

## Build & flash

```bash
# receiver
cd firmware/csi_rx
idf.py set-target esp32s3
idf.py build flash monitor       # note: CSI streams as BINARY on UART0; logs go to USB-JTAG

# transmitter (second board / second terminal)
cd firmware/csi_tx
idf.py set-target esp32s3
idf.py build flash monitor       # prints the TX MAC on boot
```

## Wiring the two boards together
1. Flash `csi_tx`, read its printed **TX MAC** from the monitor.
2. Paste that MAC into `csi_rx/main/csi_rx_main.c` → `TX_MAC[6]`, rebuild & flash `csi_rx`.
   (Leave `TX_MAC` all-zero to accept *all* senders — handy for first light.)
3. Both boards on **channel 1, HT40**. Place them 0.5 m apart, static, pointing down the
   corridor (see `docs/hardware-build.md`).

## The three load-bearing config choices (csi_rx)
`channel_filter_en=false`, `ltf_merge_en=false`, `manu_scale=true`. All three are **not** the
defaults; two are left ON in Espressif's own example and would **silently** destroy the echo
(they smooth the delay domain / corrupt HT40 / let AGC rescale between recordings). See
`docs/hardware-build.md` Part 4.1.

## Capturing to the laptop
The RX writes the binary wire records (18-byte header + 384-byte payload each) to UART0. Log
the raw bytes to a file, then parse offline:

```bash
# example: dump the serial device to a file (adjust the port)
cat /dev/ttyACM0 > capture_plate_in.bin        # then move the plate and capture plate_out
```

```python
from wifi_radar_slam.hw.csi import parse_stream
records = parse_stream(open("capture_plate_in.bin", "rb").read())
# each record.ht_ltf(cfg) is a 128-bin fftshifted CSI vector -> feed hw.delay
```

## The HT40 transmit trap
`esp_wifi_set_bandwidth(HT40)` only sets *capability*. Only an **HT (11n) data frame at an MCS
rate** actually produces an HT40 CSI record on the receiver — which is why `csi_tx` fixes the
rate to MCS7 and sends via `esp_wifi_80211_tx()`. On the RX we additionally verify
`cwb==1 && sig_mode==1 && len==384` and drop anything else. (esp-csi #52.)
