# Paper 4 — FIRST LIGHT: real ESP32-S3 CSI through the full pipeline

**Date:** 2026-07-17
**Branch:** `paper4-hardware-testbed`
**Status:** ✅ **The RF link works end-to-end on real silicon.** Two ESP32-S3 boards; genuine
HT40 CSI captured, parsed, and turned into a delay profile.

> This is the Rung-0.5 milestone (the ordering / first-light check) from
> `docs/paper4-restart-static-bench.md`, achieved on hardware.

---

## What works, verified on hardware

**The chain, all real:**
```
TX ESP32-S3 (HT40, MCS7, 100 frames/s)
   → RF →
RX ESP32-S3 (promiscuous, HT40 CSI, 384-byte records)
   → binary UART0 → FTDI/COM port → /dev/ttyUSB0 →
laptop: parse_stream → HT-LTF (128 subcarriers) → LOS-align → delay profile
```

**Measured (capture `data/hw_captures/first_light_los_2026-07-17.bin`, 5 s):**

| quantity | value | meaning |
|---|---|---|
| valid HT40 records parsed | **140** (of 145 magics) | the binary wire format + parser work |
| `cwb` / `sig_mode` | **1 / 1** | genuine **40 MHz, HT** (not 20 MHz, not legacy) |
| record length | **384 bytes** | LLTF(64) + HT-LTF(128) = 192 complex, as the parser expects |
| RSSI | **−29 dBm** | strong (boards inches apart) |
| HT-LTF subcarriers | **128, of which 114 nonzero** | **exactly** matches the config's active mask (14 dead = 3 DC-notch + 11 guard) |
| coherent average of 140 packets | **LOS peak at 0.00 m** | LOS-alignment works on real data |
| CSI amplitude across band | 173–179 (nearly flat) | **correct**: adjacent boards = pure line-of-sight, no reflector → a single clean tap |

The near-flat channel and single LOS tap are the *expected* Rung-0.5 result for two boards on a
desk with nothing to reflect off. The ordering (parser `order="A"`) is validated: the LOS aligns
to delay 0 with no spurious second peak, and the active-subcarrier count is exactly right.

---

## The bring-up: bugs found and fixed *on silicon*

The firmware compiled clean but real hardware surfaced six issues, each fixed:

| # | symptom | root cause | fix |
|---|---|---|---|
| 1 | flashed but never ran; `waiting for download` forever | **native USB port** strapping → download mode (see `docs/esp32-s3-usb-vs-com-port.md`) | use the **COM/FTDI port** |
| 2 | 0 bytes over the COM port | firmware streamed over the *native USB*, not UART0 | route console + CSI to **UART0** |
| 3 | boot-loop, abort at `set_channel` | HT40 secondary channel can't be **BELOW** on channel 1 | `WIFI_SECOND_CHAN_ABOVE` |
| 4 | boot-loop, abort at `set_fix_rate` | internal fixed-rate call aborts on an unassociated STA | use public `esp_wifi_config_80211_tx_rate`, non-fatal |
| 5 | RX received packets but **0 CSI callbacks** | TX sent **legacy** frames (no HT-LTF); RX had `lltf_en=false` so no CSI | force an **HT (MCS) rate** on the TX |
| 6 | CSI fired but **`ht40=0`** (len 256 ≠ 384) | `lltf_en=false` → HT-LTF-only 256-byte buffer, not the 384 the parser expects | `lltf_en=true` (LLTF included, `ltf_merge` still false) |

**Diagnostic method:** a heartbeat build that counted promiscuous packets, CSI callbacks, and
reported the exact `cwb`/`sig_mode`/`len` of received packets — which is how #5 and #6 were
pinned precisely rather than guessed.

---

## Board inventory

| board (MAC) | role | firmware | COM tty |
|---|---|---|---|
| `28:84:85:48:40:20` | TX (illuminator) | `csi_tx` — HT40 MCS7, 100 Hz | `/dev/ttyUSB1` |
| `28:84:85:53:99:DC` | RX (CSI logger) | `csi_rx` — HT40 CSI → binary UART | `/dev/ttyUSB0` |

Both genuine ESP32-S3 (QFN56 rev v0.2, 8 MB PSRAM), on FTDI FT232R COM ports.

---

## Known limitations / next steps

1. **Throughput is baud-limited.** The binary CSI shares the console UART at 115200 →
   ~28 HT40 records/s. Coherent averaging over ~1000 packets is therefore ~35 s per recording.
   A stable higher `CONFIG_ESP_CONSOLE_UART_BAUDRATE` (the FTDI handled 921600 poorly in a first
   attempt; 460800 to be dialed in) lifts this to the full 100/s.
2. **Rung 0.5 proper** — repeat the single-tap check in a real corridor (boards 0.5 m apart,
   nothing within a few metres) to confirm a clean LOS tap and pick the subcarrier order.
3. **Rung 1 (the atomic test)** — boards 0.5 m apart, a metal plate at 12 m in a ≥15 m corridor,
   capture plate-in / plate-out interleaved, run `experiments/run_hw_phantom.py` on the two
   `.bin` files. Prediction: a differential tap at ~78 ns (23.5 m path).

**The offline pipeline that will process all of this is already built and tested** (rung 0,
`src/wifi_radar_slam/hw/`), and it just parsed real silicon output correctly for the first time.
