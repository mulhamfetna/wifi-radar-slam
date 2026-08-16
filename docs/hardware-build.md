# Paper 4 — hardware build: bill of materials, wiring, firmware, survey

**Date:** 2026-07-15
**Branch:** `paper4-hardware-testbed`
**Companion to:** `docs/paper4-restart-static-bench.md` (the design) and
`docs/results-paper4-rung0.md` (the offline pipeline, already built and tested).

> **The whole rig is: two ESP32-S3 boards, a baking tray, a tape measure, and a long
> corridor. ~$31.** Everything else on your bench is a laptop you already own.

---

## 1. Bill of materials

### 1.1 Critical path — Rungs 0.5 → 3 (does an echo exist, and does it range?)

| # | item | exact spec — and **why this one** | qty | ~USD |
|---|---|---|---|---|
| 1 | **ESP32-S3 dev board** | ESP32-**S3** (e.g. ESP32-S3-DevKitC-1 / -DevKitM-1). Must give **HT40 (40 MHz) CSI on 2.4 GHz**, **promiscuous mode**, USB-serial. One is TX, one is RX. | **2** | ~$8 ea → **$16** |
| 2 | **flat metal reflector** | flat conductive plate **≥ 30 × 30 cm** — a baking tray/cookie sheet is ideal. Bigger = stronger echo; **flatness matters** (specular return, not scatter). | 1–2 | ~$5 |
| 3 | **tape measure** | **the ground-truth instrument.** ≥ 15 m; a laser rangefinder (±cm) is better. Everything is scored against it. | 1 | ~$5 |
| 4 | **USB data cables** | to each board's connector (USB-C or micro-USB — check the board). **Data-capable**, not charge-only. | 2 | ~$5 |
| 5 | **laptop** | Python 3 + the repo `.venv`. Does **all** processing offline. Already owned. | 1 | — |
| | | | **total** | **~$31** |

### 1.2 Additionally for Rung 4+ (the phantom rate — the headline number)

| item | spec |
|---|---|
| **a large surveyed space** | sports hall / long atrium / car-park lane. **Every reflector tape-measured.** Reflectors must sit **well beyond 3.75 m** (one range cell) or they collapse into the direct-path bin. |
| *(no new hardware)* | Rungs 4, 5 (MUSIC vs CFAR) and 6 (monostatic vs bistatic) all come out of **one static recording** with the same two boards. |

### 1.3 ⛔ Chip choice — read before buying

| chip | verdict | reason |
|---|---|---|
| **ESP32-S3** | ✅ **BUY THIS** | Correct 256/384-byte CSI buffers. Espressif's own engineer recommends S3 for CSI (esp-csi #146). |
| plain ESP32 | ⚠️ works, weakest CSI | "performance of esp32 is comparatively lower" — same source. |
| ESP32-C3 | ❌ HT20 only | half the bandwidth → the resolution cell doubles to 7.5 m. |
| **ESP32-C6** | ❌❌ broken | returns 128 bytes (no L-LTF) and **wrong HT-LTF ordering** — esp-idf #14271, still open. |
| **ESP32-C5** | ❌❌ trap | wants a **48 MHz** crystal and has an unresolved shared-clock boot failure. Different `wifi_csi_config_t` — none of the config below applies. |

---

## 2. The site requirement (a hard constraint, not a purchase)

> **A straight, clear corridor ≥ 15 m (20 m better).**

At 40 MHz one **range** cell is 3.75 m. A plate closer than ~4 m collapses into the
direct-path bin and you measure **nothing**. The geometry (design doc Part 4.2):

| plate at | excess path (2d − 0.5) | excess delay | cells from LOS | verdict |
|---|---|---|---|---|
| 4 m | 7.5 m | 25 ns | 1.0 | ❌ buried in LOS |
| 6 m | 11.5 m | 38 ns | 1.53 | ⚠️ in the LOS skirt |
| 8 m | 15.5 m | 52 ns | 2.07 | ⚠️ marginal |
| **12 m** | **23.5 m** | **78 ns** | **3.13** | ✅ **start here** |
| 14 m | 27.5 m | 92 ns | 3.67 | ✅ |

A university hallway, a warehouse aisle, or an empty car-park lane works. **A small lab does
not.**

---

## 3. Wiring and layout

```
        TX ESP32-S3                         RX ESP32-S3
        (illuminator)                       (CSI logger)
             |                                   |
        USB to laptop                       USB to laptop
             |                                   |
             +---------- baseline b = 0.5 m ------+     (tape-measured)
                              |
                              |  (both antennas pointed down the corridor)
                              v
        ============ corridor, >= 15 m ============
                                              [ metal plate ]  <- at d, tape-measured
```

- **Baseline `b = 0.5 m`**, measured with the tape. At 40 MHz this 0.5 m is **6.7 % of one
  path cell** — invisible — so the rig is **monostatic by measurement** (design doc Part 4.3).
- Both boards **static** on a table/tripod. **No motion. No servo. No angle.** (Angle is
  Rung 7, later.)
- Keep the plate **on the corridor axis**, face toward the boards.
- **Do not move or reset the boards between the plate-in and plate-out recordings** — the PLL
  re-locks with a random phase on reset (design doc Part 5 caveats).

---

## 4. Firmware (`firmware/`, ESP-IDF C on FreeRTOS)

Two apps. Both minimal. See `firmware/README.md` for build/flash.

### 4.1 The RX config — 🚨 three defaults that would silently kill the experiment

```c
wifi_csi_config_t csi_config = {
    .lltf_en           = false,  // HT40 LLTF covers only HALF the band -- useless AND dangerous
    .htltf_en          = true,   // THIS is the contiguous 40 MHz block we IFFT
    .stbc_htltf2_en    = false,
    .ltf_merge_en      = false,  // ** CRITICAL ** default true -> averages LLTF into HT-LTF -> corrupts 20 of 40 MHz
    .channel_filter_en = false,  // ** CRITICAL ** default true -> smooths subcarriers = LOW-PASS in the delay domain -> kills late taps
    .manu_scale        = true,   // fix the scaling so AGC cannot rescale between the two recordings
    .shift             = 8,      // tune so |CSI| uses the int8 range without clipping
};
```

`channel_filter_en` and `ltf_merge_en` are **ON by default, and left on in Espressif's own
example.** With the stock config you would **never see the echo, and get no error.**

### 4.2 The RX pipeline (what the firmware does)

1. `esp_wifi_set_promiscuous(true)` — **no association needed** (docs recommend against it).
2. Register the CSI callback; **do nothing in it but `xQueueSend`** (it runs in the Wi-Fi task).
3. For every CSI record, **verify `rx_ctrl.cwb == 1`, `sig_mode == 1`, `len == 384`** — discard
   anything else (management/broadcast/legacy frames are 20 MHz).
4. Pack each record into the **binary wire format** (`firmware/common/csi_wire.h`) and write it
   to UART. **Binary, not ASCII** — the stock ASCII print (~2000 chars/record) drops packets at
   256 subcarriers (esp-csi #249).
5. Record the per-packet `agc_gain` and `fft_gain` in the header (AGC compensation, design doc
   Part 7.3).

### 4.3 The TX illuminator

- `esp_wifi_set_bandwidth(WIFI_BW_HT40)` **and actually transmit HT (802.11n) data frames at an
  MCS rate** — capability alone gives 20 MHz packets (esp-csi #52). A fixed source MAC the RX
  filters on. Steady rate (~100 Hz), paced with `usleep()` not `vTaskDelay()`.

---

## 5. The measurement procedure

### 5.1 Rung 0.5 — the ordering check (30 minutes, do this FIRST)

Empty corridor, **no plate**, one HT40 capture. Run it through
`hw.delay.raw_cir` → plot `|CIR|`. **You must see exactly ONE dominant tap.** Two peaks, or a
peak at negative delay, means the subcarrier ordering is wrong (Espressif's parser is disputed
— esp-csi #224). `hw.csi` exposes both half-orderings; pick the one that gives a single tap.

> This one hour stands between you and **weeks** of chasing a reordering bug that looks exactly
> like a phantom reflection.

### 5.2 Rung 1 — the atomic test (does a wall echo exist?)

Plate at **12 m**. Record **interleaved, back-to-back, without resetting the boards**:

```
plate-IN (1000 pkts) → plate-OUT (1000) → plate-IN (1000) → plate-OUT (1000)
```

Interleaving makes thermal drift **detectable** instead of silent. Then, on the laptop:
`differential(plate_in, plate_out)` → `detect_delays`. **Prediction: a clear tap at 78 ns.**
**Kill:** no tap at 10, 12 *and* 14 m → report the negative result; the hardware programme
stops (a genuine, citable finding — nobody has resolved an ESP32 reflector tap before).

### 5.3 The survey (for Rung 4)

- Tape-measure the sensor pose and **every** reflector (walls, pillars, radiators, lockers,
  doors) in the large space.
- Feed the surveyed geometry to `radar/truth.py` to predict the **true** echo delays.
- Score detections against them with `eval/phantom.py`'s `phantom_stats_frames` — the **exact**
  definition papers 2 and 3 use, so the number is comparable to 89 % / 18.2 % / 0.1 %.

---

## 6. What each rung costs, at a glance

| rungs | hardware | site | new spend |
|---|---|---|---|
| 0 (done) | none | none | $0 |
| 0.5 – 3 | 2× ESP32-S3 + plate + tape | 15 m corridor | ~$31 |
| 4 – 6 | *(same two boards)* | large surveyed space | $0 |
| 7 (angle, later) | + servo + directional antenna | same | ~$13 |
| 8 (motion, later) | + a moving platform | — | TBD |

**Angle (Rung 7) and motion (Rung 8) are only reached if Rungs 1–6 pass**, and even then we do
**not** claim SLAM (design doc Part 2).
