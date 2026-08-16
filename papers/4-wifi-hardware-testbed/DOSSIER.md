# Paper 4 — Dossier

**Working title:** *A $30 WiFi Radar: Measuring the Phantom Ceiling on Real ESP32 Silicon*
**Author:** Mulham Fetna (ORCID 0009-0006-4432-798X)
**Status:** **ACTIVE — hardware bring-up complete, first light achieved.** Branch
`paper4-hardware-testbed` (off `paper3-wifi-vs-radar`). Awaiting a field site for Rung 1.

This dossier is paper 4's durable, in-repo record — independent of Claude's (non-branch-aware)
memory. Read it first when resuming paper 4.

---

## One-paragraph summary

Papers 1–3 are entirely ray-traced. Paper 4 moves the programme's headline number — **the
phantom rate** (the fraction of detections matching no real reflector) — onto **real ESP32-S3
silicon**, using the cheapest possible rig: **two ESP32-S3 boards (~$16), a metal plate, and a
tape measure.** One board illuminates (HT40, 40 MHz), one logs CSI; the laptop turns the CSI
into a delay profile and measures echoes against a tape-measured ground truth. It is a **static
bench**, not a moving vehicle — the phantom rate is a property of a single pose, so no motion,
map, or SLAM is needed. We validate **the reading**, and we explicitly do **not** claim a SLAM
system (four independent, cited reasons — see `docs/paper4-restart-static-bench.md` §0/Part 2).

---

## The through-line (why this paper exists)

All three prior papers reduce to one scalar: **the phantom rate.**
- MUSIC + bistatic geometry → **≈89 %** phantoms → mapping destroyed (paper 2).
- CFAR + monostatic geometry → **≈0.1 %** → mapping works (paper 3, cell B).

Paper 4 asks: **does that survive contact with a real channel?** Our own research *predicts*
real hardware will reproduce the ceiling, so a high phantom rate is **not** a null result — the
discriminating measurements are the **differences**: MUSIC vs CFAR, and monostatic vs bistatic.

---

## Status ledger (2026-07-17)

| stage | state |
|---|---|
| **Rung 0** — offline pipeline on synthetic CSI | ✅ **done.** `src/wifi_radar_slam/hw/`, 10 tests. Caught two foundational bugs (LOS-align, mid-spectrum zero-pad) before any hardware. |
| **CSI parser + firmware** | ✅ **done.** `hw/csi.py` (8 tests); ESP-IDF C firmware `firmware/csi_{tx,rx}` builds clean on v5.3.2. |
| **Hardware bring-up** | ✅ **FIRST LIGHT.** Real HT40 CSI captured, parsed, delay-profiled. Six on-silicon bugs found + fixed (see below). |
| **Full-rate capture** | ✅ 100 records/s at 460800 baud. |
| **One-command field runner** | ✅ `experiments/run_bench.py {check,rung1}`. |
| **Field setup sheet** | ✅ `docs/paper4-experiment-setup.{html,pdf,png}` (+ dark). |
| **Rung 0.5** — single-tap / ordering check | ✅ on the bench (flat LOS channel, LOS aligns to 0). A/B order to be settled with a real reflector. |
| **Rung 1** — atomic test (echo at a tape-measured plate) | ⏳ **BLOCKED ON A SITE.** Needs a straight ≥15 m corridor (home has none). |
| **Rungs 2–6** (ranging, resolution, phantom rate, MUSIC-vs-CFAR, mono-vs-bistatic) | ⏳ after Rung 1. |
| **Manuscript** | ⏳ not started. |

## First light — verified numbers

Capture `data/hw_captures/first_light_los_2026-07-17.bin`, 5 s, RSSI −29 dBm:
- **140 valid HT40 records** parsed; `cwb=1` (40 MHz), `sig_mode=1` (HT), 384-byte buffers.
- HT-LTF = **128 subcarriers, exactly 114 nonzero** — matches the config active mask (3 DC-notch
  + 11 guard dead), validating the subcarrier ordering (order A).
- Coherent average → **LOS peak at 0.00 m** (LOS-alignment works on real data).
- Near-flat channel = a single clean LOS tap — the correct Rung-0.5 result for adjacent boards.

## Six bugs fixed ON SILICON (compiling clean was not enough)

1. native USB port → download-mode lockup → **use the COM/FTDI port** (`docs/esp32-s3-usb-vs-com-port.md`).
2. 0 bytes on COM (streamed over native USB) → **route CSI to UART0**.
3. abort at `set_channel` → ch1 secondary must be **ABOVE**, not BELOW.
4. abort at internal `set_fix_rate` → use public `esp_wifi_config_80211_tx_rate`, non-fatal.
5. RX got packets but **0 CSI** (legacy frames, no HT-LTF) → **force an HT MCS rate** on the TX.
6. CSI fired but `len=256≠384` → **`lltf_en=true`** (full buffer the parser expects; merge stays false).

---

## Hardware inventory

| board (MAC) | role | firmware | COM tty |
|---|---|---|---|
| `28:84:85:48:40:20` | TX illuminator | `csi_tx` — HT40 MCS7, 100 Hz | `/dev/ttyUSB1` |
| `28:84:85:53:99:DC` | RX CSI logger | `csi_rx` — HT40 CSI → binary UART0 @ 460800 | `/dev/ttyUSB0` |

Both genuine ESP32-S3 (QFN56 rev v0.2, 8 MB PSRAM), on FTDI FT232R COM ports. Toolchain:
ESP-IDF **v5.3.2** at `~/esp/esp-idf`.

## How to run (field)

```bash
.venv/bin/python experiments/run_bench.py check           # Rung 0.5, anywhere
.venv/bin/python experiments/run_bench.py rung1 --plate 12 # Rung 1, in a >=15 m corridor
```
Captures auto-save to `data/hw_captures/`; re-analyse offline with `experiments/run_hw_phantom.py`.

## Key documents

| file | what |
|---|---|
| `docs/paper4-restart-static-bench.md` (+ `.ar.md`) | **the design/spec** — physics, the ladder, kill criteria, why-not-SLAM |
| `docs/research-paper4-hardware.md` | durable research reference, [V]/[P]/[U] tagged |
| `docs/results-paper4-rung0.md` | rung-0 offline results + the two pre-hardware bugs |
| `docs/results-paper4-first-light.md` | the hardware bring-up + first light |
| `docs/esp32-s3-usb-vs-com-port.md` | why the COM port, not native USB (in depth + baby language) |
| `docs/paper4-architecture-python-vs-c.md` | why C for acquisition, Python for the science |
| `docs/paper4-experiment-setup.{pdf,png}` (+ `.dark`) | the field setup sheet (rendered locally) |
| `docs/render-setup-sheet.sh` | regenerate the setup sheet PDFs/PNGs locally (no artifacts) |

## The scientific line (do not overstate)

**WILL claim:** first delay-resolved reflector measurement from ESP32 CSI; the phantom rate on
real 40 MHz silicon (same definition as papers 2/3, so comparable); MUSIC vs CFAR; monostatic vs
bistatic; the instrument's own phantom contribution divided out (IEEE Std 1502 background sub.).
**WILL NOT claim:** a SLAM system, a map, ego-motion, or that WiFi replaces LiDAR/radar. A high
phantom rate confirms the theory — it is not tuned away.

## Do-not-mix reminders

- Papers 1/2/3 are frozen or on their own branches; do not alter their content when evolving
  shared code for paper 4.
- **Paper 2 must not be submitted** until paper 1's erratum resolves.
- The repo is **public and linked from the papers** — record facts and actions, never private
  deliberation.
- Keep paper-4 Claude-memory notes in `paper4-*` files.
