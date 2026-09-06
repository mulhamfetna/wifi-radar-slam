# Paper 4 — Rung 0 results: the delay pipeline on synthetic CSI

**Date:** 2026-07-15
**Branch:** `paper4-hardware-testbed`
**Status:** ✅ **COMPLETE — 10/10 tests pass, no hardware used.**
**Scope:** Rung 0 of the ladder in `docs/paper4-restart-static-bench.md` Part 9.

> **What rung 0 proves:** the offline processing chain recovers a delay tap that *we
> injected ourselves*, through every corruption the real ESP32 is documented to impose —
> **before a single board is bought.** If the code cannot recover its own injected tap, the
> bug is ours and no hardware would have revealed it. Rung 0 passing means the failure mode
> of the real experiment is *physics*, not our software.

---

## 1. What was built

Pure NumPy/SciPy, under `src/wifi_radar_slam/hw/`. No Sionna, no hardware, no network.

| module | responsibility | key functions |
|---|---|---|
| `config.py` | ESP32 HT40 physical constants and derived resolution numbers | `CSIConfig`, `ESP32_HT40` |
| `synth.py` | synthetic CSI with the full corruption stack (the rung-0 ground truth) | `Tap`, `ideal_csi`, `receiver_response`, `synth_packet`, `synth_recording` |
| `delay.py` | the pipeline: CSI → aligned CIR → coherent average → differential | `raw_cir`, `prepare_packet`, `coherent_average`, `delay_profile`, `differential` |
| `detect.py` | 1-D CA-CFAR (same formula as the sim) + dip-based resolution metric | `cfar_1d`, `detect_delays`, `resolved` |

**Test runner:** `.venv/bin/python -m pytest tests/test_hw_rung0.py`

---

## 2. The physical constants (verified, derived)

All from `CSIConfig` / `ESP32_HT40`. Sources carry [V] tags in the design doc Parts 4 & 7.

| quantity | value | note |
|---|---|---|
| subcarriers reported (HT-LTF) | **128** | ESP-IDF `wifi.rst` CSI table |
| subcarrier spacing | **312.5 kHz** | 802.11 OFDM |
| occupied bandwidth `B` | **40 MHz** | 128 × 312.5 kHz |
| active subcarriers | **114** | 128 − 3 notch − 11 guard (|k|≥59: −64..−59 = 6, 59..63 = 5) |
| delay resolution `1/B` | **25 ns** | one native delay cell |
| one cell in **path** length `c/B` | **7.4948 m** | exact `c`; the doc rounds to 7.5 with `c≈3e8` |
| one cell in **monostatic range** `c/2B` | **3.7474 m** | the hard resolution floor |
| excess delay, plate at 12 m, baseline 0.5 m | **78 ns = 3.13 cells** | the recommended Rung-1 start point |

---

## 3. The corruption stack modelled in `synth.py`

Every one is a documented property of real commodity CSI (design doc Part 5 / Part 7). Rung 0
demands the pipeline survive **all of them at once**:

1. **3-bin DC notch + 11 edge guard bins** — zeroed, as the ESP32 effectively delivers.
2. **S-shaped receiver phase distortion + M-shaped amplitude ripple** — the receiver's own
   frequency response, which *manufactures a phantom tap* and is **static per device** (so it
   cancels in the plate-in/plate-out differential). This is the `Hdist`/"causes a phantom
   object" effect.
3. **Per-packet random common phase** (residual CFO + PLL) — subcarrier-independent; rotates
   the whole CIR but moves no tap.
4. **Per-packet random STO** (packet-detection jitter) — a linear phase ramp = a pure time
   shift of the whole CIR.
5. **8-bit I/Q quantisation** (~48 dB raw dynamic range) — why a single packet cannot see a
   weak echo and coherent averaging is mandatory.

---

## 4. The pipeline, and why the order is not optional

```
per packet:    prepare_packet = raw_cir → find LOS peak → divide by it → roll LOS to bin 0
per recording: coherent_average = mean of prepared CIRs
differential:  |averaged(plate_in) − averaged(plate_out)|
detection:     cfar_1d on the power profile → centroid delays (excess delay = position from bin 0)
```

- **Divide by the LOS tap** kills the per-packet random common phase *and* the amplitude
  scaling in one step.
- **Roll the LOS to bin 0** kills the STO time shift and makes every packet mutually
  coherent — only *then* is averaging legitimate.
- **After alignment the LOS sits at delay 0**, so a detected tap position *is* its excess
  delay over LOS — exactly the quantity the excess-delay method needs.
- **The differential** cancels the static receiver response (the phantom generator) and the
  LOS, isolating the plate's echo. Formally a **bistatic RCS measurement with the empty room
  as background** (IEEE Std 1502).

---

## 5. 🐛 Two real bugs caught at rung 0 — before any hardware

This is the entire justification for doing the Python side first.

### Bug 1 — mean-slope de-ramp misaligned the differential
The first implementation removed the STO by estimating the **mean phase slope** across
subcarriers. But a strong echo *pulls* that slope, so `plate_in` (LOS + echo) and `plate_out`
(LOS only) received **different** time shifts — and the differential subtracted two
misaligned profiles, biasing the echo by ~2 m.
**Fix:** align to the **LOS peak** explicitly (divide by it, roll it to bin 0), which is
robust to a strong echo. A mean-slope de-ramp is the wrong tool.

### Bug 2 — 🚨 FOUNDATIONAL: zero-padding at the wrong end of the spectrum
`np.fft.ifft(a, n=2048)` zero-pads by **appending** zeros to the *end* of the array. But
after `ifftshift`, the array is `[f0..f63, f−64..f−1]` — the **negative frequencies are at
the end**. Appending zeros there corrupts them, shifting **every tap by a
distance-dependent amount — up to 4 m.**

| plate | true path | detected (buggy) | detected (fixed) |
|---|---|---|---|
| 8 m | 15.5 m | 15.0 m (−0.5) | 15.46 m (−0.04) |
| 10 m | 19.5 m | **23.4 m (+3.9)** | 19.67 m (+0.17) |
| 12 m | 23.5 m | 22.0 m (−1.5) | 23.42 m (−0.08) |
| 14 m | 27.5 m | **30.9 m (+3.4)** | 27.64 m (+0.14) |

**Loose tolerances had hidden it.** On a $5 chip in a corridor, this bug would have looked
**exactly like a phantom reflection** and cost weeks of chasing physics that wasn't there.
**Fix:** zero-pad **in the middle** (at Nyquist), keeping the negatives at the high end.

> These two are precisely the class of error the whole restart exists to catch: *a layer
> built on an untested layer.* Both were killed by fast Python iteration + `pytest` on
> synthetic data, at zero hardware cost. **This is the argument for the development order.**

---

## 6. Results after the fix

**Ranging (single isolated echo, precision — rung-2 physics):**
- Pure echo, no noise: error **< 0.2 m** across 8–14 m.
- Full stack (noise + 8-bit quant + differential, 500 packets): error **< 0.5 m** — i.e.
  well under one 3.75 m cell.

This confirms the resolution/precision distinction (design doc Part 4.4): a *single* target
interpolates to a small fraction of a bin even at 40 MHz, because precision is SNR-limited,
not bandwidth-limited. That is what makes a range-bias measurement (rung 2) meaningful.

**Coherent averaging:** the random-noise RMS (deviation from the noiseless reference) falls
with N — 100 packets cut it by > 2× vs a single packet, the mechanism behind the +30 dB that
lifts a weak echo above the 8-bit floor.

**Differential cancels the phantom generator:** with a strong (strength 0.6) S-shaped receiver
distortion, the echo still emerges in `plate_in − plate_out` at the true excess path, > 4× above
the floor. The static receiver response cancelled.

**Resolution (two targets — rung-3 physics):** two taps ≥ 3 cells apart show a real dip
between them; taps < 0.2 cell apart merge into one lobe — a falsifiable property of the 40 MHz
bandwidth.

---

## 7. The ten tests (the rung-0 acceptance criteria)

| test | asserts |
|---|---|
| `test_config_derived_numbers` | 40 MHz, 25 ns, 7.49 m/3.75 m cells, 114 active subcarriers |
| `test_excess_delay_geometry` | 12 m plate → 78 ns → 3.13 cells |
| `test_empty_corridor_shows_one_tap` | Rung-0.5 logic: LOS-only shows one dominant tap (ordering check) |
| `test_prepare_packet_kills_common_phase_and_sto` | two packets differing only in phase+STO prepare alike |
| `test_coherent_averaging_suppresses_random_noise` | noise RMS falls with N |
| `test_rung0_recovers_injected_echo_through_full_stack` | **the kill criterion** — injected echo recovered < ½ cell through the full stack |
| `test_ranging_tracks_true_distance` | single echo located < 0.5 m across 8–14 m |
| `test_receiver_response_cancels_in_differential` | strong static distortion cancels; echo survives |
| `test_two_taps_resolve_when_far_merge_when_close` | resolution behaves per bandwidth |
| `test_cfar_1d_constant_false_alarm_on_pure_noise` | CFAR holds its design false-alarm rate |

---

## 8. What rung 0 does NOT prove

- **Nothing about real silicon.** The echo strengths (−30 dB at 6 m, −46 dB at 14 m) are
  *estimates*; rung 1 measures them for real. Whether coherent averaging delivers the full
  +30 dB on the actual chip, and whether the receiver distortion is stable enough between two
  recordings to cancel, are open questions rungs 0.5 and 1 answer.
- **Nothing about a map, a pose, or SLAM.** Those are out of scope by design (see Part 2 of
  the design doc: a self-contained SLAM system is not provable on this hardware).

---

## 9. Next

- **Firmware** (`firmware/`, ESP-IDF C): TX illuminator + RX binary CSI streamer.
- **`hw/csi.py`**: the CSI byte parser, with our *own* index derivation (Espressif's is
  disputed — issue #224) validated by the Rung-0.5 empirical check.
- Then order 2× ESP32-S3 and run Rung 0.5 (the 30-minute ordering check) and Rung 1 (the
  atomic test).

*See `docs/hardware-build.md` for the bill of materials, wiring, firmware config, and the
survey procedure.*
