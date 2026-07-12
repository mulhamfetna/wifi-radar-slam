# Paper 3 — design: WiFi vs automotive radar for SLAM

**Date:** 2026-07-12
**Status:** approved (design + spec review, 2026-07-12)
**Branch:** `paper3-wifi-vs-radar` (off `main`)
**Working title:** *Is the Phantom Ceiling Universal? Ambient WiFi vs 77 GHz Automotive Radar
for Vehicular SLAM*

## Scope

Run the same comparison substrate as paper 2 — same scenes, same footprint ground truth, same
back-end, same metrics — but against a **77 GHz FMCW automotive radar** instead of LiDAR. The
paper has two pillars, decided in brainstorming:

- **Headline (RQ1):** is the **≈89 % phantom-detection ceiling** paper 2 measured a *WiFi*
  pathology, or a property of **RF sensing** generally? Radar — 25× the bandwidth, monostatic,
  its own transmitter — is the perfect control.
- **Support (RQ2):** a **2×2 ablation** that decomposes radar's advantage into **bandwidth vs
  geometry vs carrier**. Only simulation can separate these, and separating them is a
  contribution in its own right. It also de-risks the paper: if the ghost result is dull, the
  ablation still carries it.

## Research questions

| | Question |
|---|---|
| **RQ1** | Is the phantom ceiling universal to RF sensing, or WiFi-specific? Measure the phantom rate of *radar CFAR detections* with paper 2's isolation experiment. |
| **RQ2** | Where does radar's advantage come from — bandwidth, monostatic geometry, or carrier? |
| **RQ3** | Head-to-head SLAM accuracy: six metrics **plus KITTI-style drift %** (the accepted radar protocol). |
| **RQ4** | Cost — honestly, and with an explicit statement of what cannot be sourced. |

**Explicitly dropped (YAGNI):** WiFi+radar fusion. Paper 2 already did fusion; the two sensors
are physically similar, so the expected gain is small, and it distracts from the two pillars.
Named as future work.

## 🔑 The load-bearing methodological decision

The research pass (`docs/literature-paper3.md`) established that "extract paths" vs "synthesise
the beat signal" is a **false dichotomy** — ray tracers emit paths, and the beat signal is
built *from* them. Critically:

> *"skipping the beat stage gives you ideal, infinitely-resolved delays/angles … CFAR
> behaviour and the resulting **false-alarm/ghost statistics** must all be added downstream."*

**Therefore the radar sensor MUST implement the full chain**

```
Sionna paths ──► FMCW beat signal ──► windowed range FFT ──► azimuth beamforming ──► CFAR ──► detections
```

If we extracted radar paths directly (which is what our WiFi *oracle* does), **radar would
produce zero ghosts by construction** and RQ1 — the paper's headline — would be rigged in
radar's favour and scientifically worthless. This is non-negotiable.

**Range–azimuth, not range–Doppler (and why).** Our scenes are **static** (buildings, parked
cars), so the Doppler axis would carry little but ego-motion. More decisively, the benchmark we
anchor the back-end on (Oxford Radar RobotCar / Boreas) uses **spinning radar producing
range–azimuth scans** — so a range–azimuth chain is both simpler *and* directly comparable to
the anchor. Doppler processing is an optional extension, not part of the core (it would matter
for moving targets, which these scenes do not contain). This is a stated scope choice, not an
oversight.

**The geometry asymmetry is the point, and must be explicit.** A detection maps to a world
point differently in the two geometries:
- **Bistatic (cells A, and the MUSIC reference):** the measured range is a *bistatic path
  length* (AP→reflector→vehicle); the reflector is recovered by the **ellipse solve**, which is
  ill-conditioned in exactly the ways papers 1–2 documented.
- **Monostatic (cells B, C, D):** the measured range is a direct round-trip; the reflector is a
  simple **polar → Cartesian** projection.

That difference *is* the geometry ablation (A→B). It must be reported as a mechanism, not
hidden inside a helper function.

## The ablation (RQ2), and why the detection chain must be held fixed

Radar differs from our WiFi on three axes **simultaneously**. To decompose them, we vary one at
a time and **hold the detection algorithm constant** (FFT + CFAR for every cell), so any
difference is *physical*, not algorithmic:

| Cell | Carrier | Bandwidth | Geometry | Transmitter | Isolates |
|------|---------|-----------|----------|-------------|----------|
| **A** WiFi baseline | 5.2 GHz | 160 MHz | bistatic | ambient (free) | — |
| **B** WiFi monostatic | 5.2 GHz | 160 MHz | **monostatic** | own | **geometry** (A→B) |
| **C** Radar narrowband | **77 GHz** | 160 MHz | monostatic | own | **carrier** (B→C) |
| **D** Radar full | 77 GHz | **4 GHz** | monostatic | own | **bandwidth** (C→D) |

Cell **B** is an *active WiFi radar* — physically meaningful (it is what a WiFi ISAC device
would be) and it isolates the bistatic-ellipse penalty. Cell **C** is a bandwidth-crippled
radar, which isolates what the carrier alone buys.

**Plus a 5th reference row: WiFi + joint 2-D MUSIC** (papers 1–2's front-end), so the paper
connects to the prior work and so the *superresolution vs FFT+CFAR* axis is visible rather than
silently confounded with the physics.

Every cell reports the six metrics, drift %, **and its phantom rate** — which is what makes RQ1
answerable *as a function of* bandwidth and geometry, not merely as a single number.

## Credibility: the radar baseline must not be a strawman

Radar SLAM is mature — **CFEAR 1.09 %** drift (Oxford), **DRO 0.26 %** (Boreas leaderboard). A
naïve baseline would be dismantled in review. Two defences, both required:

1. **One back-end for every sensor.** WiFi, radar (and LiDAR, inherited) all go through the
   same scan-to-map ICP, so differences are attributable to the **sensor**, not the estimator.
   This is exactly what made paper 2's comparison valid.
2. **Anchor that back-end on real radar data + cite the SOTA row.** Run our back-end on a real
   radar benchmark (Oxford Radar RobotCar or Boreas) and report **drift %** beside **CFEAR
   (1.09 %) and DRO (0.26 %)**, cited from the literature — the same move that made paper 2's
   LiDAR credible via KITTI (0.3 % drift). If our back-end lands in a plausible band, the
   baseline is credible. **If it does not, we report that and bound our claims accordingly.**

The SOTA row is **cited, not reimplemented** — provided we run our back-end on the *same*
benchmark, the numbers are directly comparable, which is cheap and rigorous.

## Architecture

New shared package `src/wifi_radar_slam/radar/` (additive; reuses paper-2 machinery):

```
radar/
  config.py       RadarConfig: carrier, bandwidth, n_chirps, MIMO tx/rx, chirp time,
                  max range, CFAR guard/training cells, Pfa. Presets: RADAR_77G_4G, RADAR_77G_160M
  processing.py   PURE NumPy signal chain (locally testable, no Sionna):
                    beat_cube(paths, cfg)  -> (n_rx, n_chirps, n_samples) complex
                    range_doppler(cube)    -> windowed 2-D FFT
                    cfar_2d(rd, cfg)       -> detection mask (CA-CFAR)
                    detections_to_scan(...)-> Scan in the sensor-local BEV frame
  sensor.py       SionnaRadarSensor: monostatic node + diffuse scattering (the paper-2
                  SionnaLidarSensor pattern) -> paths -> processing -> Scan.
                  make_sensor seam: radar_sensor(built, cfg, rng) -> (pose -> Scan)
eval/drift.py     KITTI-style drift %: translational error over 100-800 m sub-trajectories,
                  rotational error deg/100 m. REQUIRED -- it is the accepted radar protocol,
                  and ATE alone would be marked down.
```

Reused unchanged: `lidar/slam_icp.py` (the shared scan-to-map back-end), `lidar/pointcloud.Scan`,
`eval/metrics.py` (the six metrics), `map_filter.py`, and the isolation experiment
(`isolate_mapping_floor.py`), generalised to take a detection source.

## Data flow

```
scene (Sionna RT) ──► monostatic node + diffuse scattering
                          │  (specular-only returns NOTHING at 77 GHz -- see pitfalls)
                          ▼
                    paths (tau, a, AoA, doppler)
                          │
                          ▼
        beat signal ──► range-Doppler FFT ──► CFAR ──► detections ──► Scan
                                                          │
                          ┌───────────────────────────────┴──────────────┐
                          ▼                                              ▼
              scan-to-map ICP (SHARED back-end)              isolation experiment
                          │                                   (phantom rate, RQ1)
                          ▼
        six metrics + drift %  vs the SAME footprint ground truth
```

## Pitfalls (from the research; each must be handled explicitly)

1. **`normalize_delays=True` is the DEFAULT** on `cfr()`/`cir()`/`taps()` and **zeroes the
   first-path delay, destroying absolute range**. The beat-signal stage uses `cfr()` → **must
   pass `normalize_delays=False`**. *(Papers 1–2 are unaffected: they read `paths.tau`, which
   is absolute — verified 2026-07-12, LOS delay × c == true distance to 3 dp.)*
2. **Specular-only surfaces fail for monostatic 77 GHz radar** — targets become invisible
   (Altair: *"without scattering, some objects might escape detection"*). Use the
   monostatic + `diffuse_reflection=True` + `scattering_coefficient` pattern we already
   validated in paper 2 (specular → 1 return; diffuse → 8,417). Now citable, not a hack.
3. **`doppler` is zero** unless velocity vectors are assigned to devices/objects.
4. **Time evolution is synthetic** — geometry frozen, only phase rotates; no range migration or
   path birth/death within the CPI. NVIDIA warns it is "only accurate over very short time
   spans": **validate the chosen CPI (128–256 chirps) at vehicular speed** rather than assuming.
5. **Monostatic (co-located TX/RX) has an angle-convention change** (NVlabs/sionna-rt #5).
6. **Amplitudes are frequency-flat across the sweep** (solved at the single carrier; ~5 %
   fractional bandwidth at 77 GHz) — **a stated limitation**: no in-band material/antenna/RCS
   dispersion.

**Methodological anchor to cite:** Schüßler et al., *IEEE J. Microwaves* 1(4):962–974, 2021 —
measurement-validated ray-traced automotive MIMO radar in which **multipath ghosts fall out of
the method inherently**. Note their own caveat: the reflection models are "simplistic"
(specular + diffuse lobes, not full-wave RCS).

## Honesty guards

- **Radar is expected to win.** Existing WiFi-vs-radar head-to-heads favour radar decisively
  (97.78 % vs 65.09 % on matched HAR). We design to **explain** the gap, not to close it, and
  we do **not** tune WiFi to manufacture parity.
- **Our radar baseline is not CFEAR-class.** Say so, anchor it on real data, and place the
  cited SOTA row beside it.
- **The cost argument is structurally weak here** and must not be oversold: radar is the same
  order of cost as the WiFi package, so the 84–600× story of paper 2 **does not transfer**.
  Moreover **OEM automotive-radar pricing is not public** (two research passes found none) —
  the paper will source **evaluation-board/retail** prices with dates, cite analyst figures
  explicitly *as estimates*, and **state the limitation plainly**. WiFi's remaining case is
  **zero marginal cost** (hardware already present, transmitter free), not performance.
- **A null RQ1 result is publishable.** If radar's phantom rate is low, the ceiling is
  WiFi-specific — that is a clean, useful finding and the ablation explains why.
- **Frequency-flat amplitudes and the synthetic time evolution are limitations, stated in the
  abstract**, not buried.

## Non-goals

- No WiFi+radar fusion (paper 2 covered fusion; dropped here).
- No 4D/elevation radar — the comparison plane stays **2-D BEV**, as in paper 2, for
  comparability. (4D radar SLAM is itself unsolved: SNAIL reports 0.2 m → 216.7 m ATE.)
- No real radar hardware.
- No change to papers 1–2 content.

## Target venue

**IEEE IoT-J** — decided at spec review (2026-07-12). Continuity with papers 1–2, the same
audience, and the WiFi/ISAC framing fits. Noted trade-off, accepted: a radar-specialist venue
(e.g. IEEE Trans. Radar Systems) would scrutinise the radar baseline harder — which the
credibility anchor + cited SOTA row are designed to survive — but would care less about the
WiFi/IoT framing.

## Decomposition into sub-projects

This is the **paper-level** design. Implementation is decomposed, each with its own plan and
merge (the pattern that worked for paper 2):

| # | Sub-project | Deliverable |
|---|-------------|-------------|
| **1** | **Radar substrate** | `radar/` (RadarConfig, pure-NumPy beat→range→azimuth→CFAR chain, Sionna monostatic sensor on the `make_sensor` seam) + `eval/drift.py` (KITTI-style drift %). Unit-tested; sensor gated. |
| **2** | **Credibility anchor** | Run the shared back-end on a real radar benchmark (Oxford/Boreas); report drift % beside the cited CFEAR (1.09 %) / DRO (0.26 %) rows. Decides whether the baseline is defensible **before** we invest in the ablation. |
| **3** | **Ablation + phantom rates (RQ1, RQ2, RQ3)** | The five cells × two scenes → six metrics, drift %, and phantom rate each. This is the paper. |
| **4** | **Cost (RQ4) + manuscript** | Eval-board/retail pricing sourced directly; manuscript reusing the paper-2 scaffold. |

**Sub-project 2 is a gate.** If our shared back-end cannot reach a plausible drift band on real
radar data, the radar baseline is not credible and the ablation would be built on sand — we stop
and reconsider rather than proceeding.

## Acceptance

- `radar/` package: `RadarConfig`, the pure-NumPy signal chain (beat → RD-FFT → CFAR →
  detections), and a Sionna monostatic sensor on the `make_sensor` seam; pure parts unit-tested
  locally, the Sionna sensor gated.
- `eval/drift.py`: KITTI-style drift %, unit-tested.
- All five ablation cells (A–D + the MUSIC reference) produce the six metrics, drift %, **and a
  phantom rate**, on both scenes.
- The shared back-end is anchored on a real radar benchmark, reported beside the cited
  CFEAR/DRO numbers.
- Every quantitative claim traces to a committed artifact; the full test suite stays green.
