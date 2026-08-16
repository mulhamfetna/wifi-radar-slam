# WiFi-Radar-for-SLAM

[![tests](https://github.com/mulhamfetna/wifi-radar-slam/actions/workflows/tests.yml/badge.svg)](https://github.com/mulhamfetna/wifi-radar-slam/actions/workflows/tests.yml)
[![DOI](https://zenodo.org/badge/1292636094.svg)](https://zenodo.org/badge/latestdoi/1292636094)
[![License: AGPL v3](https://img.shields.io/badge/License-AGPL_v3-blue.svg)](LICENSE)

**Can ambient WiFi stand in for radar/LiDAR in automotive SLAM? Localization yes — mapping hits a
phantom ceiling — and here is exactly what that ceiling is, in simulation across three sensors and
on $30 of real ESP32 hardware.**

This repository is being consolidated from four component studies into **one substantial journal
paper**. It carries a single shared codebase (`src/`, `experiments/`, `firmware/`, `tests/`) and the
manuscript that ties the results together.

---

## The result, in one arc

1. **Feasibility (simulation).** Physics-based, ray-traced (NVIDIA **Sionna RT**): ambient sub-7 GHz
   WiFi on a moving vehicle as a radar-replacement SLAM front-end. **Localization is centimetre-level**
   (ATE 0.098 ± 0.028 m). **Mapping is two-tier** — good with oracle sensing, poor (~4–5 m) with
   realistic commodity CSI.
2. **Mechanism + LiDAR comparison.** The mapping ceiling is **≈89 % phantom detections + a 6.45 m
   range bias** — *not* path-discrimination failure. A learned filter cannot invent the real paths
   or correct the bias. Anchored against real KITTI LiDAR (1.16 m ATE over 394 m).
3. **Radar comparison (universality).** The same substrate vs a 77 GHz FMCW radar shows **the carrier
   does nothing; geometry is everything** — monostatic 0.1 % vs bistatic 18.2 % phantoms — and *more*
   bandwidth makes phantoms *worse*.
4. **$30 hardware validation.** Two ESP32-S3 boards; **first light** — real HT40 (40 MHz) CSI captured,
   parsed, and turned into a delay profile on real silicon, validated against a tape measure.

**Thesis:** WiFi *localization* rivals LiDAR/radar at 1–2 orders lower cost; WiFi *mapping* is capped by
a **front-end + geometry phantom ceiling** that is neither carrier-specific nor WiFi-specific.

> **Resolution note (corrected):** commodity WiFi CSI measures a **bistatic path length**, whose Rayleigh
> resolution is **c/B** (= 1.87 m at 160 MHz), not the monostatic c/2B. This corrects an earlier draft.

## Repository map

| Path | Contents |
|------|----------|
| **`manuscript/`** | the **consolidated journal paper** (assembling) + its `DOSSIER.md` |
| `papers/1..4/` | the four **component drafts** (lineage) — retained, not for standalone submission |
| `src/wifi_radar_slam/` | shared pipeline: `channel`, `radar`, `lidar`, `hw` (ESP32 CSI), `slam`, `eval`, … |
| `firmware/` | ESP-IDF C firmware for the ESP32-S3 hardware testbed (`csi_tx`, `csi_rx`) |
| `experiments/` | run scripts (sim + hardware: `run_bench.py`, `run_hw_phantom.py`, …) |
| `configs/` | sensor/scene configs |
| `docs/` | design specs, results, research references, the field setup sheet |
| `literature/` | the verified, adversarially fact-checked survey |
| `tests/` | `pytest` suite (pure-NumPy; Sionna tests skip when it's absent) |

## Reproduce

```bash
pip install -e '.[dev]'          # core + tests   (add ,sim for Sionna, ,ml for the discriminator)
pytest -q                        # 199 pass, 4 skip (the Sionna-dependent tests)
```

Hardware bench (needs two ESP32-S3 boards on their COM/FTDI ports):
```bash
.venv/bin/python experiments/run_bench.py check            # confirm the CSI link
.venv/bin/python experiments/run_bench.py rung1 --plate 12 # the atomic echo test (needs a >=15 m corridor)
```
Field setup drawings: `docs/paper4-experiment-setup.pdf` (light) / `.dark.pdf`.

## Status & history

Active on branch `consolidation` (assembling the single paper). The four component papers were
developed and frozen separately — their exact submission states live in immutable tags/branches
(`v0.7.1` / `paper1-submitted`, `paper2-v1.0.0` / `paper2-held`, `paper3-v*`, `paper4-hardware-testbed`)
and are never modified. Progress and the reviewer-response ledger: `manuscript/DOSSIER.md`.

## Cite / license

Author: **Mulham Fetna** ([ORCID 0009-0006-4432-798X](https://orcid.org/0009-0006-4432-798X)).
See [`CITATION.cff`](CITATION.cff). Licensed **AGPL-3.0-or-later** ([`LICENSE`](LICENSE)).
