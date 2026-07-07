# WiFi-Radar-for-SLAM

[![DOI](https://zenodo.org/badge/1292636094.svg)](https://zenodo.org/badge/latestdoi/1292636094)
[![License: AGPL v3](https://img.shields.io/badge/License-AGPL_v3-blue.svg)](LICENSE)

**Ambient WiFi as a radar replacement for automotive SLAM.**

Using existing WiFi signals in the environment, plus an on-vehicle WiFi antenna, to build a 3D scan of
the surroundings that can substitute for radar in Simultaneous Localization and Mapping (SLAM) pipelines.

> **Status:** research project, literature phase complete; first build is a **simulation-first
> feasibility study** for the sub-7 GHz WiFi path. A 60 GHz / IEEE 802.11ad mmWave extension is planned
> as future work.

## Why this is novel

Verified across a two-round, adversarially fact-checked literature survey (see [`literature/`](literature/)):
prior WiFi/CSI sensing is overwhelmingly **indoor, static, and infrastructure-fixed**; the only
demonstrated moving-vehicle passive radar uses **5G**, not WiFi. The defensible contribution is an
**on-vehicle, mobile, outdoor WiFi (802.11) passive-radar / CSI system producing 3D scans for SLAM** —
an unoccupied point in the design space.

A key verified constraint shapes the roadmap: sub-7 GHz WiFi is bandwidth-limited (ΔR = c/2B ≈ 3.75 m at
40 MHz, 0.94 m at 160 MHz), so radar-grade resolution (~8.5 cm) ultimately needs 60 GHz mmWave. We start
with the achievable sub-7 GHz path and keep mmWave for future development.

## This milestone (v1): feasibility simulation

Physics-based, ray-traced (NVIDIA **Sionna RT**) simulation of a vehicle driving through an outdoor
parking-lot / campus scene, receiving ambient WiFi, running a full **CSI → scene → SLAM** pipeline, and
evaluated against ground truth. See the design spec:
[`docs/superpowers/specs/2026-07-04-wifi-radar-slam-feasibility-sim-design.md`](docs/superpowers/specs/2026-07-04-wifi-radar-slam-feasibility-sim-design.md).

Two phases:
- **Phase A — nominal:** reconstruct map + trajectory; report map error (Chamfer/IoU) and trajectory
  error (ATE/RPE).
- **Phase B — operating envelope:** sweep AP density, SNR, vehicle speed, and WiFi bandwidth
  (20 → 160 MHz) to show where WiFi-SLAM works and breaks.

## Repository

| Path | Contents |
|------|----------|
| `literature/` | Verified survey, detailed report, HTML report, `references.bib` |
| `outreach/` | Personalized author-contact email drafts |
| `docs/` | Design specs and implementation plans |
| `src/` | Simulation pipeline (added during implementation) |

## Citing

Archived on Zenodo — **concept DOI [10.5281/zenodo.21247288](https://doi.org/10.5281/zenodo.21247288)**
(always resolves to the latest version). Licensed **AGPL-3.0-or-later** (see [`LICENSE`](LICENSE));
citation metadata in [`CITATION.cff`](CITATION.cff).

```bibtex
@software{fetna_wifi_radar_slam,
  author  = {Fetna, Mulham},
  title   = {{WiFi-Radar-for-SLAM: Ambient WiFi as a Radar Replacement for Automotive SLAM}},
  year    = {2026},
  publisher = {Zenodo},
  doi     = {10.5281/zenodo.21247288},
  url     = {https://github.com/mulhamfetna/wifi-radar-slam}
}
```

**Author:** Mulham Fetna — [ORCID 0009-0006-4432-798X](https://orcid.org/0009-0006-4432-798X) ·
contact@mulhamfetna.com · [GitHub](https://github.com/mulhamfetna)
