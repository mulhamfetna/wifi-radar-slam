# Paper 2 — *Can Ambient WiFi Replace LiDAR for Automotive SLAM?*

**Subtitle:** Localization Yes, Mapping No — and Why
**Target venue:** IEEE Internet of Things Journal (IoT-J)
**Author:** Mulham Fetna (ORCID 0009-0006-4432-798X)
**Status:** draft, builds clean (7 pages, 0 undefined references/citations)

## Build

```bash
pdflatex main && bibtex main && pdflatex main && pdflatex main
```

`IEEEtran.cls` / `IEEEtran.bst` are **vendored** here and there is **no siunitx
dependency** (unit macros are defined inline in the preamble), so the paper builds
anywhere — including Overleaf — with no package installation.

## Regenerating the figures

Figures 2–6 are **generated from the committed result files**; no number in them is
hand-typed. Fig. 1 is TikZ inside `main.tex`.

```bash
python experiments/make_paper2_figures.py      # from the repository root
```

This writes `docs/assets/paper2_fig{2..6}.pdf`, which `main.tex` picks up via
`\graphicspath{{../../docs/assets/}}`. A test (`tests/test_paper2_figures.py`) asserts
the figure inputs equal the contents of the JSON artifacts, so a figure cannot silently
drift from the data.

## Data provenance — every quantitative claim traces to an artifact

| Paper section | Claim | Source artifact |
|---|---|---|
| §IV LiDAR baselines | Models A/B envelope | `data/lidar_geo_results.json`, `data/lidar_sionna_results.json` |
| §IV KITTI anchor | RPE 0.154 m, ATE 1.16 m / 394 m (~0.3 % drift) | `data/kitti_results.json` |
| §V Accuracy (Table I, Fig. 2) | WiFi vs LiDAR, six metrics, both scenes | `data/wifi_results.json` (frozen paper-1 WiFi), `data/lidar_*_results.json` |
| §VI Cost (Figs. 3–4) | Price envelope, 84–600×, cost-normalized value | `data/cost_data.yaml` (sourced prices + dates) → `data/cost_results.json` |
| §VII Fusion (Table III, Fig. 5) | Tight/loose vs solo; the parity condition | `data/fusion_results.json` |
| §VIII Ceiling (Table IV, Fig. 6) | 89 % phantoms, 6.45 m bias, ladder failure | `data/mapping_floor_isolation.json`, `data/enhanced_map_results.json`, `data/map_filter_f1.json` |

Reproduction scripts: `experiments/{run_lidar_geo,run_lidar_sionna,run_lidar_kitti,run_cost_model,run_fusion,train_map_filter,run_enhanced_map,isolate_mapping_floor}.py`.

## Relationship to paper 1

Paper 1 (*Ambient WiFi as a Radar Replacement for Automotive SLAM*, `../1-wifi-radar-slam/`,
submitted to IoT-J, frozen at tag `v0.7.1`) is cited as prior work and its WiFi results are
**reused, not re-derived**.

§VIII **refines one of its interpretations**: paper 1 attributed the realistic-mapping floor
to *path discrimination* and argued the discrimination was learnable (F1 ≈ 0.9). The
isolation experiment here shows discrimination is the **smallest** of three mechanisms
(phantom detections ≈ 89 % and a 6.45 m range bias dominate), and that its reported F1 used
`elevation` — a feature a single-ULA 2-D delay–azimuth front-end **cannot measure**.

Paper 1's **empirical** results stand (the oracle map; the 60 GHz + 16-antenna null result).
The correction is to be folded into **paper 1's revision** when IoT-J reviews arrive; the
frozen submission is **not** edited.

## Bibliography

Author lists and venues were **verified against primary sources** (arXiv abstract pages, the
RA-L PDF) rather than reconstructed from memory. Note: P2SLAM is IEEE **RA-L**, not T-RO.

Prices cited in §VI are dated market estimates; each carries its source and a `verified` flag
in `data/cost_data.yaml`. Entries marked `verified: false` are drawn from press/market
reporting and **should be re-checked against primary vendor pages before submission**.
