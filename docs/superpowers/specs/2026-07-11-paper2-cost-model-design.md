# Paper 2, sub-project 2 — Cost model (RQ5)

**Date:** 2026-07-11
**Status:** approved (brainstorming), pending execution
**Paper:** 2 — *WiFi sensing as a drop-in LiDAR replacement for SLAM*
(`papers/2-wifi-vs-lidar/DOSSIER.md`)
**Integration branch:** `paper2-wifi-vs-lidar`

## Scope

Quantify paper 2's central value proposition (**RQ5**): the **hardware cost gap** between
a commodity-WiFi sensing package and automotive LiDAR, and — by overlaying the RQ3
accuracy table — **cost-normalized performance** ($ per unit of localization / mapping
quality). The deliverable is a reproducible cost model (sourced price data + computation +
tables) that the paper's cost section and the RQ3 comparison read together.

**NOT** in scope: re-running any WiFi/LiDAR *experiment* (the model consumes the existing
RQ3 numbers in `data/lidar_*_results.json` and `docs/results-paper2.md`); full total-cost-of-
ownership / lifecycle modelling (a noted extension); fusion (RQ4) or DL (RQ2).

## Decisions (from brainstorming)

- **LiDAR denominator = an envelope**, not a single unit: legacy high-end spinning →
  mid solid-state → cheap solid-state → budget 2D scanner. Report the cost ratio across
  the whole range so the result is reviewer-proof (shows exactly where the WiFi gap is
  enormous vs where it narrows).
- **AP accounting = ambient-free headline + deployed sensitivity.** Headline counts only
  the **vehicle-side WiFi receiver** (CSI NIC + antennas); ambient APs are treated as
  pre-existing infrastructure (the passive-radar premise). A **sensitivity row** adds
  *N* self-deployed APs so the deploy-your-own case is also shown.

## Sourced price data (deep-research 2026-07-11; see `docs/literature-paper2.md`)

Prices are **estimates with ranges, dates, and sources** — not vote-verified in the
research pass, so each entry in the data file carries a primary-source URL + date and the
model reports ranges (low/high), never a single false-precision number.

**Automotive/robotics LiDAR envelope**
| Tier | Unit | Price (USD) | Note |
|------|------|-------------|------|
| Legacy high-end spinning | Velodyne HDL-64 class | ~$75–80 k | historical; the grade that delivers full 3D mapping |
| Mid solid-state | Luminar Halo / series solid-state | ~$500–600 | 2026 automotive-grade |
| Cheap solid-state | MicroVision Movia S / Hesai | ~$200 (long-term $100 goal) | volume ADAS target |
| Budget 2D scanner | Slamtec RPLIDAR A1 | ~$99 | **2D/indoor-grade — not automotive**; flag as a floor, not a fair peer |

**Commodity WiFi-CSI receiver (vehicle side)**
| Item | Unit | Price (USD) |
|------|------|-------------|
| CSI receiver (budget) | ESP32 (Espressif IDF CSI) | ~$5–15 |
| CSI receiver (headline) | Raspberry Pi 4 + nexmon_csi | ~$35–75 |
| CSI receiver (legacy) | Intel 5300 NIC (iwl5300 tool) | commodity, ~$20–40 used |
| Antennas | 2–3× 2.4/5 GHz | ~$5–20 total |

## Architecture

Cost is a **data + computation** task; keep it small and pure so it tests locally.

```
data/cost_data.yaml                 # sourced prices: WiFi BOM items + LiDAR tiers,
                                    #   each {low, high, source_url, date, note}
src/wifi_radar_slam/cost.py         # pure functions: load, aggregate, ratio, normalize
experiments/run_cost_model.py       # produce cost tables + cost-normalized overlay
tests/test_cost.py                  # pure-function unit tests (local)
```

`cost.py` exposes:
- `load_cost_data(path) -> dict` — parse the YAML (coerce numeric ranges).
- `wifi_package_cost(data, rx="pi_nexmon", n_aps=0) -> (low, high)` — the **headline
  vehicle-side package** = one CSI receiver (`rx`, default the representative
  Raspberry Pi 4 + nexmon_csi) **+ antennas**; `rx="esp32"` gives the low-end variant.
  When `n_aps>0`, add `n_aps × ap_unit` (deployed-AP sensitivity). Compute/host is
  assumed to be the vehicle's existing SoC → $0 marginal (stated as an assumption).
- `lidar_envelope(data) -> list[tier]` — the LiDAR tiers with (low, high).
- `cost_ratio(wifi_range, lidar_range) -> (low, high)` — how many × cheaper WiFi is
  (report as a range from the price ranges).
- `cost_normalized(price_range, metric_value, mode) -> value` — overlay cost on an RQ3
  metric: `mode="accuracy"` → cost×ATE ($·m, lower=better, rewards cheap **and** accurate);
  `mode="coverage"` → cost÷IoU ($ per IoU point, lower=better). Returns ranges.

`run_cost_model.py` reads `data/cost_data.yaml` + the RQ3 results
(`data/lidar_geo_results.json`, `data/lidar_sionna_results.json`, and the WiFi numbers
from `docs/results-paper2.md`, kept as a small literal in the script or a companion JSON),
and emits:
1. **Absolute cost table** — WiFi package (ambient-free) vs each LiDAR tier.
2. **Cost-ratio table** — × cheaper per tier (headline + deployed-AP sensitivity).
3. **Cost-normalized performance** — $·ATE (localization value) and $/IoU (mapping value)
   for WiFi vs LiDAR-A/B on both scenes, so the price gap is read *with* the accuracy gap.
Output appended to `docs/results-paper2.md` (a new "Cost (RQ5)" section) + `data/cost_results.json`.

## Data flow

```
data/cost_data.yaml ──┐
                      ├─► cost.py ──► absolute $ table, cost ratios (× cheaper)
RQ3 metrics (json) ───┘        └────► cost-normalized: $·ATE (localization value),
                                        $/IoU (mapping value)  — WiFi vs LiDAR A/B
                                              │
                                              ▼
                       docs/results-paper2.md "Cost (RQ5)" + data/cost_results.json
```

## Honesty guards

- **Cost is never reported alone.** The cost-normalized overlay forces the price gap to be
  read next to the RQ3 accuracy/coverage gap — WiFi wins localization value decisively but
  LiDAR still wins mapping-coverage value at the tiers that map well.
- **Distinguish automotive-grade from budget 2D.** RPLIDAR A1 (~$99) is a 2D indoor scanner,
  not a peer to the 3D automotive LiDAR our A/B models represent — mark it as a price floor,
  not a fair comparison, so we don't strawman.
- **Ranges + sources + dates, no false precision.** Every price is a range with a cited
  primary source and date in `cost_data.yaml`; the model propagates ranges to outputs.
- **Volume vs retail** noted (e.g. LiDAR $100 long-term *target* vs current unit price).
- **Optional secondary axis (power/energy)** is a noted extension, not core — the headline
  the paper makes is hardware BOM price.

## Testing

Pure-Python unit tests (`tests/test_cost.py`), no server:
- `wifi_package_cost`: sums items; `n_aps` adds AP cost; returns (low,high).
- `cost_ratio`: correct × range from two price ranges (e.g. WiFi $50 vs LiDAR $500 → 10×).
- `cost_normalized`: `accuracy` mode = price×ATE, `coverage` mode = price÷IoU, on known inputs.
- `load_cost_data`: parses a small fixture YAML incl. ranges + sources.
Full existing suite must still pass.

## Non-goals

- No new experiments; no fusion/DL; no full TCO/lifecycle; no change to paper-1 content.
- Not a market forecast — a snapshot with dated sources and a sensitivity range.

## Acceptance

- `data/cost_data.yaml` exists with WiFi BOM + LiDAR tiers, each carrying `low`/`high`,
  `source_url`, `date`, `note`.
- `cost.py` pure functions pass local unit tests; `run_cost_model.py` produces the three
  tables + `data/cost_results.json`.
- `docs/results-paper2.md` gains a "Cost (RQ5)" section: absolute $ gap, ratio envelope
  (headline + deployed-AP sensitivity), and the cost-normalized localization/mapping-value
  overlay tied to the RQ3 numbers.
- Full test suite green.
