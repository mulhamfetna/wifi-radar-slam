# Paper 2 Cost Model (RQ5) Implementation Plan

> **For agentic workers:** REQUIRED SUB-SKILL: Use superpowers:subagent-driven-development (recommended) or superpowers:executing-plans to implement this plan task-by-task. Steps use checkbox (`- [ ]`) syntax for tracking.

**Goal:** Quantify the WiFi-package-vs-LiDAR hardware cost gap (RQ5) and overlay it on the RQ3 accuracy numbers, producing an absolute-cost table, a cost-ratio envelope, and cost-normalized localization/mapping-value tables.

**Architecture:** A sourced price data file (`data/cost_data.yaml`) plus a small pure module (`src/wifi_radar_slam/cost.py`) with four functions (load, package cost, envelope/ratio, cost-normalize). A report script (`experiments/run_cost_model.py`) joins those prices with the existing RQ3 metrics (`data/lidar_geo_results.json`, `data/lidar_sionna_results.json`, and a new `data/wifi_results.json` holding the frozen paper-1 WiFi numbers) and emits the tables. No new experiments, no server, no compute.

**Tech Stack:** Python 3, PyYAML (already a core dep — `yaml.safe_load`, same idiom as `config.py`). NumPy not required. `pytest` for pure unit tests.

## Global Constraints

- **Branch:** all work on `paper2-cost-model`, cut from `paper2-wifi-vs-lidar`; merge back on completion. Never commit to `main` or any `paper1-*` ref.
- **No new experiments** — the model *consumes* existing RQ3 results; it never re-runs WiFi or LiDAR sims. Runs entirely locally (no Sionna, no amd server).
- **Ranges, never false precision.** Every price is a `low`/`high` pair with a `source` citation and `date`; all outputs propagate `(low, high)` ranges.
- **Honesty guards (from the spec):** the budget 2D scanner (RPLIDAR A1, ~$99) is `automotive_grade: false` and must be labelled a **price floor, not a fair automotive peer**; volume-vs-retail noted; cost is never reported without the accuracy overlay.
- **No fabricated URLs.** Each price carries a `source` **text citation** (publisher/title/date). `source_url` appears **only** where the canonical vendor/project page is certain (Espressif, nexmon_csi repo, Slamtec). News-derived prices carry `verified: false` to flag a primary-source check before submission.
- **LiDAR denominator = envelope** (legacy spinning → mid solid-state → cheap solid-state → budget 2D). **AP accounting = ambient-free headline** (vehicle RX + antennas only) **+ deployed-AP sensitivity** (`n_aps > 0`).
- **WiFi headline package** = one CSI receiver (default `pi_nexmon`) + antennas. Vehicle compute assumed $0 marginal (stated assumption).
- **Divide-by-zero is a real result:** WiFi-realistic IoU ≈ 0, so `$/IoU` → `inf`, meaning *no mapping value at any price*. Must be handled explicitly, not crash.

---

### Task 1: Sourced price data + `load_cost_data`

**Files:**
- Create: `data/cost_data.yaml`
- Create: `src/wifi_radar_slam/cost.py`
- Test: `tests/test_cost.py`

**Interfaces:**
- Consumes: nothing.
- Produces: `load_cost_data(path) -> dict` (parsed YAML). Data schema used by Tasks 2–4:
  `data["wifi"]["receivers"][key] -> {low, high, ...}`, `data["wifi"]["antennas"] -> {low, high}`,
  `data["wifi"]["ap_unit"] -> {low, high}`, `data["lidar_tiers"] -> list[{key, name, low, high, automotive_grade, ...}]`.

- [ ] **Step 1: Write the failing test**

```python
# tests/test_cost.py
from wifi_radar_slam.cost import load_cost_data

COST_YAML = "data/cost_data.yaml"


def test_cost_data_has_wifi_and_lidar_tiers():
    d = load_cost_data(COST_YAML)
    assert d["currency"] == "USD"
    # WiFi receivers include the headline (Pi+nexmon) and low-end (ESP32) options
    rx = d["wifi"]["receivers"]
    assert {"pi_nexmon", "esp32"} <= set(rx)
    assert rx["esp32"]["low"] < rx["pi_nexmon"]["high"]
    # antennas and an AP unit (for the deployed sensitivity) exist
    assert d["wifi"]["antennas"]["low"] > 0
    assert d["wifi"]["ap_unit"]["low"] > 0
    # LiDAR envelope spans legacy spinning down to a budget 2D floor, and MUST include
    # the ouster_os1 tier -- the sensor whose params models A/B actually simulate.
    tiers = {t["key"]: t for t in d["lidar_tiers"]}
    assert {"legacy_spinning", "ouster_os1", "mid_solid_state",
            "cheap_solid_state", "budget_2d"} <= set(tiers)
    assert tiers["legacy_spinning"]["low"] > tiers["ouster_os1"]["high"]
    assert tiers["ouster_os1"]["low"] > tiers["mid_solid_state"]["high"]
    # the budget 2D scanner is explicitly NOT an automotive peer (honesty guard)
    assert tiers["budget_2d"]["automotive_grade"] is False
    assert tiers["cheap_solid_state"]["automotive_grade"] is True


def test_every_price_entry_is_sourced_and_dated():
    d = load_cost_data(COST_YAML)
    entries = list(d["wifi"]["receivers"].values()) + [
        d["wifi"]["antennas"], d["wifi"]["ap_unit"]] + d["lidar_tiers"]
    for e in entries:
        assert e["low"] <= e["high"]
        assert e["source"] and isinstance(e["source"], str)   # text citation, no fabricated URLs
        assert e["date"]
```

- [ ] **Step 2: Run test to verify it fails**

Run: `pytest tests/test_cost.py -v`
Expected: FAIL with `ModuleNotFoundError: No module named 'wifi_radar_slam.cost'`

- [ ] **Step 3: Write the data file and the loader**

```yaml
# data/cost_data.yaml
# Sourced hardware prices for the paper-2 cost model (RQ5).
# Every entry carries low/high (USD), a text `source` citation and a `date`.
# `verified: false` marks a price taken from press/market reporting that must be
# re-checked against a primary vendor page before submission (see docs/literature-paper2.md).
currency: USD

wifi:
  receivers:
    esp32:
      low: 5
      high: 15
      source: "Espressif ESP32 SoC; CSI capture via ESP-IDF (vendor product page)"
      source_url: "https://www.espressif.com/en/products/socs/esp32"
      date: "2026-07"
      verified: true
      note: "low-end CSI node"
    pi_nexmon:
      low: 35
      high: 75
      source: "Raspberry Pi 4 + nexmon_csi firmware patch (seemoo-lab/nexmon_csi)"
      source_url: "https://github.com/seemoo-lab/nexmon_csi"
      date: "2026-07"
      verified: true
      note: "headline commodity CSI receiver used by the paper"
    intel5300:
      low: 20
      high: 40
      source: "Intel 5300 NIC (Halperin linux-80211n-csitool); used-market price"
      date: "2026-07"
      verified: false
      note: "legacy commodity NIC; the classic CSI platform"
  antennas:
    low: 5
    high: 20
    source: "2-3x commodity 2.4/5 GHz antennas (retail)"
    date: "2026-07"
    verified: false
    note: "vehicle-side receive antennas"
  ap_unit:
    low: 30
    high: 80
    source: "Commodity consumer WiFi access point (retail)"
    date: "2026-07"
    verified: false
    note: "ONLY counted in the deployed-AP sensitivity; ambient APs are free in the headline"

lidar_tiers:
  - key: legacy_spinning
    name: "Legacy high-end spinning (Velodyne HDL-64 class)"
    low: 75000
    high: 80000
    automotive_grade: true
    source: "Velodyne HDL-64E historical list price, widely reported"
    date: "2024"
    verified: false
    note: "the grade that actually delivers full 3D mapping"
  - key: ouster_os1
    name: "Mid/high spinning (Ouster OS1) -- THE SENSOR OUR A/B MODELS SIMULATE"
    low: 8000
    high: 24000
    automotive_grade: true
    source: "Ouster OS1 market/list pricing (mid-range spinning LiDAR)"
    date: "2025"
    verified: false
    note: "A/B were measured at OUSTER_OS1 params (120 m, +/-3 cm, 360 deg). The cost-normalized table MUST use this tier for LiDAR rows, so measured performance is priced at the sensor that produces it."
  - key: mid_solid_state
    name: "Mid solid-state automotive (Luminar Halo / series solid-state)"
    low: 500
    high: 600
    automotive_grade: true
    source: "Electronic Design, '$500 Price Point' (automotive solid-state LiDAR)"
    date: "2026"
    verified: false
  - key: cheap_solid_state
    name: "Cheap solid-state automotive (MicroVision Movia S / Hesai)"
    low: 100
    high: 200
    automotive_grade: true
    source: "AOL/press, '$200 Lidar Could Reshuffle Auto Sensor Economics' (Movia S ~$200 production, $100 long-term goal)"
    date: "2025"
    verified: false
    note: "volume ADAS target; $100 is a stated long-term goal, not current unit price"
  - key: budget_2d
    name: "Budget 2D scanner (Slamtec RPLIDAR A1)"
    low: 99
    high: 99
    automotive_grade: false
    source: "Slamtec RPLIDAR A1 vendor product page"
    source_url: "https://www.slamtec.com/en/Lidar/A1"
    date: "2026-07"
    verified: true
    note: "2D INDOOR-grade -- a price FLOOR, not a fair automotive peer. Never compare mapping quality against this tier."
```

```python
# src/wifi_radar_slam/cost.py
"""Cost model (paper 2, RQ5): WiFi sensing package vs the automotive LiDAR envelope.

Pure functions over a sourced price file (`data/cost_data.yaml`) and the RQ3 metrics.
Every price is a (low, high) range with a text citation and date; outputs propagate
ranges rather than inventing point estimates.
"""
from __future__ import annotations
import yaml


def load_cost_data(path: str) -> dict:
    """Load the sourced price data (same yaml.safe_load idiom as config.load_config)."""
    with open(path, "r") as f:
        return yaml.safe_load(f)
```

- [ ] **Step 4: Run test to verify it passes**

Run: `pytest tests/test_cost.py -v`
Expected: PASS (2 tests)

- [ ] **Step 5: Commit**

```bash
git add data/cost_data.yaml src/wifi_radar_slam/cost.py tests/test_cost.py
git commit -m "paper2(cost): sourced price data (WiFi BOM + LiDAR envelope) + loader"
```

---

### Task 2: `wifi_package_cost`, `lidar_envelope`, `cost_ratio`

**Files:**
- Modify: `src/wifi_radar_slam/cost.py` (append)
- Test: `tests/test_cost.py` (append)

**Interfaces:**
- Consumes: `load_cost_data` (Task 1) and its schema.
- Produces:
  `wifi_package_cost(data, rx="pi_nexmon", n_aps=0) -> (low, high)`;
  `lidar_envelope(data) -> list[dict]` (each with `key`, `name`, `low`, `high`, `automotive_grade`);
  `cost_ratio(wifi_range, lidar_range) -> (low, high)` — how many **×cheaper** WiFi is.

- [ ] **Step 1: Write the failing test**

```python
# append to tests/test_cost.py
import pytest
from wifi_radar_slam.cost import wifi_package_cost, lidar_envelope, cost_ratio


def test_wifi_package_cost_headline_and_deployed():
    d = load_cost_data(COST_YAML)
    lo, hi = wifi_package_cost(d)                       # default rx=pi_nexmon + antennas
    exp_lo = d["wifi"]["receivers"]["pi_nexmon"]["low"] + d["wifi"]["antennas"]["low"]
    exp_hi = d["wifi"]["receivers"]["pi_nexmon"]["high"] + d["wifi"]["antennas"]["high"]
    assert (lo, hi) == (exp_lo, exp_hi)
    # low-end receiver is cheaper
    assert wifi_package_cost(d, rx="esp32")[0] < lo
    # deployed-AP sensitivity adds n_aps * ap_unit
    ap = d["wifi"]["ap_unit"]
    lo3, hi3 = wifi_package_cost(d, n_aps=3)
    assert lo3 == exp_lo + 3 * ap["low"]
    assert hi3 == exp_hi + 3 * ap["high"]


def test_unknown_receiver_raises():
    d = load_cost_data(COST_YAML)
    with pytest.raises(KeyError):
        wifi_package_cost(d, rx="not_a_receiver")


def test_lidar_envelope_ordered_and_flagged():
    d = load_cost_data(COST_YAML)
    tiers = lidar_envelope(d)
    assert len(tiers) == 5
    keys = [t["key"] for t in tiers]
    assert "budget_2d" in keys and "ouster_os1" in keys
    assert all(isinstance(t["low"], float) for t in tiers)


def test_cost_ratio_is_a_conservative_range():
    # WiFi $50-100 vs LiDAR $500-600:
    #   conservative low  = cheapest LiDAR / priciest WiFi = 500/100 = 5x
    #   optimistic  high  = priciest  LiDAR / cheapest WiFi = 600/50  = 12x
    lo, hi = cost_ratio((50.0, 100.0), (500.0, 600.0))
    assert lo == pytest.approx(5.0)
    assert hi == pytest.approx(12.0)
```

- [ ] **Step 2: Run test to verify it fails**

Run: `pytest tests/test_cost.py -v`
Expected: FAIL with `ImportError: cannot import name 'wifi_package_cost'`

- [ ] **Step 3: Write minimal implementation** (append to `cost.py`)

```python
def _range(item) -> tuple[float, float]:
    return float(item["low"]), float(item["high"])


def wifi_package_cost(data: dict, rx: str = "pi_nexmon", n_aps: int = 0) -> tuple[float, float]:
    """Vehicle-side WiFi sensing package: one CSI receiver + antennas.

    Headline (ambient-free): APs are pre-existing infrastructure -> not counted.
    `n_aps > 0` adds the deployed-AP sensitivity. Vehicle compute is assumed to be
    the existing SoC ($0 marginal).
    """
    receivers = data["wifi"]["receivers"]
    if rx not in receivers:
        raise KeyError(f"unknown receiver {rx!r}; have {sorted(receivers)}")
    lo, hi = _range(receivers[rx])
    a_lo, a_hi = _range(data["wifi"]["antennas"])
    lo, hi = lo + a_lo, hi + a_hi
    if n_aps:
        p_lo, p_hi = _range(data["wifi"]["ap_unit"])
        lo, hi = lo + n_aps * p_lo, hi + n_aps * p_hi
    return lo, hi


def lidar_envelope(data: dict) -> list[dict]:
    """The LiDAR price tiers, with numeric low/high coerced to float."""
    return [{**t, "low": float(t["low"]), "high": float(t["high"])}
            for t in data["lidar_tiers"]]


def cost_ratio(wifi_range, lidar_range) -> tuple[float, float]:
    """How many x cheaper the WiFi package is than a LiDAR, as a (low, high) range.

    Conservative low  = cheapest LiDAR / priciest  WiFi
    Optimistic   high = priciest  LiDAR / cheapest WiFi
    """
    w_lo, w_hi = float(wifi_range[0]), float(wifi_range[1])
    l_lo, l_hi = float(lidar_range[0]), float(lidar_range[1])
    return l_lo / w_hi, l_hi / w_lo
```

- [ ] **Step 4: Run test to verify it passes**

Run: `pytest tests/test_cost.py -v`
Expected: PASS (6 tests)

- [ ] **Step 5: Commit**

```bash
git add src/wifi_radar_slam/cost.py tests/test_cost.py
git commit -m "paper2(cost): package cost, LiDAR envelope, cost ratio"
```

---

### Task 3: `cost_normalized` (tie price to the RQ3 accuracy)

**Files:**
- Modify: `src/wifi_radar_slam/cost.py` (append)
- Test: `tests/test_cost.py` (append)

**Interfaces:**
- Consumes: a `(low, high)` price range + an RQ3 metric value.
- Produces: `cost_normalized(price_range, metric_value, mode) -> (low, high)`.
  `mode="accuracy"` → `price * ATE` ($·m; lower = better; rewards cheap **and** accurate).
  `mode="coverage"` → `price / IoU` ($ per IoU point; lower = better); `IoU <= 0` → `inf`
  (a real result: **no mapping value at any price**).

- [ ] **Step 1: Write the failing test**

```python
# append to tests/test_cost.py
import math
from wifi_radar_slam.cost import cost_normalized


def test_cost_normalized_accuracy_is_price_times_error():
    # $50-100 package at 0.03 m ATE -> 1.5-3.0 $.m (lower is better)
    lo, hi = cost_normalized((50.0, 100.0), 0.03, mode="accuracy")
    assert lo == pytest.approx(1.5)
    assert hi == pytest.approx(3.0)


def test_cost_normalized_coverage_is_price_per_iou():
    lo, hi = cost_normalized((50.0, 100.0), 0.5, mode="coverage")
    assert lo == pytest.approx(100.0)
    assert hi == pytest.approx(200.0)


def test_zero_iou_means_no_mapping_value_at_any_price():
    # WiFi-realistic IoU ~ 0 -> infinite $ per IoU point (not a crash)
    lo, hi = cost_normalized((50.0, 100.0), 0.0, mode="coverage")
    assert math.isinf(lo) and math.isinf(hi)


def test_unknown_mode_raises():
    with pytest.raises(ValueError):
        cost_normalized((50.0, 100.0), 0.5, mode="nonsense")
```

- [ ] **Step 2: Run test to verify it fails**

Run: `pytest tests/test_cost.py -v`
Expected: FAIL with `ImportError: cannot import name 'cost_normalized'`

- [ ] **Step 3: Write minimal implementation** (append to `cost.py`)

```python
def cost_normalized(price_range, metric_value: float, mode: str) -> tuple[float, float]:
    """Overlay price on an RQ3 metric so cost is never read without accuracy.

    mode="accuracy": price * ATE   -> $.m  (lower is better; cheap AND accurate wins)
    mode="coverage": price / IoU   -> $ per IoU point (lower is better).
                     IoU <= 0 returns inf: no mapping value at any price (the
                     WiFi-realistic case), which is a result, not an error.
    """
    lo, hi = float(price_range[0]), float(price_range[1])
    v = float(metric_value)
    if mode == "accuracy":
        return lo * v, hi * v
    if mode == "coverage":
        if v <= 0.0:
            return float("inf"), float("inf")
        return lo / v, hi / v
    raise ValueError(f"unknown mode {mode!r}; use 'accuracy' or 'coverage'")
```

- [ ] **Step 4: Run test to verify it passes**

Run: `pytest tests/test_cost.py -v`
Expected: PASS (10 tests)

- [ ] **Step 5: Commit**

```bash
git add src/wifi_radar_slam/cost.py tests/test_cost.py
git commit -m "paper2(cost): cost-normalized overlay (\$.ATE, \$/IoU) with zero-IoU guard"
```

---

### Task 4: WiFi metrics JSON + report script + docs section

**Files:**
- Create: `data/wifi_results.json`
- Create: `experiments/run_cost_model.py`
- Modify: `docs/results-paper2.md` (append a "Cost (RQ5)" section from the script output)

**Interfaces:**
- Consumes: `load_cost_data`, `wifi_package_cost`, `lidar_envelope`, `cost_ratio`,
  `cost_normalized` (Tasks 1–3); `data/lidar_geo_results.json`,
  `data/lidar_sionna_results.json` (existing, six-metric dicts per scene).
- Produces: `data/cost_results.json` and the printed markdown tables.

- [ ] **Step 1: Create the machine-readable WiFi metrics**

The WiFi numbers currently live only in a markdown table; the overlay needs them as data.
These are the **frozen paper-1 results** (`docs/results-v1.md`, submitted IoT-J `v0.7.1`) —
do not recompute them.

```json
{
  "_note": "Frozen paper-1 WiFi results (docs/results-v1.md, submitted IoT-J v0.7.1). Do not recompute: these must match the published paper. 'realistic' IoU is ~0; street realistic mapping was characterized qualitatively (not tabulated), so only ATE is given there.",
  "controlled_wall": {
    "oracle":    {"ate": 0.045, "rpe": 0.007, "chamfer": 0.51, "map_accuracy": 0.25, "map_completeness": 0.77, "iou": 0.79},
    "realistic": {"ate": 0.027, "chamfer": 4.1, "map_accuracy": 4.8, "map_completeness": 3.5, "iou": 0.0}
  },
  "street_canyon": {
    "oracle":    {"ate": 0.116, "rpe": 0.007, "chamfer": 12.3, "map_accuracy": 0.30, "map_completeness": 24.4, "iou": 0.077},
    "realistic": {"ate": 0.09, "iou": 0.0}
  }
}
```

- [ ] **Step 2: Write the report script**

```python
# experiments/run_cost_model.py
"""Paper-2 cost model (RQ5): WiFi sensing package vs the automotive LiDAR envelope.

Joins sourced prices (data/cost_data.yaml) with the RQ3 accuracy numbers and emits:
  1. absolute cost table (WiFi package vs each LiDAR tier)
  2. cost-ratio envelope (x cheaper), headline + deployed-AP sensitivity
  3. cost-normalized performance ($.ATE localization value, $/IoU mapping value)
Runs locally in seconds; no Sionna, no server.

    python experiments/run_cost_model.py
"""
import json

from wifi_radar_slam.cost import (load_cost_data, wifi_package_cost, lidar_envelope,
                                  cost_ratio, cost_normalized)

COST_YAML = "data/cost_data.yaml"
WIFI = "data/wifi_results.json"
LIDAR_A = "data/lidar_geo_results.json"
LIDAR_B = "data/lidar_sionna_results.json"


def _fmt(rng, unit="", nd=0):
    lo, hi = rng
    if lo == float("inf"):
        return "inf (no mapping value at any price)"
    if nd:
        return f"{lo:,.{nd}f}-{hi:,.{nd}f}{unit}"
    return f"{lo:,.0f}-{hi:,.0f}{unit}"


def main() -> None:
    data = load_cost_data(COST_YAML)
    wifi = json.load(open(WIFI))
    lidar = {"A (geometric)": json.load(open(LIDAR_A)),
             "B (Sionna diffuse)": json.load(open(LIDAR_B))}

    pkg = wifi_package_cost(data)                       # headline: pi_nexmon + antennas
    pkg_esp = wifi_package_cost(data, rx="esp32")
    pkg_dep3 = wifi_package_cost(data, n_aps=3)         # deployed-AP sensitivity
    tiers = lidar_envelope(data)

    print("## Cost (RQ5)\n")
    print(f"WiFi package (headline, ambient-free: Pi4+nexmon + antennas): "
          f"**${_fmt(pkg)}**")
    print(f"WiFi package (low-end ESP32 + antennas): ${_fmt(pkg_esp)}")
    print(f"WiFi package (+3 deployed APs, sensitivity): ${_fmt(pkg_dep3)}\n")

    print("| LiDAR tier | Automotive? | Price (USD) | WiFi is X cheaper (headline) "
          "| X cheaper (+3 APs) |")
    print("|---|---|---:|---:|---:|")
    for t in tiers:
        r = cost_ratio(pkg, (t["low"], t["high"]))
        rd = cost_ratio(pkg_dep3, (t["low"], t["high"]))
        auto = "yes" if t["automotive_grade"] else "**no (2D floor)**"
        print(f"| {t['name']} | {auto} | ${_fmt((t['low'], t['high']))} "
              f"| {_fmt(r, 'x', 1)} | {_fmt(rd, 'x', 1)} |")

    print("\n### Cost-normalized performance (price read WITH accuracy)\n")
    print("$.m = price x ATE (localization value, lower better). "
          "$/IoU = price per IoU point (mapping value, lower better).\n")
    print("| Scene | Sensor | Price (USD) | $.m (localization) | $/IoU (mapping) |")
    print("|---|---|---:|---:|---:|")
    # CONSISTENCY: models A/B were measured at OUSTER_OS1 params, so their measured
    # performance must be priced at the OS1 tier -- NOT at a cheap solid-state tier we
    # never simulated. Pricing high-end performance at budget-LiDAR cost would be
    # apples-to-oranges (and would flatter LiDAR).
    os1 = next(t for t in tiers if t["key"] == "ouster_os1")
    rows = []
    for scene in ("controlled_wall", "street_canyon"):
        w = wifi[scene]["realistic"]
        rows.append((scene, "WiFi (realistic CSI)", pkg, w.get("ate"), w.get("iou", 0.0)))
        for name, res in lidar.items():
            m = res[scene]
            rows.append((scene, f"LiDAR {name} @ OS1 price", (os1["low"], os1["high"]),
                         m["ate"], m["iou"]))
    for scene, sensor, price, ate, iou in rows:
        acc = cost_normalized(price, ate, "accuracy")
        cov = cost_normalized(price, iou, "coverage")
        print(f"| {scene} | {sensor} | ${_fmt(price)} | {_fmt(acc, '', 2)} "
              f"| {_fmt(cov, '', 0)} |")

    out = {
        "wifi_package_usd": {"headline": pkg, "esp32": pkg_esp, "deployed_3ap": pkg_dep3},
        "lidar_tiers": [{"key": t["key"], "low": t["low"], "high": t["high"],
                         "automotive_grade": t["automotive_grade"],
                         "ratio_vs_wifi_headline": cost_ratio(pkg, (t["low"], t["high"]))}
                        for t in tiers],
    }
    with open("data/cost_results.json", "w") as f:
        json.dump(out, f, indent=2)
    print("\nsaved -> data/cost_results.json")


if __name__ == "__main__":
    main()
```

- [ ] **Step 3: Run it and check the output is sane**

Run: `python experiments/run_cost_model.py`
Expected: prints the WiFi package (~$40–95 headline), a **5-row** LiDAR tier table with
×-cheaper ranges (huge vs legacy spinning and the OS1 class we simulated; single-digit×
vs cheap solid-state), the cost-normalized table (LiDAR rows priced at the **OS1** tier;
WiFi-realistic `$/IoU` shows `inf` on both scenes — no mapping value at any price), and
writes `data/cost_results.json`.

- [ ] **Step 4: Paste the generated markdown into the docs**

Append the script's stdout as a new `## Cost (RQ5)` section at the end of
`docs/results-paper2.md`, followed by this interpretation paragraph:

```markdown
**Reading the cost table.** The WiFi sensing package is ~100-1000x cheaper than the
LiDAR grades that actually deliver full 3D mapping (Ouster OS1 class -- the sensor our
A/B models simulate -- and legacy spinning), and still cheaper than the cheapest
emerging automotive solid-state unit, though there the gap narrows to single-digit x.

**Two honesty notes.** (1) The cost-normalized rows price LiDAR at the **OS1 tier**,
because that is the sensor whose parameters produced our measured A/B accuracy; the
cheaper solid-state tiers show where the market is heading, but we did **not** measure
their (lower) performance, so pricing OS1-grade accuracy at $150 would be
apples-to-oranges. (2) The budget 2D scanner is a **price floor, not a peer** -- it is
not automotive-grade and we never compare mapping quality against it.

**The trade.** WiFi wins **localization value** decisively ($.m: it is both cheaper and,
on the controlled scene, more accurate). But its **mapping value is undefined**
($/IoU -> inf, because realistic-CSI IoU ~ 0) -- LiDAR is the only modality that buys map
coverage at any price. That asymmetry is precisely the gap RQ2 (deep learning) and RQ4
(fusion) exist to close, and it is the honest boundary of the "drop-in replacement" claim.
```

- [ ] **Step 5: Full suite green, then commit**

Run: `pytest -q`
Expected: all pass (10 new cost tests included).

```bash
git add data/wifi_results.json data/cost_results.json experiments/run_cost_model.py \
        docs/results-paper2.md
git commit -m "paper2(cost): cost model results + RQ5 section (cost read with accuracy)"
```

---

## After this plan

Merge `paper2-cost-model` into `paper2-wifi-vs-lidar`, update
`papers/2-wifi-vs-lidar/DOSSIER.md` with the RQ5 headline (cost gap + the
mapping-value-undefined finding), and tag the next paper-2 milestone. Remaining
sub-projects, each its own brainstorm → spec → plan: **RQ4 fusion** and **RQ2 deep
learning** — both now motivated directly by the cost table's mapping-value gap.
