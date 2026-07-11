# Paper 2 Manuscript Implementation Plan

> **For agentic workers:** REQUIRED SUB-SKILL: Use superpowers:subagent-driven-development (recommended) or superpowers:executing-plans to implement this plan task-by-task. Steps use checkbox (`- [ ]`) syntax for tracking.

**Goal:** Assemble paper 2's IEEE IoT-J manuscript — *"Can Ambient WiFi Replace LiDAR for Automotive SLAM? Localization Yes, Mapping No — and Why"* — with all figures generated from committed data files.

**Architecture:** A figure script renders 5 data figures from the committed JSONs (Fig. 1 is TikZ inside the `.tex`). `main.tex` reuses paper-1's vendored IEEEtran scaffold (no siunitx). Every number in the prose traces to a file in `data/`.

**Tech Stack:** Python 3 + matplotlib (already a core dep). LaTeX: `pdflatex` + `bibtex` (both available locally), vendored `IEEEtran.cls`/`.bst`.

## Global Constraints

- **Branch:** all work on `paper2-manuscript`, cut from `paper2-wifi-vs-lidar`; merge back on completion. Never commit to `main` or any `paper1-*` ref.
- **NO NEW EXPERIMENTS.** Every number comes from a committed file:
  `data/{lidar_geo,lidar_sionna,wifi,kitti,cost,fusion,enhanced_map,map_filter_f1,mapping_floor_isolation}*.{json,yaml}`.
- **NO HAND-TYPED NUMBERS IN FIGURES.** `make_paper2_figures.py` reads the JSONs.
- **Do not edit paper 1's frozen submission** (`papers/1-wifi-radar-slam/`). Paper 2 *cites* it and states the refinement in its own §VIII.
- **Figure rules (dataviz):**
  - Validated colorblind-safe palette (Okabe-Ito, all CVD checks pass):
    `WIFI=#0072B2, LIDAR_A=#D55E00, LIDAR_B=#009E73, FUSED=#CC79A7`.
  - **Color follows the ENTITY, never the rank** — WiFi is blue in *every* figure.
  - **NEVER a dual-axis chart.** Two measures of different scale ⇒ **two panels**.
  - **Grayscale relief:** hatch patterns per series (IEEE prints B/W).
  - **Contrast relief:** direct value labels on bars (the `#CC79A7` contrast WARN makes
    labels obligatory, not optional).
  - Legend for ≥2 series; recessive grid/axes; text in black/gray, never the series color.
- **Honesty guards carried into the prose:** simulation-based (state in the abstract); the
  cost claim depends on the ambient-AP premise and *inverts* with self-deployed APs; LiDAR is
  priced at the tier we simulated (OS1); the fusion street/A 8× regression is reported.
- **Prose is authored at execution** against the content specs below. The plan gives the exact
  *numbers, tables, claims, and structure* (the error-prone parts) in full; it does not
  duplicate several hundred lines of paper prose that would then be rewritten verbatim.

---

### Task 1: `make_paper2_figures.py` — 5 data figures from the committed JSONs

**Files:**
- Create: `experiments/make_paper2_figures.py`
- Test: `tests/test_paper2_figures.py`
- Output: `docs/assets/paper2_fig{2..6}.pdf`

**Interfaces:**
- Consumes: the committed result JSONs (paths listed in the script).
- Produces: 5 PDF figures + `figure_data()` returning the parsed numbers (so a test can
  assert the figures are driven by the files, not literals).

- [ ] **Step 1: Write the failing test**

```python
# tests/test_paper2_figures.py
import json
import subprocess
import sys
from pathlib import Path


def test_figure_data_comes_from_the_committed_jsons():
    from experiments.make_paper2_figures import figure_data
    d = figure_data()
    # RQ3 numbers must equal what is in the result files (no hand-typed values)
    lidar_a = json.load(open("data/lidar_geo_results.json"))
    assert d["rq3"]["controlled_wall"]["LiDAR-A"]["ate"] == \
        lidar_a["controlled_wall"]["ate"]
    # the mapping-ceiling figure must use the isolation result
    iso = json.load(open("data/mapping_floor_isolation.json"))
    assert d["ceiling"]["controlled_wall"]["no_plausible_match_pct"] == \
        iso["controlled_wall"]["no_plausible_match_pct"]


def test_script_renders_all_five_figures(tmp_path):
    out = Path("docs/assets")
    subprocess.run([sys.executable, "experiments/make_paper2_figures.py"], check=True)
    for i in range(2, 7):
        f = out / f"paper2_fig{i}.pdf"
        assert f.exists() and f.stat().st_size > 1000, f"missing/empty {f}"
```

- [ ] **Step 2: Run test to verify it fails**

Run: `pytest tests/test_paper2_figures.py -v`
Expected: FAIL — `ModuleNotFoundError: No module named 'experiments.make_paper2_figures'`
(add an empty `experiments/__init__.py` if the import needs it).

- [ ] **Step 3: Write the figure script**

```python
# experiments/make_paper2_figures.py
"""Render paper-2's data figures from the COMMITTED result files.

No number is hand-typed here: every value is read from data/*.json|yaml, so each
figure is reproducible and traceable to an artifact.

Palette: Okabe-Ito (colorblind-safe; validated -- worst adjacent CVD dE 17.9 deutan).
Colour follows the ENTITY, never the rank: WiFi is blue in every figure.
IEEE prints in greyscale, so every series also carries a hatch (secondary encoding),
and every bar carries a direct value label (contrast relief).
NEVER a dual axis: two measures of different scale are two panels.
"""
import json
import os

import matplotlib
matplotlib.use("Agg")
import matplotlib.pyplot as plt
import yaml

OUT = "docs/assets"
WIFI, LIDAR_A, LIDAR_B, FUSED = "#0072B2", "#D55E00", "#009E73", "#CC79A7"
HATCH = {"WiFi": "", "LiDAR-A": "//", "LiDAR-B": "\\\\", "Fused": "xx"}
COLOR = {"WiFi": WIFI, "LiDAR-A": LIDAR_A, "LiDAR-B": LIDAR_B, "Fused": FUSED}
SCENES = ["controlled_wall", "street_canyon"]
NICE = {"controlled_wall": "Controlled wall", "street_canyon": "Street canyon"}

plt.rcParams.update({
    "font.size": 8, "axes.labelsize": 8, "axes.titlesize": 8,
    "legend.fontsize": 7, "xtick.labelsize": 7, "ytick.labelsize": 7,
    "axes.spines.top": False, "axes.spines.right": False,
    "grid.alpha": 0.3, "grid.linewidth": 0.4, "figure.dpi": 200,
})


def figure_data():
    """Every number the figures use, read from the committed artifacts."""
    ga = json.load(open("data/lidar_geo_results.json"))
    gb = json.load(open("data/lidar_sionna_results.json"))
    wf = json.load(open("data/wifi_results.json"))
    cost = json.load(open("data/cost_results.json"))
    fus = json.load(open("data/fusion_results.json"))
    iso = json.load(open("data/mapping_floor_isolation.json"))
    rq3 = {s: {"WiFi": wf[s]["realistic"], "LiDAR-A": ga[s], "LiDAR-B": gb[s]}
           for s in SCENES}
    return {"rq3": rq3, "cost": cost, "fusion": fus, "ceiling": iso}


def _label_bars(ax, bars, fmt="{:.2f}"):
    for b in bars:
        h = b.get_height()
        ax.annotate(fmt.format(h), (b.get_x() + b.get_width() / 2, h),
                    ha="center", va="bottom", fontsize=6, color="#333333",
                    xytext=(0, 1), textcoords="offset points")


def fig2_rq3(d):
    """Two panels (ATE, IoU) -- never one dual axis."""
    fig, axes = plt.subplots(1, 2, figsize=(7.0, 2.3))
    series = ["WiFi", "LiDAR-A", "LiDAR-B"]
    for ax, key, ylab in ((axes[0], "ate", "ATE (m) $\\downarrow$"),
                          (axes[1], "iou", "Occupancy IoU $\\uparrow$")):
        w, x = 0.26, range(len(SCENES))
        for i, s in enumerate(series):
            vals = [d["rq3"][sc][s].get(key, 0.0) for sc in SCENES]
            bars = ax.bar([xx + (i - 1) * w for xx in x], vals, w, label=s,
                          color=COLOR[s], hatch=HATCH[s], edgecolor="white",
                          linewidth=0.6)
            _label_bars(ax, bars, "{:.3f}" if key == "ate" else "{:.2f}")
        ax.set_xticks(list(x)); ax.set_xticklabels([NICE[s] for s in SCENES])
        ax.set_ylabel(ylab); ax.grid(axis="y")
    axes[0].legend(frameon=False, ncol=3, loc="upper left")
    fig.tight_layout(); fig.savefig(f"{OUT}/paper2_fig2.pdf"); plt.close(fig)


def fig3_cost(d):
    """Cost envelope: single series, log scale (magnitude spans 4 orders)."""
    tiers = d["cost"]["lidar_tiers"]
    wifi_lo, wifi_hi = d["cost"]["wifi_package_usd"]["headline"]
    fig, ax = plt.subplots(figsize=(7.0, 2.4))
    names = [t["key"].replace("_", " ") for t in tiers]
    lows = [t["low"] for t in tiers]
    highs = [t["high"] for t in tiers]
    y = range(len(tiers))
    # a fixed-price tier (budget_2d: low == high) would render as a ZERO-WIDTH bar and
    # vanish on the log axis -- give it a visible minimum extent.
    widths = [max(h - l, l * 0.06) for l, h in zip(lows, highs)]
    ax.barh(list(y), widths, left=lows, height=0.5,
            color="#999999", edgecolor="white", label="LiDAR price range")
    for i, t in enumerate(tiers):
        r = t["ratio_vs_wifi_headline"]
        ax.annotate(f"{r[0]:,.0f}-{r[1]:,.0f}x cheaper", (t["high"], i),
                    xytext=(4, 0), textcoords="offset points", va="center",
                    fontsize=6, color="#333333")
    ax.axvspan(wifi_lo, wifi_hi, color=WIFI, alpha=0.85, label="WiFi package")
    ax.set_xscale("log"); ax.set_xlabel("Unit price (USD, log scale)")
    ax.set_yticks(list(y)); ax.set_yticklabels(names)
    ax.legend(frameon=False, loc="lower right"); ax.grid(axis="x")
    fig.tight_layout(); fig.savefig(f"{OUT}/paper2_fig3.pdf"); plt.close(fig)


def fig4_cost_normalized(d):
    """TWO PANELS: $.m (localization value) and $/IoU (mapping value).
    These have different scales -- a dual axis would be the #1 chart mistake."""
    os1 = next(t for t in d["cost"]["lidar_tiers"] if t["key"] == "ouster_os1")
    wlo, whi = d["cost"]["wifi_package_usd"]["headline"]
    fig, axes = plt.subplots(1, 2, figsize=(7.0, 2.3))
    series = ["WiFi", "LiDAR-A", "LiDAR-B"]
    price = {"WiFi": (wlo, whi), "LiDAR-A": (os1["low"], os1["high"]),
             "LiDAR-B": (os1["low"], os1["high"])}
    w, x = 0.26, range(len(SCENES))
    for i, s in enumerate(series):                     # panel a: $.m (lower better)
        vals = [price[s][1] * d["rq3"][sc][s]["ate"] for sc in SCENES]
        bars = axes[0].bar([xx + (i - 1) * w for xx in x], vals, w, label=s,
                           color=COLOR[s], hatch=HATCH[s], edgecolor="white",
                           linewidth=0.6)
        _label_bars(axes[0], bars, "{:,.0f}")
    axes[0].set_yscale("log")
    axes[0].set_ylabel("$\\cdot$m  (localization cost, $\\downarrow$)")
    # panel b: $/IoU (lower better). WiFi's IoU is 0 => the cost is INFINITE. A
    # zero-height bar would read as "cheapest" -- the exact opposite of the truth -- so
    # an infinite value is drawn to the top of the axis and labelled, never as 0.
    finite = [price[s][1] / d["rq3"][sc][s]["iou"]
              for s in series for sc in SCENES if d["rq3"][sc][s].get("iou", 0.0) > 0]
    top = max(finite) * 1.35
    for i, s in enumerate(series):
        vals, is_inf = [], []
        for sc in SCENES:
            iou = d["rq3"][sc][s].get("iou", 0.0)
            if iou > 0:
                vals.append(price[s][1] / iou); is_inf.append(False)
            else:
                vals.append(top); is_inf.append(True)     # off the chart, not zero
        bars = axes[1].bar([xx + (i - 1) * w for xx in x], vals, w, label=s,
                           color=COLOR[s], hatch=HATCH[s], edgecolor="white",
                           linewidth=0.6)
        for b, v, inf in zip(bars, vals, is_inf):
            axes[1].annotate(r"$\infty$ (no map)" if inf else f"{v:,.0f}",
                             (b.get_x() + b.get_width() / 2, v), ha="center",
                             va="bottom", fontsize=6,
                             color="#B00020" if inf else "#333333",
                             xytext=(0, 1), textcoords="offset points")
    axes[1].set_ylim(0, top * 1.18)
    axes[1].set_ylabel("\\$/IoU  (mapping cost, $\\downarrow$)")
    for ax in axes:
        ax.set_xticks(list(x)); ax.set_xticklabels([NICE[s] for s in SCENES])
        ax.grid(axis="y")
    axes[0].legend(frameon=False, ncol=3, loc="upper left")
    fig.tight_layout(); fig.savefig(f"{OUT}/paper2_fig4.pdf"); plt.close(fig)


def fig5_fusion(d):
    """ATE for solo vs fused across the 4 (scene x LiDAR model) configs."""
    fus = d["fusion"]
    configs = [(s, m) for s in SCENES for m in ("A_geometric", "B_sionna")]
    rows = ["WiFi", "LiDAR", "Fused"]
    fig, ax = plt.subplots(figsize=(7.0, 2.5))
    w, x = 0.26, range(len(configs))
    for i, r in enumerate(rows):
        vals = []
        for s, m in configs:
            if r == "WiFi":
                vals.append(fus[s]["wifi_only"]["ate"])
            elif r == "LiDAR":
                vals.append(fus[s][f"lidar_only_{m}"]["ate"])
            else:
                vals.append(fus[s][f"fused_tight_{m}"]["ate"])
        key = {"WiFi": "WiFi", "LiDAR": "LiDAR-A", "Fused": "Fused"}[r]
        bars = ax.bar([xx + (i - 1) * w for xx in x], vals, w, label=r,
                      color=COLOR[key], hatch=HATCH[key], edgecolor="white",
                      linewidth=0.6)
        _label_bars(ax, bars, "{:.3f}")
    ax.set_xticks(list(x))
    ax.set_xticklabels([f"{NICE[s]}\n{m.split('_')[0]}" for s, m in configs])
    ax.set_ylabel("ATE (m) $\\downarrow$"); ax.grid(axis="y")
    ax.legend(frameon=False, ncol=3, loc="upper left")
    fig.tight_layout(); fig.savefig(f"{OUT}/paper2_fig5.pdf"); plt.close(fig)


def fig6_ceiling(d):
    """The mapping ceiling. Panel a: where MUSIC detections come from.
    Panel b: the same facade paths triangulated with TRUE vs MUSIC parameters."""
    iso = d["ceiling"]
    fig, axes = plt.subplots(1, 2, figsize=(7.0, 2.4))
    parts = [("no_plausible_match_pct", "Phantom (no real path)", "#555555", ".."),
             ("matched_non_facade_pct", "Non-facade path (discrimination)", "#AAAAAA", "//"),
             ("matched_true_facade_pct", "True facade path", WIFI, "")]
    bottom = [0.0, 0.0]
    for key, lab, col, hat in parts:                    # panel a: stacked provenance
        vals = [iso[s][key] for s in SCENES]
        axes[0].bar([NICE[s] for s in SCENES], vals, 0.5, bottom=bottom, label=lab,
                    color=col, hatch=hat, edgecolor="white", linewidth=0.8)
        for i, v in enumerate(vals):
            if v > 4:
                axes[0].annotate(f"{v:.1f}%", (i, bottom[i] + v / 2), ha="center",
                                 va="center", fontsize=6, color="white")
        bottom = [b + v for b, v in zip(bottom, vals)]
    axes[0].set_ylabel("Share of MUSIC detections (%)")
    axes[0].legend(frameon=False, fontsize=6, loc="lower center",
                   bbox_to_anchor=(0.5, -0.55), ncol=1)
    w, x = 0.3, range(len(SCENES))                      # panel b: true vs MUSIC params
    for i, (key, lab, col, hat) in enumerate(
            (("triangulation_within_1m_true_params_pct", "True delay/AoA", WIFI, ""),
             ("triangulation_within_1m_music_params_pct", "MUSIC delay/AoA", LIDAR_A, "//"))):
        vals = [iso[s][key] for s in SCENES]
        bars = axes[1].bar([xx + (i - 0.5) * w for xx in x], vals, w, label=lab,
                           color=col, hatch=hat, edgecolor="white", linewidth=0.6)
        _label_bars(axes[1], bars, "{:.1f}")
    axes[1].set_xticks(list(x)); axes[1].set_xticklabels([NICE[s] for s in SCENES])
    axes[1].set_ylabel("Triangulated within 1 m (%) $\\uparrow$")
    axes[1].set_ylim(0, 112); axes[1].grid(axis="y")
    axes[1].legend(frameon=False, fontsize=6, loc="upper right")
    fig.tight_layout(); fig.savefig(f"{OUT}/paper2_fig6.pdf"); plt.close(fig)


def main() -> None:
    os.makedirs(OUT, exist_ok=True)
    d = figure_data()
    fig2_rq3(d); fig3_cost(d); fig4_cost_normalized(d); fig5_fusion(d); fig6_ceiling(d)
    print(f"wrote {OUT}/paper2_fig2..6.pdf")


if __name__ == "__main__":
    main()
```

- [ ] **Step 4: Run test to verify it passes, then LOOK at the figures**

Run: `pytest tests/test_paper2_figures.py -v` → PASS (2 tests)
Then **render and inspect** (the validator checks colour, not layout): open each
`docs/assets/paper2_fig{2..6}.pdf` and check for label collisions, clipped bars, and
overflow. Fix any layout issue before proceeding. Check each figure against the
dataviz anti-patterns list (no dual axis, no cycled hues, legend present, labels not
on every point where it crowds).

- [ ] **Step 5: Commit**

```bash
git add experiments/make_paper2_figures.py tests/test_paper2_figures.py docs/assets/paper2_fig*.pdf
git commit -m "paper2(ms): figures 2-6 generated from committed result JSONs"
```

---

### Task 2: Scaffold + front matter + bibliography

**Files:**
- Create: `papers/2-wifi-vs-lidar/{IEEEtran.cls,IEEEtran.bst}` (copied from paper 1)
- Create: `papers/2-wifi-vs-lidar/refs.bib`
- Create: `papers/2-wifi-vs-lidar/main.tex` (preamble → §III)

**Interfaces:**
- Consumes: paper-1's preamble conventions (vendored IEEEtran, inline unit macros,
  `\graphicspath{{../../docs/assets/}}`, no siunitx, `\providecommand` BIB fallbacks).
- Produces: a `main.tex` that compiles through §III.

- [ ] **Step 1: Copy the vendored class/style and set up the preamble**

```bash
cp papers/1-wifi-radar-slam/IEEEtran.cls papers/1-wifi-radar-slam/IEEEtran.bst \
   papers/2-wifi-vs-lidar/
```

`main.tex` preamble is paper-1's verbatim (same `\documentclass[journal]{IEEEtran}`,
`amsmath,amssymb,graphicx,booktabs,url,hyperref,tikz`, the inline `\SI/\SIrange/\num/\si`
macros, the `\providecommand` BIB fallbacks, and `\graphicspath{{../../docs/assets/}}`).

- [ ] **Step 2: Write `refs.bib`**

Start from `papers/1-wifi-radar-slam/refs.bib` (already cleaned — internal `note` fields
stripped) and **add** the works surfaced by the review (`docs/literature-paper2.md`):
P2SLAM (T-RO 2022); radio-fingerprint SLAM (arXiv 2305.13635); ViWiD (arXiv 2209.08091);
WiFi-RSS-augmented visual SLAM (arXiv 1903.06687); DLoc/LocAP; two-level WiFi+LiDAR graph
SLAM (arXiv 2206.08733); EKF WiFi/LiDAR/IMU fusion (arXiv 2509.23118); laser+WiFi
re-localization (PMC7570627); CSI→3-D point cloud transformer (arXiv 2410.16303); DensePose
from WiFi (arXiv 2301.00250); RF-Pose (CVPR 2018); NeRF²; KITTI odometry (Geiger et al.);
Ouster OS1 / MicroVision / Luminar price sources; **and paper 1** (self-citation).
**No fabricated entries** — every key must correspond to a work in
`docs/literature-paper2.md` or paper-1's bib.

- [ ] **Step 3: Write the front matter and §I–§III**

- `\title{Can Ambient WiFi Replace LiDAR for Automotive SLAM? Localization Yes, Mapping No --- and Why}`
- Author block: Mulham Fetna, ORCID 0009-0006-4432-798X, contact@mulhamfetna.com.
- **Abstract** must state: the question; the head-to-head method; that it is
  **simulation-based** (Sionna RT) with a real-LiDAR KITTI anchor; the four headline
  numbers — WiFi realistic ATE **0.027 m** (better than both LiDAR models on the controlled
  scene) vs WiFi mapping **IoU ≈ 0**; **84–600×** lower sensor cost; fusion **+0.5 % cost →
  36–79 % better ATE**; and the ceiling mechanism (**~89 % phantom detections** + a
  **6.45 m** range bias, *not* path discrimination).
- **§I Introduction:** the LiDAR cost problem; the replacement question; the four
  contributions, each with its headline number; a paragraph stating the paper-1 relationship.
- **§II Related Work:** from `docs/literature-paper2.md` — P2SLAM (standalone WiFi/CSI SLAM
  but **indoor**, benchmarked vs *visual* SLAM); radio-fingerprint SLAM (outdoor/on-vehicle
  but **RSS not CSI**, slow UGV, best accuracy needs LiDAR fusion); ViWiD / RSS-augmented
  visual SLAM / DLoc (WiFi *augments* a primary sensor); WiFi+LiDAR fusion works; DL RF
  sensing. Close with the **open cell**: on-vehicle, outdoor, commodity-CSI, head-to-head.
- **§III System Model:** the shared pipeline — WiFi (CSI → joint 2-D MUSIC → bistatic
  triangulation → particle-filter SLAM) and LiDAR (2-D BEV scan → scan-to-map ICP) — the
  common **2-D BEV comparison plane**, the same footprint ground truth, and the six metrics
  (ATE, RPE, Chamfer, map-accuracy, map-completeness, occupancy IoU). Include **Fig. 1**, a
  TikZ diagram of both pipelines converging on the comparison plane (adapt paper-1's Fig. 1).

- [ ] **Step 4: Verify it compiles this far**

Run:
```bash
cd papers/2-wifi-vs-lidar && pdflatex -interaction=nonstopmode main.tex >/dev/null && echo BUILD_OK
```
Expected: `BUILD_OK` and a `main.pdf` (citations will show as `[?]` until Task 4's bibtex
pass — that is expected at this stage).

- [ ] **Step 5: Commit**

```bash
git add papers/2-wifi-vs-lidar/
git commit -m "paper2(ms): scaffold, front matter, related work, system model"
```

---

### Task 3: Results sections §IV–§VIII (the four contributions)

**Files:**
- Modify: `papers/2-wifi-vs-lidar/main.tex`

**Interfaces:**
- Consumes: the figures from Task 1; the numbers below (all traceable to `data/`).
- Produces: §IV–§VIII with 5 tables and Figs. 2–6 placed.

**Every number below is the value to typeset — copy them exactly.**

- [ ] **Step 1: §IV LiDAR baselines + the KITTI anchor**

Models **A** (geometric bbox ray-cast) and **B** (Sionna diffuse optical) as an **envelope**,
both at `OUSTER_OS1` parameters (120 m, ±3 cm, 360°). State plainly that A is
precise-but-low-coverage and B is dense-but-noisier, so a real LiDAR sits **between** them —
this is why LiDAR is reported as an envelope, not a single baseline.
**KITTI anchor:** seq-04, 271 frames, 394 m → **RPE 0.154 m/frame, aligned ATE 1.16 m
(~0.3 % drift)** — real-LiDAR-plausible (SOTA is ~0.1–0.5 %), validating the back-end.

- [ ] **Step 2: §V RQ3 — accuracy (Table I + Fig. 2)**

Table I (both scenes × {WiFi oracle, WiFi realistic, LiDAR-A, LiDAR-B} × six metrics):

| Scene | Sensor | ATE | RPE | Chamfer | map-acc | map-compl | IoU |
|---|---|---|---|---|---|---|---|
| controlled | WiFi oracle | 0.045 | 0.007 | 0.51 | 0.25 | 0.77 | 0.79 |
| controlled | WiFi realistic | 0.027 | — | 4.1 | 4.8 | 3.5 | ~0 |
| controlled | LiDAR-A | 0.102 | 0.030 | 0.209 | 0.250 | 0.168 | 0.977 |
| controlled | LiDAR-B | 0.483 | 0.055 | 0.187 | 0.251 | 0.123 | 1.000 |
| street | WiFi oracle | 0.116 | 0.007 | 12.3 | 0.30 | 24.4 | 0.077 |
| street | WiFi realistic | ~0.09 | — | bounded | — | — | ~0 |
| street | LiDAR-A | 0.026 | 0.017 | 8.674 | 0.251 | 17.097 | 0.163 |
| street | LiDAR-B | 0.857 | 0.117 | 3.734 | 2.125 | 5.344 | 0.261 |

Claim: **WiFi wins localization** (realistic 0.027 m beats both LiDAR models on the
controlled scene); **LiDAR wins mapping** (IoU up to 1.000 vs WiFi ≈ 0).

- [ ] **Step 3: §VI RQ5 — cost (Table II + Figs. 3–4)**

WiFi package (ambient-free: Pi 4 + nexmon + antennas) **\$40–95**; ESP32 variant \$10–35;
+3 deployed APs \$130–335. LiDAR tiers and ×-cheaper vs the WiFi headline:

| Tier | Price (USD) | × cheaper |
|---|---|---|
| Legacy spinning (HDL-64 class) | 75,000–80,000 | 789–2,000 |
| **Ouster OS1 — the tier we simulate** | 8,000–24,000 | **84–600** |
| Mid solid-state | 500–600 | 5.3–15 |
| Cheap solid-state | 100–200 | 1.1–5.0 |
| Budget 2-D scanner (price floor, **not** an automotive peer) | 99 | 1.0–2.5 |

Cost-normalized: WiFi **1–9 \$·m** vs LiDAR **212–2,448 \$·m** (2–3 orders better accuracy
per dollar); WiFi **\$/IoU = ∞** (it buys no map coverage at any price).
**Honesty (must appear):** the advantage depends on the **ambient-AP premise** — with 3
self-deployed APs WiFi can be **more expensive** (0.3–1.5×) than the cheapest solid-state
LiDAR; and LiDAR rows are priced at the **OS1 tier we simulated**, not a cheap tier whose
performance we never measured.

- [ ] **Step 4: §VII RQ4 — fusion (Table III + Fig. 5)**

Tight fusion = one particle filter, `w = w_wifi(bistatic) × w_lidar(scan-match)`, map = union.
Loose = naive equal-weight baseline.

| Scene / LiDAR | WiFi ATE | LiDAR ATE | **Fused-tight** | Fused-loose | LiDAR IoU | Fused IoU |
|---|---|---|---|---|---|---|
| controlled / A | 0.081 | 0.102 | **0.065** | 0.056 | 0.977 | 0.913 |
| controlled / B | 0.081 | 0.212 | **0.044** | 0.086 | 1.000 | 0.913 |
| street / A | 0.281 | **0.027** | 0.218 | 0.137 | 0.149 | 0.177 |
| street / B | 0.281 | 0.844 | **0.175** | 0.338 | 0.262 | 0.309 |

Claims: tight fusion beats **both** solo modalities in **3 of 4** configs (up to **79 %**
better than the LiDAR); it **degrades** the stronger sensor under large accuracy mismatch
(street/A: 0.027 → 0.218, an **8× regression**) — the condition is **sensor parity**. The
honest fix (confidence-adaptive weighting) is future work, **not** claimed. Cost verdict:
WiFi is **+0.2–1.2 %** on an OS1-class LiDAR, so **when fusion helps it is essentially free**.

- [ ] **Step 5: §VIII RQ2 — the mapping ceiling (Table IV + Fig. 6)**

The ladder (none / heuristic / RandomForest / MLP) leaves **IoU 0.000** on both scenes; the
learned rungs reject everything and empty the map. Corrected discriminator F1 on
**MUSIC-observable** features: **0.00–0.45** held-out, **0.00–0.20** cross-scene.

The isolation experiment (same facade paths, TRUE vs MUSIC parameters):

| | controlled | street |
|---|---|---|
| Phantom (matches **no** real path) | **89.2 %** | **89.5 %** |
| Non-facade real path (*discrimination*) | 2.2 % | 8.4 % |
| True facade path | 8.6 % | 2.0 % |
| Triangulated <1 m — **TRUE** params | 100 % | 100 % |
| Triangulated <1 m — **MUSIC** params | **2.4 %** | 76.7 % |
| MUSIC range error (median) | **6.45 m** | 2.11 m |

**Claim:** the floor is dominated by **phantom detections (~89 %)** and **estimator range
bias** (6.45 m — far beyond the 0.94 m resolution limit at 160 MHz, so a *bias*, not a
resolution bound); **path discrimination is the smallest term (2–8 %)**. A filter can neither
*invent* the real paths 89 % of detections lack nor *correct* the bias — hence every rung fails.
**Paper-1 refinement (must be stated plainly):** paper 1's *empirical* results stand (the
oracle map; the 60 GHz + 16-antenna null result); its *interpretation* — that discrimination
is the floor and a learned discriminator would fix mapping — does not survive this experiment,
and its reported F1 used `elevation`, unmeasurable by a single-ULA 2-D front-end.
**Silver lining:** street shows **76.7 %** of correctly-matched facade paths triangulate
within 1 m ⇒ the geometry **is** recoverable; the ceiling is set by the **front-end**, not the
physics.

- [ ] **Step 6: Commit**

```bash
git add papers/2-wifi-vs-lidar/main.tex
git commit -m "paper2(ms): results sections IV-VIII (comparison, cost, fusion, ceiling)"
```

---

### Task 4: Discussion, conclusion, build, README

**Files:**
- Modify: `papers/2-wifi-vs-lidar/main.tex`
- Create: `papers/2-wifi-vs-lidar/README.md`
- Output: `papers/2-wifi-vs-lidar/main.pdf`

- [ ] **Step 1: §IX Discussion & Limitations**

State, without hedging: **simulation-based** (Sionna RT ray tracing; the only real-data
element is the KITTI LiDAR anchor); the **2-D BEV** comparison plane; the **ambient-AP
premise** the cost claim rests on; fusion needs **confidence-adaptive weighting** to be safe;
the LiDAR envelope brackets rather than reproduces a specific unit; prices are dated
estimates with ranges.

- [ ] **Step 2: §X Future Work, §XI Conclusion**

Future work: **end-to-end CSI → geometry** (bypassing the estimator); **phantom suppression /
range-bias correction** at the front-end (where the ceiling actually sits);
confidence-adaptive fusion; real on-vehicle CSI.
Conclusion restates the thesis precisely: WiFi replaces LiDAR **for localization** at
84–600× lower cost, **not for mapping**; the mapping ceiling is a **front-end** limit
(phantoms + bias), not a discrimination limit; the practical recommendation is the **hybrid**.

- [ ] **Step 3: Full build**

```bash
cd papers/2-wifi-vs-lidar
pdflatex -interaction=nonstopmode main.tex >/dev/null
bibtex main >/dev/null
pdflatex -interaction=nonstopmode main.tex >/dev/null
pdflatex -interaction=nonstopmode main.tex >/dev/null
echo "--- undefined refs/citations (must be empty) ---"
grep -iE "undefined (reference|citation)|LaTeX Warning: Citation" main.log || echo "NONE"
pdfinfo main.pdf | grep Pages
```
Expected: `NONE` for undefined references/citations, and a page count of ~10–12.

- [ ] **Step 4: Write `README.md`**

Document: the build command (`pdflatex; bibtex; pdflatex ×2`), that IEEEtran is vendored and
siunitx is not required, how to regenerate every figure
(`python experiments/make_paper2_figures.py`), and the **data-provenance table** mapping each
section to its `data/*.json|yaml` source.

- [ ] **Step 5: Full suite + commit**

Run: `pytest -q` → all pass.

```bash
git add papers/2-wifi-vs-lidar/
git commit -m "paper2(ms): discussion, conclusion, build (main.pdf), README"
```

---

## After this plan

Merge `paper2-manuscript` into `paper2-wifi-vs-lidar`, update the DOSSIER (manuscript status
+ page count), and tag `paper2-v0.5.0`. The submission package (cover letter, supplementary,
keywords, IoT-J topic taxonomy) is a **separate** cycle, mirroring paper 1's — do not start it
before design approval.
