"""Render paper-2's data figures from the COMMITTED result files.

No number is hand-typed here: every value is read from data/*.json, so each figure is
reproducible and traceable to an artifact.

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


def _save(fig, path, **kw):
    """Write a BYTE-REPRODUCIBLE PDF.

    matplotlib stamps a CreationDate into the PDF, so re-running the script (as the
    test does) would dirty the working tree on every run even though the content is
    identical. Setting CreationDate to None makes the artifact deterministic.
    """
    fig.savefig(path, metadata={"CreationDate": None}, **kw)


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
        ax.set_xticks(list(x))
        ax.set_xticklabels([NICE[s] for s in SCENES])
        ax.set_ylabel(ylab)
        ax.grid(axis="y")
    axes[0].legend(frameon=False, ncol=3, loc="upper left")
    fig.tight_layout()
    _save(fig, f"{OUT}/paper2_fig2.pdf")
    plt.close(fig)


def fig3_cost(d):
    """Cost envelope: single series, log scale (magnitude spans 4 orders).

    Tiers run MOST-EXPENSIVE-AT-TOP (reading order), and the legend is placed OUTSIDE
    the axes -- inside, it covered the legacy-spinning bar entirely.
    """
    tiers = list(d["cost"]["lidar_tiers"])[::-1]        # most expensive at top
    wifi_lo, wifi_hi = d["cost"]["wifi_package_usd"]["headline"]
    fig, ax = plt.subplots(figsize=(7.0, 2.6))
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
        ax.annotate(f"{r[0]:,.0f}-{r[1]:,.0f}$\\times$ cheaper",
                    (max(t["high"], t["low"] * 1.06), i),
                    xytext=(6, 0), textcoords="offset points", va="center",
                    fontsize=6, color="#333333")
    ax.axvspan(wifi_lo, wifi_hi, color=WIFI, alpha=0.85, label="WiFi package")
    ax.set_xscale("log")
    ax.set_xlabel("Unit price (USD, log scale)")
    ax.set_yticks(list(y))
    ax.set_yticklabels(names)
    ax.set_xlim(5, 1.2e6)
    ax.legend(frameon=False, loc="upper center", bbox_to_anchor=(0.5, -0.28), ncol=2)
    ax.grid(axis="x")
    fig.tight_layout()
    _save(fig, f"{OUT}/paper2_fig3.pdf")
    plt.close(fig)


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
    axes[0].set_ylabel("\\$$\\cdot$m  (localization cost $\\downarrow$)")
    # headroom so the legend (placed below) never collides with the tallest bar label
    lo0, hi0 = axes[0].get_ylim()
    axes[0].set_ylim(lo0, hi0 * 4)

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
                vals.append(price[s][1] / iou)
                is_inf.append(False)
            else:
                vals.append(top)                       # off the chart, not zero
                is_inf.append(True)
        bars = axes[1].bar([xx + (i - 1) * w for xx in x], vals, w, label=s,
                           color=COLOR[s], hatch=HATCH[s], edgecolor="white",
                           linewidth=0.6)
        # SELECTIVE labels only: labelling every bar made the adjacent LiDAR values
        # collide ("24,57124,000"). The panel's message is "WiFi = inf, LiDAR = finite";
        # the exact figures are in the table.
        for b, v, inf in zip(bars, vals, is_inf):
            if inf:
                axes[1].annotate(r"$\infty$ (no map)",
                                 (b.get_x() + b.get_width() / 2, v), ha="center",
                                 va="bottom", fontsize=6, color="#B00020",
                                 xytext=(0, 1), textcoords="offset points")
    axes[1].set_ylim(0, top * 1.22)
    axes[1].set_ylabel("\\$/IoU  (mapping cost $\\downarrow$)")
    for ax in axes:
        ax.set_xticks(list(x))
        ax.set_xticklabels([NICE[s] for s in SCENES])
        ax.grid(axis="y")
    # legend BELOW both panels: inside panel (a) it collided with the tallest bar label
    axes[0].legend(frameon=False, ncol=3, loc="upper center",
                   bbox_to_anchor=(1.1, -0.18))
    fig.tight_layout()
    _save(fig, f"{OUT}/paper2_fig4.pdf", bbox_inches="tight")
    plt.close(fig)


def fig5_fusion(d):
    """ATE for solo vs fused across the 4 (scene x LiDAR model) configs."""
    fus = d["fusion"]
    configs = [(s, m) for s in SCENES for m in ("A_geometric", "B_sionna")]
    rows = ["WiFi", "LiDAR", "Fused"]
    fig, ax = plt.subplots(figsize=(7.0, 2.6))
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
    ax.set_xticklabels([f"{NICE[s]}\nLiDAR-{m.split('_')[0]}" for s, m in configs])
    ax.set_ylabel("ATE (m) $\\downarrow$")
    ax.grid(axis="y")
    ax.legend(frameon=False, ncol=3, loc="upper left")
    fig.tight_layout()
    _save(fig, f"{OUT}/paper2_fig5.pdf")
    plt.close(fig)


def fig6_ceiling(d):
    """The mapping ceiling. Panel a: where MUSIC detections come from.
    Panel b: the same facade paths triangulated with TRUE vs MUSIC parameters."""
    iso = d["ceiling"]
    fig, axes = plt.subplots(1, 2, figsize=(7.0, 2.7))
    parts = [("no_plausible_match_pct", "Phantom (no real path)", "#555555", ".."),
             ("matched_non_facade_pct", "Non-facade (discrimination)", "#AAAAAA", "//"),
             ("matched_true_facade_pct", "True facade path", WIFI, "")]
    bottom = [0.0, 0.0]
    for key, lab, col, hat in parts:                    # panel a: stacked provenance
        vals = [iso[s][key] for s in SCENES]
        axes[0].bar([NICE[s] for s in SCENES], vals, 0.5, bottom=bottom, label=lab,
                    color=col, hatch=hat, edgecolor="white", linewidth=0.8)
        for i, v in enumerate(vals):
            if v > 4:
                # a plain white label is illegible over the dotted hatch -- give it an
                # opaque plate so the dominant 89% number actually reads.
                axes[0].annotate(f"{v:.1f}%", (i, bottom[i] + v / 2), ha="center",
                                 va="center", fontsize=7, color="#111111", weight="bold",
                                 bbox=dict(boxstyle="round,pad=0.18", fc="white",
                                           ec="none", alpha=0.92))
        bottom = [b + v for b, v in zip(bottom, vals)]
    axes[0].set_ylabel("Share of MUSIC detections (%)")
    axes[0].set_ylim(0, 100)
    axes[0].legend(frameon=False, fontsize=6, loc="upper center",
                   bbox_to_anchor=(0.5, -0.14), ncol=1)

    w, x = 0.3, range(len(SCENES))                      # panel b: true vs MUSIC params
    for i, (key, lab, col, hat) in enumerate(
            (("triangulation_within_1m_true_params_pct", "True delay/AoA", WIFI, ""),
             ("triangulation_within_1m_music_params_pct", "MUSIC delay/AoA", LIDAR_A, "//"))):
        vals = [iso[s][key] for s in SCENES]
        bars = axes[1].bar([xx + (i - 0.5) * w for xx in x], vals, w, label=lab,
                           color=col, hatch=hat, edgecolor="white", linewidth=0.6)
        _label_bars(axes[1], bars, "{:.1f}")
    axes[1].set_xticks(list(x))
    axes[1].set_xticklabels([NICE[s] for s in SCENES])
    axes[1].set_ylabel("Triangulated within 1 m (%) $\\uparrow$")
    axes[1].set_ylim(0, 125)
    axes[1].grid(axis="y")
    # legend BELOW: at upper-right it collided with the 100.0 bar label
    axes[1].legend(frameon=False, fontsize=6, loc="upper center",
                   bbox_to_anchor=(0.5, -0.14), ncol=1)
    fig.tight_layout()
    _save(fig, f"{OUT}/paper2_fig6.pdf", bbox_inches="tight")
    plt.close(fig)


def main() -> None:
    os.makedirs(OUT, exist_ok=True)
    d = figure_data()
    fig2_rq3(d)
    fig3_cost(d)
    fig4_cost_normalized(d)
    fig5_fusion(d)
    fig6_ceiling(d)
    print(f"wrote {OUT}/paper2_fig2..6.pdf")


if __name__ == "__main__":
    main()
