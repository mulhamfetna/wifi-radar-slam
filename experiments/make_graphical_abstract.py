#!/usr/bin/env python3
"""Render the graphical abstract for the consolidated IEEE Access paper (local files only).

Three bands: the verdict (localization vs mapping), the mechanism (the geometry ablation),
and the hardware anchor. Every number here is transcribed from ``manuscript/main.tex`` --
Table "WiFi versus LiDAR" (controlled-scene rows) and Table "phantom rate" (ablation cells).
Nothing is recomputed, so this script has no dependency on the simulation outputs.

Outputs (300 dpi PNG + vector PDF + dark mirror) into ``manuscript/figures/``:
    graphical-abstract.png  graphical-abstract.pdf  graphical-abstract-dark.png

Palette: dataviz categorical slots 1/2/3, validated with the skill's validator under
``--pairs all`` in both modes (light: worst CVD dE 9.2, normal-vision 24.0; aqua carries a
sub-3:1 contrast WARN, answered by the relief rule -- every bar is directly labelled and
every row is named in text, so identity is never colour-alone).

Bars use square ends rather than the 4 px rounded data-ends of the mark spec: matplotlib
cannot round a corner in data coordinates without distorting it as the axis rescales, and
the attempt collapsed short bars into hairlines. The rest of the spec holds -- thin marks,
a surface gap between rows, recessive grid, selective direct labels.
"""
import matplotlib
matplotlib.use("Agg")
import matplotlib.pyplot as plt
from matplotlib.patches import FancyBboxPatch

# ---------------------------------------------------------------- data (from main.tex)
# Table "WiFi versus LiDAR on identical scenes and metrics" -- CONTROLLED scene rows.
LOC = [("WiFi\nrealistic CSI", 0.098, 0.028, "wifi", "0.098 m ± 0.028"),
       ("LiDAR-A", 0.102, None, "lidar", "0.102 m")]          # ATE (m), lower is better
MAP = [("WiFi\nrealistic CSI", 0.000, None, "wifi", "0.000"),
       ("LiDAR-A", 0.977, None, "lidar", "0.977")]            # occupancy IoU, higher better

# Table "phantom rate". Cell M (MUSIC, bistatic) is omitted: it reports no phantom rate,
# only IoU 0.003.
ABL = [("CFAR · bistatic\n5.2 GHz / 160 MHz",   18.2, 0.6,  "wifi",  "18.2 % ± 0.6"),
       ("CFAR · monostatic\n5.2 GHz / 160 MHz",  0.1, 0.2,  "wifi",  "0.1 % ± 0.2"),
       ("CFAR · monostatic\n77 GHz / 160 MHz",   0.0, None, "radar", "0.0 %"),
       ("CFAR · monostatic\n77 GHz / 4 GHz",     9.0, 1.5,  "radar", "9.0 % ± 1.5")]

STEPS = [("geometry", "phantoms all but vanish"),
         ("carrier", "no change"),
         ("10× bandwidth", "phantoms return")]

THEMES = {
    "light": dict(surface="#fcfcfb", ink="#0b0b0b", ink2="#52514e", ink3="#87867f",
                  rule="#e3e2dd", band="#f0efea",
                  wifi="#2a78d6", lidar="#eb6834", radar="#1baf7a"),
    "dark":  dict(surface="#1a1a19", ink="#ffffff", ink2="#c3c2b7", ink3="#8a8983",
                  rule="#33322f", band="#262523",
                  wifi="#3987e5", lidar="#d95926", radar="#199e70"),
}

GOOD, CRITICAL = "#0ca30c", "#d03b3b"   # reserved status steps, never themed

BAR_H = 0.46          # well under the 1.0 row pitch, so rows keep a clear surface gap


def bars(ax, rows, t, xmax, xlabel, title=None, glyph=None, gcolor=None, stub=0.012):
    """Directly-labelled horizontal bars on a recessive grid.

    Zero-valued rows get a small stub so they read as "measured zero" rather than
    "missing"; the label carries the actual value either way.
    """
    n = len(rows)
    for i, (name, val, err, kind, label) in enumerate(rows):
        y = n - 1 - i
        ax.barh(y, max(val, stub * xmax), height=BAR_H, color=t[kind],
                linewidth=0, zorder=3)
        if err is not None:
            ax.errorbar(val, y, xerr=err, color=t["ink2"], lw=1.3,
                        capsize=3.5, capthick=1.3, zorder=4)
        ax.text(max(val, stub * xmax) + (err or 0) + xmax * 0.025, y, label,
                va="center", ha="left", fontsize=11, fontweight="bold", color=t["ink"])

    ax.set_yticks(range(n))
    ax.set_yticklabels([r[0] for r in reversed(rows)], fontsize=9.5, color=t["ink2"])
    ax.set_xlim(0, xmax)
    ax.set_ylim(-0.6, n - 0.4)
    ax.set_xlabel(xlabel, fontsize=9, color=t["ink3"], labelpad=5)
    ax.tick_params(axis="x", colors=t["ink3"], labelsize=8, length=3)
    ax.tick_params(axis="y", length=0, pad=6)
    for s in ("top", "right", "left"):
        ax.spines[s].set_visible(False)
    ax.spines["bottom"].set_color(t["rule"])
    ax.grid(axis="x", color=t["rule"], lw=0.8, zorder=0)
    ax.set_axisbelow(True)
    if title:
        # the heading wears primary ink; only the verdict glyph carries status colour,
        # so no text is ever mistaken for a series label
        ax.set_title(title, fontsize=12.5, fontweight="bold", color=t["ink"],
                     loc="left", pad=10)
        ax.text(0, 1.045, glyph, transform=ax.transAxes, fontsize=14,
                fontweight="bold", color=gcolor, va="bottom", ha="left")


def render(mode, out_png, out_pdf=None):
    t = THEMES[mode]
    fig = plt.figure(figsize=(10.0, 8.4), dpi=300, facecolor=t["surface"])
    outer = fig.add_gridspec(4, 1, height_ratios=[0.30, 1.00, 1.18, 0.26],
                             left=0.155, right=0.965, top=0.955, bottom=0.070, hspace=0.82)

    # ---- header -------------------------------------------------------------
    fig.text(0.038, 0.975, "Can ambient WiFi replace radar or LiDAR as a SLAM front-end?",
             fontsize=17, fontweight="bold", color=t["ink"], va="top")
    fig.text(0.038, 0.936,
             "Ray-traced sub-7 GHz WiFi on a moving vehicle, scored against real-LiDAR-anchored "
             "baselines and reproduced\non $30 of commodity silicon.   Controlled scene.",
             fontsize=10, color=t["ink2"], va="top", linespacing=1.5)

    # one legend for the whole figure -- the three colours are consistent across bands
    handles = [plt.Line2D([], [], marker="s", ls="", ms=8.5, color=t[k], label=v)
               for k, v in (("wifi", "WiFi · 5.2 GHz"), ("lidar", "LiDAR"),
                            ("radar", "radar · 77 GHz"))]
    fig.legend(handles=handles, loc="upper right", bbox_to_anchor=(0.965, 0.905),
               ncols=3, frameon=False, fontsize=9.5, labelcolor=t["ink2"],
               handletextpad=0.35, columnspacing=1.5)

    # ---- band 1: the verdict ------------------------------------------------
    g1 = outer[1].subgridspec(1, 2, wspace=0.55)
    a1, a2 = fig.add_subplot(g1[0]), fig.add_subplot(g1[1])
    for ax in (a1, a2):
        ax.set_facecolor(t["surface"])
    bars(a1, LOC, t, 0.20, "absolute trajectory error (m) · lower is better",
         "     LOCALIZATION works", "✓", GOOD)
    bars(a2, MAP, t, 1.55, "occupancy IoU · higher is better",
         "     MAPPING hits a ceiling", "✗", CRITICAL)
    a1.text(0, -0.30, "a statistical tie — at 84–600× lower sensor cost",
            transform=a1.transAxes, fontsize=10, color=t["ink"], style="italic")
    a2.text(0, -0.30, "≈89 % of realistic-CSI detections are phantoms",
            transform=a2.transAxes, fontsize=10, color=t["ink"], style="italic")

    # ---- band 2: the mechanism ---------------------------------------------
    g2 = outer[2].subgridspec(1, 2, width_ratios=[0.74, 0.26], wspace=0.02)
    ax, axn = fig.add_subplot(g2[0]), fig.add_subplot(g2[1])
    ax.set_facecolor(t["surface"])
    bars(ax, ABL, t, 24.0, "phantom detection rate (%) · lower is better")
    ax.set_title("Why?  The ceiling is GEOMETRY, not the carrier",
                 fontsize=12.5, fontweight="bold", color=t["ink"], loc="left", pad=10)

    # the three single-variable comparisons the ablation isolates, drawn in their own
    # column so they never overplot the bars
    axn.set_axis_off()
    axn.set_xlim(0, 1)
    axn.set_ylim(-0.6, len(ABL) - 0.4)
    for k, (head, tail) in enumerate(STEPS):
        y0, y1 = len(ABL) - 1 - k, len(ABL) - 2 - k
        axn.annotate("", xy=(0.12, y1 + 0.28), xytext=(0.12, y0 - 0.28),
                     arrowprops=dict(arrowstyle="-|>", color=t["ink3"], lw=1.3,
                                     shrinkA=0, shrinkB=0))
        axn.text(0.24, (y0 + y1) / 2 + 0.10, head, va="center", ha="left",
                 fontsize=10, fontweight="bold", color=t["ink"])
        axn.text(0.24, (y0 + y1) / 2 - 0.16, tail, va="center", ha="left",
                 fontsize=9, color=t["ink2"])

    # ---- band 3: the hardware anchor ---------------------------------------
    ax3 = fig.add_subplot(outer[3])
    ax3.set_axis_off()
    # anchor the strip just under the ablation axes rather than letting the grid row
    # stretch -- otherwise a short final row inherits a full hspace of dead canvas
    b2, b3 = ax.get_position(), ax3.get_position()
    ax3.set_position([b3.x0, b2.y0 - 0.155, b3.width, b3.height])
    ax3.add_patch(FancyBboxPatch((-0.14, 0.0), 1.14, 1.0, transform=ax3.transAxes,
                                 boxstyle="round,pad=0,rounding_size=0.04",
                                 facecolor=t["band"], linewidth=0, clip_on=False))
    ax3.text(-0.115, 0.50, "ON $30 OF SILICON", transform=ax3.transAxes, va="center",
             fontsize=11.5, fontweight="bold", color=t["ink"])
    ax3.text(0.235, 0.50,
             "two ESP32-S3 boards capture real HT40 CSI — 128 subcarriers, 114 active",
             transform=ax3.transAxes, va="center", fontsize=10, color=t["ink2"])

    fig.text(0.038, 0.020, "Mulham Fetna · University of Aleppo · "
             "github.com/mulhamfetna/wifi-radar-slam (AGPL-3.0)",
             fontsize=8.5, color=t["ink3"])

    fig.savefig(out_png, facecolor=t["surface"], dpi=300)
    if out_pdf:
        fig.savefig(out_pdf, facecolor=t["surface"])
    plt.close(fig)
    # flatten RGBA -> RGB: the surface is opaque anyway, and some submission portals
    # reject PNGs carrying an alpha channel. Pillow ships with matplotlib.
    from PIL import Image
    Image.open(out_png).convert("RGB").save(out_png, dpi=(300, 300))
    print(f"wrote {out_png}" + (f" and {out_pdf}" if out_pdf else ""))


if __name__ == "__main__":
    D = "manuscript/figures/"
    render("light", D + "graphical-abstract.png", D + "graphical-abstract.pdf")
    render("dark", D + "graphical-abstract-dark.png")
