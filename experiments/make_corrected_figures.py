#!/usr/bin/env python3
"""Figures for the revised manuscript, read from the corrected-evaluation JSON (no recompute,
except Fig. 3, which re-triangulates one seed's cached detections at ground-truth poses).

    .venv/bin/python experiments/make_corrected_figures.py   -> manuscript/elsevier/figures/
"""
from __future__ import annotations

import json
import os
import sys

import matplotlib
matplotlib.use("Agg")
import matplotlib.pyplot as plt
import numpy as np

sys.path.insert(0, os.path.dirname(__file__))
OUT = "manuscript/elsevier/figures"
# reference categorical palette, slots 1-5 in fixed order (validated: dataviz validator)
SERIES = ["#2a78d6", "#eb6834", "#1baf7a", "#eda100", "#e87ba4"]
INK, MUTED, GRID = "#2b2b29", "#6b6a63", "#e4e3dc"
SCENES = [("controlled_wall", "Controlled wall"), ("street_canyon", "Street canyon"),
          ("street_canyon_scurve", "Street, S-curve")]

plt.rcParams.update({"font.family": "serif", "font.size": 8, "axes.edgecolor": MUTED,
                     "axes.labelcolor": INK, "xtick.color": MUTED, "ytick.color": MUTED,
                     "text.color": INK, "axes.spines.top": False, "axes.spines.right": False,
                     "axes.grid": True, "grid.color": GRID, "grid.linewidth": 0.6,
                     "axes.axisbelow": True, "legend.frameon": False,
                     "pdf.fonttype": 42, "ps.fonttype": 42})


def fig_localization(S, L):
    methods = [("Dead reckoning", "slam_music/ate_dead_reckoning"),
               ("WiFi, oracle", "slam_oracle/ate"), ("WiFi, MUSIC", "slam_music/ate"),
               ("WiFi, MUSIC two-branch", "slam_music_two_branch/ate"), ("LiDAR-A", None)]
    fig, ax = plt.subplots(figsize=(3.5, 2.4))
    x = np.arange(len(SCENES)); w = 0.16
    for i, (lab, key) in enumerate(methods):
        m, s = [], []
        for sc, _ in SCENES:
            if key is None:
                v = L[sc]["noisy_summary"]["ate"]
            else:
                v = S[f"{sc}/bw160"][key]
            m.append(v["mean"]); s.append(v["std"])
        ax.bar(x + (i - 2) * w, m, w * 0.88, yerr=s, color=SERIES[i], label=lab,
               error_kw={"elinewidth": 0.7, "capsize": 1.5, "ecolor": MUTED})
    ax.set_xticks(x, [n for _, n in SCENES]); ax.set_ylabel("ATE (m)")
    ax.grid(axis="x", visible=False)
    ax.legend(ncol=2, fontsize=6.5, loc="upper left")
    fig.tight_layout(); fig.savefig(f"{OUT}/localization.pdf"); plt.close(fig)


def fig_frontend(S):
    cats = [("True facade path", ("matched_facade",)),
            ("Other real path", ("matched_other",)),
            ("Real path, front/back folded", ("folded_facade", "folded_other")),
            ("Unexplained", ("unexplained",))]
    rows, labels = [], []
    for sc, name in SCENES:
        for tag, key in (("as published", "phantoms_as_published"), ("corrected", "phantoms_corrected")):
            g = S[f"{sc}/bw160"]
            rows.append([sum(g[f"{key}/{k}"]["mean"] for k in ks) for _, ks in cats])
            labels.append(f"{name}\n{tag}")
    rows = np.array(rows) * 100
    fig, ax = plt.subplots(figsize=(3.5, 2.8))
    y = np.arange(len(rows))[::-1]
    left = np.zeros(len(rows))
    for i, (lab, _) in enumerate(cats):
        ax.barh(y, rows[:, i], 0.7, left=left, color=SERIES[i], label=lab,
                edgecolor="white", linewidth=1.0)
        left += rows[:, i]
    ax.set_yticks(y, labels, fontsize=6.5); ax.set_xlim(0, 100)
    ax.set_xlabel("Share of joint-MUSIC detections (%)"); ax.grid(axis="y", visible=False)
    ax.legend(ncol=2, fontsize=6.3, loc="upper center", bbox_to_anchor=(0.42, 1.2))
    fig.tight_layout(); fig.savefig(f"{OUT}/frontend_decomposition.pdf"); plt.close(fig)


def fig_maps():
    from make_music_detections import SCENES as PATHS, load_dets
    from wifi_radar_slam.eval.scenes import (SceneCache, excess_oracle_detections,
                                             excess_to_absolute, horizontal)
    from wifi_radar_slam.slam.particle_filter import _triangulate_bistatic, _cluster
    sc = SceneCache.load(PATHS["controlled_wall"])
    music = load_dets("controlled_wall", 160, 0)
    variants = [("Oracle", excess_to_absolute(excess_oracle_detections(sc), sc.poses, sc.ap_positions)),
                ("MUSIC, as published", music),
                ("MUSIC, corrected", excess_to_absolute(music, sc.poses, sc.ap_positions))]
    fig, axs = plt.subplots(1, 3, figsize=(5.2, 3.0), sharex=True, sharey=True)
    for ax, (name, d3) in zip(axs, variants):
        pts = []
        for p, d in zip(sc.poses, horizontal(d3, sc.ap_positions)):
            for r, a, ap in d:
                q = _triangulate_bistatic(p[:2], sc.ap_positions[int(ap)][:2], r, a)
                if q is not None:
                    pts.append(q)
        m = _cluster(np.array(pts), radius=0.5, min_support=5) if pts else np.empty((0, 2))
        ax.scatter(sc.gt_map[:, 0], sc.gt_map[:, 1], s=4, color=GRID, label="Facade footprint")
        ax.plot(sc.poses[:, 0], sc.poses[:, 1], color=MUTED, lw=1, label="Trajectory")
        ax.scatter(sc.ap_positions[:, 0], sc.ap_positions[:, 1], marker="^", s=18, color=INK, label="AP")
        if len(m):
            ax.scatter(m[:, 0], m[:, 1], s=8, color=SERIES[0], edgecolor="white", linewidth=0.4,
                       label="Mapped landmark", zorder=3)
        ax.set_title(name, fontsize=8); ax.set_aspect("equal")
        ax.set_xlim(-16, 4); ax.set_ylim(-24, 24)
    axs[0].set_ylabel("y (m)"); [a.set_xlabel("x (m)") for a in axs]
    h, lab = axs[0].get_legend_handles_labels()
    h2, lab2 = axs[2].get_legend_handles_labels()
    for hh, ll in zip(h2, lab2):
        if ll not in lab:
            h.append(hh); lab.append(ll)
    fig.legend(h, lab, ncol=4, fontsize=6.5, loc="lower center")
    fig.tight_layout(rect=(0, 0.08, 1, 1)); fig.savefig(f"{OUT}/maps_controlled.pdf"); plt.close(fig)


def main():
    os.makedirs(OUT, exist_ok=True)
    S = json.load(open("results/corrected_eval.json"))["summary"]
    L = json.load(open("results/lidar_noisy_odom_geo.json"))
    fig_localization(S, L); fig_frontend(S); fig_maps()
    if os.path.exists("results/robustness_corrected.json"):
        fig_robustness(json.load(open("results/robustness_corrected.json"))["summary"])
    print("figures ->", OUT, sorted(os.listdir(OUT)))


def fig_robustness(R):
    fig, axs = plt.subplots(1, 2, figsize=(7.0, 2.3))
    for j, (sc, name) in enumerate(SCENES[:2]):
        for fr, col in (("oracle", SERIES[1]), ("music", SERIES[2])):
            t = R[sc][f"ap_{fr}"]
            xs = list(range(len(t))); m = [r["ate"][0] for r in t]; s = [r["ate"][1] for r in t]
            axs[0].errorbar(xs, m, yerr=s, color=col, lw=1.5 if j == 0 else 1.0,
                            ls="-" if j == 0 else "--", marker="o", ms=3,
                            label=f"{name}, {'oracle' if fr == 'oracle' else 'MUSIC'}")
            t = R[sc][f"blackout_{fr}"]
            xs = [100 * r["frames"] / {"controlled_wall": 120, "street_canyon": 240}[sc] for r in t]; m = [r["ate"][0] for r in t]; s = [r["ate"][1] for r in t]
            axs[1].errorbar(xs, m, yerr=s, color=col, lw=1.5 if j == 0 else 1.0,
                            ls="-" if j == 0 else "--", marker="o", ms=3)
        for ax in axs:
            ax.axhline(R[sc]["ate_dead_reckoning"], color=MUTED, lw=0.8, ls=":" if j else "-.")
    sig = [r["sigma_m"] for r in R[SCENES[0][0]]["ap_oracle"]]
    axs[0].set_xticks(range(len(sig)), [f"{v:g}" for v in sig])
    axs[0].set_xlabel("AP position error std (m)")
    axs[1].set_xlabel("Blackout length (% of run)")
    for ax in axs:
        ax.set_ylabel("ATE (m)")
    axs[0].legend(fontsize=6)
    fig.tight_layout(); fig.savefig(f"{OUT}/robustness.pdf"); plt.close(fig)


if __name__ == "__main__":
    main()
