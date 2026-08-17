#!/usr/bin/env python3
"""Render the R1.6/R1.7 result figures (local PNG). Okabe-Ito colorblind-safe series colors."""
import json
import matplotlib
matplotlib.use("Agg")
import matplotlib.pyplot as plt

ORACLE, MUSIC, MAPC = "#0072B2", "#E69F00", "#009E73"   # Okabe-Ito (CVD-safe)
plt.rcParams.update({"font.size": 10, "axes.grid": True, "grid.alpha": 0.3,
                     "axes.spines.top": False, "axes.spines.right": False})

# ---- R1.6 AP sensitivity ----
ap = json.load(open("results/ap_sensitivity.json"))
sig = ap["sigmas_m"]
fig, (a1, a2) = plt.subplots(1, 2, figsize=(9, 3.6))
a1.errorbar(sig, [r["ate_mean"] for r in ap["oracle"]], [r["ate_std"] for r in ap["oracle"]],
            marker="o", color=ORACLE, label="oracle", capsize=3)
a1.errorbar(sig, [r["ate_mean"] for r in ap["music"]], [r["ate_std"] for r in ap["music"]],
            marker="s", color=MUSIC, label="realistic MUSIC", capsize=3)
a1.set_xlabel("AP-position error σ (m)"); a1.set_ylabel("trajectory ATE (m)")
a1.set_title("Localization is insensitive to AP error"); a1.set_ylim(0, None); a1.legend(frameon=False)
a2.errorbar(sig, [r["map_acc_mean"] for r in ap["music"]], [r["map_acc_std"] for r in ap["music"]],
            marker="s", color=MAPC, capsize=3)
a2.set_xlabel("AP-position error σ (m)"); a2.set_ylabel("map accuracy (m)  ·  MUSIC")
a2.set_title("The map degrades gently")
fig.suptitle("R1.6 — AP-position sensitivity  (8 seeds, nominal scene)", fontweight="bold")
fig.tight_layout(); fig.savefig("docs/figures/ap_sensitivity.png", dpi=150); plt.close(fig)

# ---- R1.7 blockage ----
bl = json.load(open("results/blockage_robustness.json"))
L = bl["lengths_frames"]
fig, ax = plt.subplots(figsize=(6, 3.8))
ax.plot(L, [r["drift_during_mean"] for r in bl["oracle"]], marker="o", color=ORACLE, label="oracle")
ax.plot(L, [r["drift_during_mean"] for r in bl["music"]], marker="s", color=MUSIC, label="realistic MUSIC")
ax.set_xlabel("blackout length (frames of 240)"); ax.set_ylabel("drift during blackout (m)")
ax.set_title("R1.7 — PF dead-reckons through measurement blackouts", fontweight="bold")
ax.set_ylim(0, None); ax.legend(frameon=False)
fig.tight_layout(); fig.savefig("docs/figures/blockage_robustness.png", dpi=150); plt.close(fig)
print("wrote docs/figures/ap_sensitivity.png and docs/figures/blockage_robustness.png")
