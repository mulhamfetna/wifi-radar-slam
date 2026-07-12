# Paper 3 · Sub-project 2 — the radar credibility anchor

**Status: 🔴 GATE NOT PASSED — work in progress, paused 2026-07-12.**
**Branch:** `paper3-sub2-anchor` (not merged, not tagged — correctly, since the gate has not passed).

This file is the running record of the credibility gate. It is deliberately written *before* the
gate passes, because the debugging trail is itself a result: it says precise, quantified things
about why point-based ICP struggles on spinning radar.

---

## The setup

| | |
|---|---|
| Data | **Boreas** `boreas-2020-11-26-13-58`, 2,500 scans, **5,008 m**, 4 Hz Navtech spinning radar |
| Why Boreas, not Oxford | Boreas is anonymous public HTTPS; Oxford requires a registration that cannot be automated |
| Back-end | `lidar/slam_icp.run_lidar_slam` — **unchanged**, as the whole argument requires |
| Front-end | k-strongest per azimuth (CFEAR-class) |
| Scoring | KITTI protocol, standard 100–800 m lengths (valid: the sequence is km-scale) |
| Ground truth | `applanix/radar_poses.csv`, joined to scans **by filename** (`GPSTime` *is* the PNG name) |

**Yaw convention verified, not assumed:** mean |heading − course of travel| = **1.5°**. Boreas
`heading` is the maths convention. This was checked first, precisely because sub-project 1 was
burned three times by assumed conventions.

## Cited SOTA — with the caveats that make them honest

| Method | Drift | Dataset | Caveat |
|---|---|---|---|
| **CFEAR** | **1.09 %** | Oxford | The **tuned** figure (**1.16 %** untuned). Radar-only, point-based. **Our real reference point.** |
| **DRO** | 0.26 % | Boreas | **Gyro-aided**, and a *direct-intensity* method that extracts no points at all. **Not an apples-to-apples bound for us**, and must never be presented as one. |

---

## Gate runs so far — all FAILED

| Run | Front-end | Drift (trans) | Rot | Verdict |
|---|---|---|---|---|
| 1 | k=12, 100 m, no range NMS | 57.4 % | 21.4 °/100 m | FAIL |
| 2 | k=12, 100 m, **range NMS** | 88.1 % | 15.9 °/100 m | FAIL |
| 3 | **k=40, 50 m**, range NMS | **84.9 %** | 15.7 °/100 m | FAIL |

Thresholds (fixed in advance, never moved): **< 5 % PASS · 5–10 % MARGINAL · > 10 % FAIL.**

---

## What the debugging established — three real findings, each measured

### 1. The k-strongest front-end was picking bins, not targets *(fixed)*

A radar target is **extended**: a wall lights up a run of adjacent range bins. So the "k strongest
bins" were, overwhelmingly, k samples of **one** target. Measured on real data: the 12 picks in an
azimuth spanned a median of **0.7 m**, and **96 % of consecutive picks sat < 0.15 m apart**. A
nominal 4,800-point cloud carried only ~400 independent measurements, each smeared into a short
**radial streak** — and point-to-point ICP slides along such streaks almost for free.

Handed the *exact* frame-to-frame motion as its starting guess, ICP still converged **0.62 m** away
from it, on a 2 m step. Enforcing 1 m of range separation (non-maximum suppression) cut that to
**0.13 m**. Fixed in `radar/kstrongest.py` (`min_separation_m`), with a regression test.

### 2. Radar's noise is ANISOTROPIC, and that is what wrecks yaw

Range is accurate (0.06 m) but **cross-range error grows with range**: the 0.9° beam gives ±1.6 m
of tangential error at 100 m. **Yaw is estimated from tangential displacement — exactly the noisy
direction — and point-to-point ICP weights every direction equally.**

Measured per-frame yaw error (scan-to-scan, moving frames):

| front-end | yaw err std | implied random-walk drift |
|---|---|---|
| k=12, 100 m | **5.35°** | ~38 °/100 m |
| k=12, 50 m | 3.99° | ~28 °/100 m |
| **k=40, 50 m** | **0.46°** | **~3 °/100 m** |
| k=40, 30 m | 0.52° | ~4 °/100 m |

More returns per azimuth average the noise down; cropping range removes the worst of it. **This is
precisely why CFEAR uses a point-to-LINE metric on oriented surface points** rather than
point-to-point — it projects error onto surface normals and so ignores the noisy tangential
component.

### 3. Scan-to-scan WORKS. Scan-to-map is what fails.

| registration | per-frame translation error |
|---|---|
| scan-to-**scan**, GT init | 0.31 m |
| scan-to-**scan**, constant-velocity init | **0.24 m** (the motion model is fine — exonerated) |
| scan-to-**map** (what the back-end does) | **1.22 m** |

And the accumulated map *actively hurts*, monotonically — measured at k=12/100 m:

| map | per-frame error |
|---|---|
| unbounded global (current back-end) | 1.22 m |
| 50-frame window | 0.86 m |
| 25-frame window | 0.57 m |
| **10-frame window** | **0.43 m** |

Radar's cross-range error means each frame's far returns land in slightly different places, so an
unbounded map that never forgets **smears into a cloud**; the back-end's first-point-wins voxels
never average it out. CFEAR registers against a **sliding local map**, not a global one.

### Things investigated and EXONERATED (recorded so nobody re-runs them)

- **The noise floor / `z_min`.** The prime suspect, and wrong: the 12 picks have median power 72–78
  against a noise floor of 9 (p99 = 64). **0 % of picks are near noise.**
- **Motion compensation.** Neither translational nor full translation+rotation compensation changes
  the yaw error (std 4.87 / 4.87 / 4.82°). *(Kept anyway — it is physically correct, and the
  249 ms sweep at 15 m/s really does smear a scan by 3.7 m.)*
- **The yaw convention.** Verified at 1.5°.
- **The motion model.** Constant-velocity init is as good as GT init (0.24 vs 0.31 m).

---

## ⏭ Where to resume

**The open contradiction:** scan-to-scan yaw error is now **0.46°** (k=40/50 m) — good enough for
~3 °/100 m — yet the full scan-to-**map** SLAM still drifts 85 %. The window sweep that showed the
global map is the culprit was run with the **old, bad front-end (k=12/100 m)**. It must be redone
with the corrected one.

**Next step, concretely:** re-run the map-window sweep (`window ∈ {1, 3, 5, 10, 20}`) with
**k=40, max_range 50 m, NMS on**, over the full 2,500 frames, and report drift for each. Diagnostic
scripts are in the session scratchpad (`win_sweep.py`, `yaw_diag.py`, `mc_diag.py`).

**Then the decision, which is the user's to make:**

- **(a)** If a bounded local-map window fixes it → add `map_window` to `run_lidar_slam` as an
  **optional parameter defaulting to `None` (= today's exact behaviour, so paper 2 stays
  bit-identical)**, and use a finite window for **every sensor** in paper 3. That keeps the
  "one back-end for every sensor" argument fully intact — the constraint is *identical across
  sensors*, not *frozen forever*.
- **(b)** If it does not → the honest finding is that **point-to-point ICP is the wrong registration
  metric for spinning radar** (finding 2 above says why, quantitatively). The options are then to
  implement a point-to-line metric (a real back-end change, applied to all sensors), or to accept a
  MARGINAL/FAIL verdict and **bound the paper's claims**: radar would be *understated*, making any
  WiFi-vs-radar gap we report a **lower bound**.

**Do not tag or merge this branch until the gate passes or the paper's claims are explicitly
bounded.** A FAIL is a real outcome, not something to tune around.
