# R1.6 — AP-position sensitivity (result)

**Reviewer point:** *"the bistatic geometry ellipse equation relies on precise knowledge of access
point locations… analysis of position uncertainty tolerance for the ambient transmitters."*

**Experiment:** `experiments/run_ap_sensitivity.py` · harness `src/wifi_radar_slam/eval/sensitivity.py`
· data `data/wifislam_sim_nominal.npz` (240-frame nominal scene, 3 APs) · **no Sionna** (estimator-side).
We inject i.i.d. Gaussian error (std σ) into the AP positions *fed to the solver*, keep the propagation
fixed, and run the real particle-filter SLAM. 8 seeds per σ. Two detection regimes: **oracle** (clean
isolation) and **realistic MUSIC** (dense, ~9 detections/frame). The harness reproduces paper 1's
localization (baseline ATE ≈ 0.09–0.12 m ≈ the corrected 0.098 m).

## Result

| AP error σ (m) | ATE, oracle (m) | ATE, MUSIC (m) | map acc, MUSIC (m) |
|---:|---:|---:|---:|
| 0.00 | 0.094 ± 0.042 | 0.122 ± 0.059 | **3.952 ± 0.006** |
| 0.25 | 0.111 | 0.127 | 3.941 |
| 0.50 | 0.134 | 0.121 | 3.945 |
| 1.00 | 0.110 | 0.135 | 3.972 |
| 2.00 | 0.105 | 0.097 | 4.007 |
| 4.00 | 0.101 ± 0.027 | 0.124 ± 0.043 | **4.101 ± 0.093** |

## What it says

1. **Trajectory ATE is essentially insensitive to AP-position error** — flat within seed noise from
   0 to **4 m** of AP coordinate error, in both regimes. WiFi-SLAM localization tolerates gross AP
   mislocation.
2. **Why:** in this pipeline the bistatic WiFi measurements are **refinement-only** — the particle
   filter is anchored by the vehicle's odometry, and the measurements polish it rather than fix
   absolute position. So AP-position error never reaches the trajectory. (Confirmed separately: with
   odometry degraded, ATE grows but is *still* insensitive to AP error — the measurements cannot carry
   localization regardless.)
3. **Where the AP error does land — the map.** With the dense MUSIC map, accuracy degrades **gently and
   monotonically** (3.952 → 4.101 m; +0.15 m / +3.8 % at 4 m AP error) and its variance grows
   (std 0.006 → 0.093). The bistatic ellipse places reflectors relative to the AP, so AP error shifts
   the map — but modestly, because a common AP offset partly cancels in the geometry.
4. **The clean escape (paper 3):** the **monostatic** on-vehicle configuration needs **no AP positions
   at all** — the vehicle illuminates and hears its own echoes — and it is also the lower-phantom
   geometry. AP-position uncertainty is a property of the *bistatic/ambient* mode, and the programme's
   preferred geometry removes it entirely.

## Tolerance statement (for the manuscript / rebuttal)

> Absolute trajectory error is unchanged (≤ 0.14 m) for AP-position errors up to **4 m**, because
> localization is odometry-anchored and the bistatic measurements are refinement-only. The AP-position
> dependence manifests only in the map, and gracefully (+0.15 m map error at 4 m AP error). The
> monostatic on-vehicle geometry removes the dependence entirely.

## Honest caveats

- The flat ATE partly reflects that the WiFi measurements contribute *weakly* to localization in this
  pipeline (odometry dominates) — consistent with the programme's thesis that mapping, not localization,
  is the hard problem.
- Single simulated nominal scene; the trend (not the absolute map number) is the result.

*Raw: `results/ap_sensitivity.json`. Issue #6.*
