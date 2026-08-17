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

1. **Trajectory ATE is insensitive to AP-position error — structurally.** ATE is flat within seed
   noise not just to 4 m but all the way to **32 m** of AP error (larger than the scene itself), in
   both regimes. This is not a lucky range: the particle filter **requires a motion model**, so it is
   always **odometry-anchored**, and the bistatic measurements are **refinement-only** — they polish a
   good estimate but cannot set absolute position. AP-position error therefore *cannot* reach the
   trajectory in this architecture. (Confirmed: degrading the odometry inflates ATE uniformly but it is
   *still* insensitive to AP error — the measurements cannot carry localization regardless of density.)
2. **Where the AP error does land — the map, cleanly and monotonically.** The bistatic ellipse places
   each reflector relative to its AP, so AP error shifts the map. Across the full range the MUSIC map
   metrics degrade smoothly and roughly linearly (see the extended table): map accuracy, Chamfer and
   completeness each worsen ~**+18 %** from σ = 0 to 32 m. This is the sensitivity the reviewer asked
   about — and it is graceful, not a cliff.
3. **The clean escape (paper 3):** the **monostatic** on-vehicle configuration needs **no AP positions
   at all** — the vehicle illuminates and hears its own echoes — and it is also the lower-phantom
   geometry. AP-position uncertainty is a property of the *bistatic/ambient* mode; the programme's
   preferred geometry removes it entirely.

### Extended break-point (realistic MUSIC, 6 seeds) — the map degradation curve

| AP error σ (m) | ATE (m) | map acc (m) | Chamfer (m) | completeness (m) |
|---:|---:|---:|---:|---:|
| 0  | 0.095 | 3.95 | 6.64 | 9.33 |
| 4  | 0.110 | 4.11 | 6.82 | 9.53 |
| 8  | 0.153 | 4.21 | 6.98 | 9.75 |
| 16 | 0.118 | 4.30 | 7.23 | 10.16 |
| 32 | 0.100 | 4.65 | 7.82 | 10.98 |

**ATE never trends; the three map metrics rise monotonically.** The trajectory is immune; the map
degrades gracefully.

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
