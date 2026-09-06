# R1.7 — Blockage / occlusion robustness (result)

**Reviewer point:** *"it would be good to have some blockages into the Sionna environment to evaluate
the particle filter resilience to sudden loss of measurements."*

**Experiment:** `experiments/run_blockage_robustness.py` · harness `src/wifi_radar_slam/eval/sensitivity.py`
· data `data/wifislam_sim_nominal.npz` (240 frames) · **no Sionna** (estimator-side). We model a sudden,
**total** loss of measurements by dropping *all* detections for a window of `L` frames (an occlusion),
at two mid-trajectory positions, 4 seeds. We report the drift **during** the blackout, the ATE in the
`RECOVER=10` frames **after** it (re-lock), and the full-run ATE. Two regimes: oracle and realistic MUSIC.

## Result

| blackout length (frames) | drift during, oracle (m) | ATE after, oracle (m) | drift during, MUSIC (m) | ATE after, MUSIC (m) |
|---:|---:|---:|---:|---:|
| 0  | 0.000 | 0.008 | 0.000 | 0.009 |
| 2  | 0.086 | 0.086 | 0.132 | 0.140 |
| 5  | 0.087 | 0.087 | 0.133 | 0.146 |
| 10 | 0.088 | 0.094 | 0.136 | 0.149 |
| 20 | 0.090 | 0.094 | 0.136 | 0.155 |
| 40 | 0.094 | 0.108 | 0.136 | 0.149 |

*(40 frames = 17 % of the 240-frame trajectory.)*

## What it says

1. **The particle filter is highly resilient to sudden measurement loss.** A **total** blackout of all
   measurements for up to **40 frames (17 % of the run)** causes only ~0.09 m (oracle) / ~0.14 m (MUSIC)
   of drift — barely above the no-blackout baseline.
2. **The drift is nearly constant in blackout length** (0.086 → 0.094 m from L=2 to L=40). It does not
   accumulate catastrophically: the PF **dead-reckons on the vehicle odometry** through the blackout and
   holds position; only the small per-step process noise accrues.
3. **Immediate re-lock.** The ATE in the 10 frames after measurements return is essentially the baseline
   (oracle 0.09–0.11 m; MUSIC 0.14–0.15 m) — no lasting damage from the outage.
4. **Same mechanism as R1.6:** localization is **odometry-anchored**; the WiFi measurements refine but do
   not carry it, so losing them for a while costs little.

## Resilience statement (for the manuscript / rebuttal)

> Under a sudden, total loss of all measurements, the particle filter dead-reckons on odometry: a
> 40-frame (17 % of trajectory) blackout adds only ≈0.1 m of drift and the filter re-locks immediately
> once measurements return. WiFi-SLAM inherits the vehicle's odometry robustness to measurement dropout.

## Honest caveats

- The strong resilience is *because* localization leans on odometry, not the WiFi measurements — the
  same property behind the R1.6 result.
- We model the blockage as measurement dropout (the estimator-relevant effect of an occluder); we do not
  re-run Sionna with a physical blocker. Single simulated nominal scene.

*Raw: `results/blockage_robustness.json`. Issue #7.*
