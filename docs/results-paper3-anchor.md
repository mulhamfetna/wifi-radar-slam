# Paper 3 · Sub-project 2 — the radar credibility anchor

**Status: 🔴 GATE FAILED — and the root cause is architectural, not a bug.**
**Branch:** `paper3-sub2-anchor` — deliberately **not merged, not tagged**.

The gate did its job. It was built to answer one question before we invested in the ablation —
*is our shared back-end a credible radar baseline?* — and the answer is **no**, for a reason that
is now precisely characterised.

---

## The finding

> **Our shared point-to-point scan-to-map ICP back-end cannot estimate rotation from spinning-radar
> point clouds — at all. The registration cost is FLAT in yaw.**

This is not a tuning problem, not a convergence problem, and not an initialisation problem. Six
independent hypotheses were tested and killed. The evidence:

**1. The cost function does not have its minimum at the true yaw.** Sweeping yaw across ±8° around
the truth on the sharpest turning frames (true yaw change ~11°/frame), holding translation fixed:

| frame | true Δyaw | cost at truth | cost at its minimum | where the minimum actually is |
|---|---|---|---|---|
| 151 | +11.90° | 0.3933 | 0.3850 | **−6.0°** from truth |
| 150 | +11.69° | 0.3966 | 0.3886 | **+6.0°** |
| 149 | +11.20° | 0.3900 | 0.3839 | **+8.0°** |
| 147 | +10.93° | 0.3890 | 0.3789 | **−8.0°** |
| 148 | +10.91° | 0.3895 | 0.3858 | **+2.0°** |
| 146 | +10.85° | 0.3755 | 0.3723 | **−8.0°** |

The cost varies by ~2% over a ±8° span and its minimum sits at **random** offsets. There is no
rotational signal to descend.

**2. ICP therefore recovers essentially ZERO rotation.** Correlation between the yaw *error* and the
*true* yaw change: **−0.992**. The estimate simply stays wherever the initial guess put it. (True
per-frame yaw change: std 2.26°. Yaw error: std 2.42°. Nearly identical — the definition of
recovering nothing.)

**3. It is not the point density.** Contrast (cost at ±6° ÷ cost at truth; > 1.3 would mean a real
minimum) measured across the whole front-end range:

| front-end | pts/scan | contrast | |
|---|---|---|---|
| k=2 | 800 | 0.94 | FLAT |
| k=4 | 1,600 | 0.96 | FLAT |
| k=12 | 4,800 | 1.03 | FLAT |
| k=40 | 16,000 | 1.01 | FLAT |
| k=12, 3 m sep | 4,797 | 1.03 | FLAT |
| k=4, 5 m sep | 1,598 | 0.97 | FLAT |

**Every density is flat.**

---

## Hypotheses tested and KILLED (recorded so nobody re-runs them)

| # | Hypothesis | Verdict | Evidence |
|---|---|---|---|
| 1 | The noise floor — `z_min=0` admits noise | **WRONG** | The 12 picks have median power 72–78 vs a noise floor of 9 (p99 = 64). **0 %** of picks are near noise. |
| 2 | Motion distortion (249 ms sweep) | **WRONG** | Translation-only *and* full translation+rotation compensation leave yaw error unchanged (4.87 / 4.87 / 4.82°). |
| 3 | The yaw convention | **WRONG** | Verified: mean \|heading − course of travel\| = **1.5°**. |
| 4 | The motion model | **WRONG** | Constant-velocity init ≈ GT init on translation. Extrapolating yaw makes it **worse** (3.34° vs 1.87°) — the yaw rate is too noisy. |
| 5 | The accumulated map | **PARTLY** — but not the cause | With the old front-end, a shorter window helped (1.22 → 0.43 m). With the corrected front-end, *no* window helps: global 84.9 %, window=1 59.3 %, window=10 83.4 %. |
| 6 | The registration **metric** (point-to-line, as CFEAR uses) | **WRONG as I implemented it** | Yaw std 2.55° vs 2.41° — no help. *Because* local PCA normals on a cloud strung along 400 fixed azimuth rays return the **ray** direction, not the surface normal. |

### One real bug WAS found and fixed along the way

**k-strongest was picking bins, not targets.** A radar target is *extended*: a wall lights up a run
of adjacent range bins, so the "12 strongest bins" were 12 samples of **one** wall (median spread
0.7 m; 96 % of consecutive picks < 0.15 m apart). The 4,800-point cloud held only ~400 independent
measurements, each a short **radial streak**. Fixed with range non-maximum suppression
(`min_separation_m`), with a regression test. It improved translation markedly — but it did not
touch rotation, because rotation was never there to begin with.

### ⚠️ A measurement trap I fell into, recorded honestly

I initially concluded that `k=40, 50 m` was much better because it cut the yaw error from 5.35° to
0.46° **when ICP was given a perfect initial guess**. That conclusion was **wrong**, and the reason
matters: a flatter cost makes ICP *move less*, so a method that returns its input unchanged scores
perfectly on a perfect-init test. **I was measuring stillness, not accuracy.** The commit that set
`K=40, MAX_RANGE_M=50` rests on that flawed measurement and should be revisited, not trusted.

---

## Why radar breaks point-to-point ICP (the mechanism)

Radar's noise is **anisotropic**: range is accurate (0.06 m) but cross-range error grows with range
— a 0.9° beam gives ±1.6 m of tangential error at 100 m. **Rotation is read from tangential
displacement — exactly the noisy direction** — and point-to-point ICP weights every direction
equally. On top of that, the returns lie along **400 fixed azimuth rays**, identical in every scan,
so nearest-neighbour correspondences have no distinctive structure to lock onto.

This is precisely why **CFEAR** — the SOTA baseline — does none of what we do. It compresses each
scan into a few hundred **oriented surface points** (grid cells → mean + covariance → a normal),
and registers **point-to-line** on *those*. The normals are what make yaw observable. My
point-to-line attempt failed because I computed normals on the raw ray-strung cloud, where PCA
returns the ray direction, not the surface.

---

## Gate runs (thresholds fixed in advance: < 5 % PASS · 5–10 % MARGINAL · > 10 % FAIL)

| Run | Front-end | Drift | Rot | Verdict |
|---|---|---|---|---|
| 1 | k=12, 100 m, no NMS | 57.4 % | 21.4 °/100 m | FAIL |
| 2 | k=12, 100 m, NMS | 88.1 % | 15.9 °/100 m | FAIL |
| 3 | k=40, 50 m, NMS | 84.9 % | 15.7 °/100 m | FAIL |

Setup: **Boreas** `boreas-2020-11-26-13-58`, 2,500 scans, **5,008 m**, 4 Hz Navtech. GT joined to
scans by filename. Yaw convention verified (1.5°). Back-end used **unchanged**, as the argument
requires.

**Cited SOTA, with the caveats that keep them honest:** CFEAR **1.09 %** (IEEE T-RO 39(2), 2023 —
the *tuned* figure; 1.16 % untuned; Oxford, not Boreas). DRO **0.26 %** (arXiv 2504.20339, Boreas)
— but **gyro-aided** and *direct-intensity*, therefore **not** an apples-to-apples bound for a
point-based radar-only method like ours.

---

## What this means for the paper — the decision is the user's

The gate has done exactly what it was built to do: it stopped us **before** the ablation was built
on sand. Three ways forward.

**A. Implement CFEAR-class registration** (oriented surfels + point-to-line) as an option in the
shared back-end, applied identically to every sensor. Substantial work — effectively a sub-project.
Keeps the paper exactly as designed, and would very likely improve LiDAR and WiFi too. The
"one back-end for every sensor" argument survives intact, so long as the default stays
point-to-point so paper 2 remains bit-identical.

**B. Restructure paper 3 around what the substrate can actually do.** Note that **RQ1 — the
headline — does not need SLAM at all.** "Is the ≈89 % phantom ceiling universal to RF sensing?" is
answered from the detection chain against ground-truth geometry, using GT poses. So are the map
metrics and most of the 2×2 ablation. Only **RQ3 (head-to-head SLAM accuracy)** requires a working
radar odometry. Paper 3 could report RQ1/RQ2/RQ4 in full and either drop RQ3 or state plainly that
our shared point-based back-end cannot do radar odometry — **which is itself a defensible, cited
finding**, and is exactly why CFEAR exists.

**C. Report the FAIL and bound the claims** — keep RQ3 but state that radar is *understated* by our
back-end, making any WiFi-vs-radar SLAM gap a **lower bound**. Weakest option: a reviewer who knows
the radar literature will ask why we did not simply use a point-to-line metric.

**Recommendation: B, possibly with A later.** B is honest, preserves the paper's actual headline,
and costs nothing we have not already built. A is the ambitious path and is a real contribution to
the shared substrate, but it is a sub-project in its own right and should be chosen deliberately,
not slipped in to rescue a gate.
