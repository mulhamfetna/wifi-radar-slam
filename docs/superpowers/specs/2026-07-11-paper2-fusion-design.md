# Paper 2, sub-project 3 — WiFi + LiDAR fusion (RQ4)

**Date:** 2026-07-11
**Status:** approved (brainstorming), pending execution
**Paper:** 2 — *WiFi sensing as a drop-in LiDAR replacement for SLAM*
**Integration branch:** `paper2-wifi-vs-lidar`

## Scope

Answer **RQ4**: does running WiFi and LiDAR **side by side** improve accuracy/coverage
**significantly, marginally, or not at all** — and (novel) **is the gain worth paying for
both sensors**? Produces fused SLAM results on the same two scenes, same six metrics, same
GT, plus a cost-normalized verdict that ties RQ4 to the RQ5 cost model.

**NOT** in scope: deep-learning enhancement (RQ2, next sub-project); re-deriving the WiFi or
LiDAR baselines (they exist); new sensor models.

## Why not the literature's architecture

Every prior WiFi+LiDAR system found in the review (`docs/literature-paper2.md`) uses **WiFi
to *assist* LiDAR** — WiFi supplies loop closure / global constraints while LiDAR does the
primary ranging and mapping. **Our RQ3 result inverts that premise:** realistic WiFi
localizes to 2.7 cm (better than both our LiDAR models on the controlled scene) while LiDAR
is the only modality that maps. Demoting WiFi to loop closure would waste its measured
strength. So we fuse **symmetrically** and let the data decide.

## Design

### Tight fusion (primary) — one back-end, both measurements

Extend the WiFi particle filter (`slam/particle_filter.run_slam`) so each particle's weight
is the **product of two independent likelihoods** (independent sensors ⇒ product):

- `w_wifi` — bistatic reprojection consistency of the WiFi detections (the existing
  `_reproject_bistatic` error term).
- `w_lidar` — **scan match**: transform the LiDAR scan into world coordinates at that
  particle's pose, take the mean nearest-neighbour distance `d` to the accumulated map
  (KD-tree), likelihood `exp(-0.5 · d² / σ_lidar²)`.

**Map = union** of (a) WiFi-triangulated bistatic reflectors and (b) LiDAR scan points,
voxel-deduplicated. This is what makes fusion able to fix WiFi's coverage hole.

**Tractability:** the per-particle likelihood uses a **subsampled scan** (default 100
points, `rng`-drawn); the **full** scan feeds map accumulation. Without this, 200 particles
× ~1–3 k scan points × N frames is needlessly expensive.

### Loose fusion (baseline) — deliberately naive

Run the two SLAMs independently, then fuse **outputs**: trajectory = **equal-weight
average**; map = **union** (voxel-dedup). It is naive *on purpose* — its role is to answer
"does tight coupling actually beat blind combination?" Equal weights (not
performance-weighted) because weighting by measured ATE would leak ground truth.

### Third reference — "oracle best-of-each" (no new code)

WiFi's trajectory + LiDAR's map, i.e. the best row of each from the existing RQ3 table.
Reported as an upper bound on *naive* combination and explicitly labelled **oracle-selected**
(it needs GT to know which is best, so it is a reference, not a deployable system).

## Comparison matrix

For **each scene** (`controlled_wall`, `street_canyon_metal`) × **each LiDAR model**
(A geometric, B Sionna diffuse — keeping the envelope):

| Row | Source |
|-----|--------|
| WiFi-only (realistic commodity CSI) | existing RQ3 |
| LiDAR-only | existing RQ3 |
| **Fused — tight** | new |
| **Fused — loose** | new |
| *oracle best-of-each* | table arithmetic, no run |

All six metrics (ATE, RPE, Chamfer, map-acc, map-completeness, IoU), same GT.

**WiFi input is the realistic (joint-MUSIC, commodity-CSI) case** — the deployable one, and
the one whose mapping collapses (IoU ≈ 0). That is precisely where fusion must prove itself.
A `street_canyon_metal` + realistic-MUSIC config does not exist yet and will be added.

## Cost-normalized fusion (the novel twist — ties RQ4 → RQ5)

Fusion means **buying both sensors**. Using `cost.py`, price the fused system at
`wifi_package + lidar_tier` and recompute the RQ5 value metrics:
- `$·m` (price × ATE) — localization value
- `$/IoU` (price ÷ IoU) — mapping value

Then the RQ4 verdict is not just "fusion is more accurate" but **"fusion is / is not worth
its price."** Since WiFi is $40–95 and the LiDAR we model is $8–24 k, the fused price is
dominated by LiDAR — so fusion must deliver a *large* gain over LiDAR-only to justify
itself. No prior fusion work does this analysis.

## Architecture

```
src/wifi_radar_slam/fusion.py     # run_fused_slam (tight) + fuse_loose (baseline)
experiments/run_fusion.py         # both scenes x LiDAR A/B x {tight, loose} -> metrics
configs/street_metal_music.yaml   # NEW: street scene + realistic joint-MUSIC WiFi
tests/test_fusion.py              # pure-Python unit tests (no Sionna)
```

`fusion.py` exposes:
- `run_fused_slam(detections, scans, ap_positions, velocity, timestep_s, rng,
  n_particles=200, init_pose=None, map_min_support=1, map_min_excess_m=0.0,
  sigma_lidar=0.5, scan_subsample=100, voxel=0.5) -> (est_traj, est_map)`
- `fuse_loose(wifi_traj, wifi_map, lidar_traj, lidar_map, voxel=0.5) -> (traj, map)`

Both return the same `(est_traj (n,3), est_map (M,2))` contract as the existing back-ends,
so `eval/metrics.py` scores them unchanged and the results slot straight into the RQ3 table.

## Data flow

```
scene ──┬── WiFi CSI ──► MUSIC front-end ──► detections (path_len, aoa, ap)
        └── LiDAR sensor (A or B) ────────► scans
                     │
                     ▼
      run_fused_slam: particle weight = w_wifi(bistatic) x w_lidar(scan-match)
                      map = WiFi reflectors  ∪  LiDAR points
                     │
                     ▼        (and, separately, fuse_loose on the two solo runs)
        six metrics vs the SAME GT  ──►  RQ4 table  ──►  cost-normalized verdict
```

## Error handling / honesty guards

- **Empty modality frames.** If a frame has no WiFi detections, the WiFi likelihood is
  skipped (weights untouched) and the LiDAR term alone drives that update — and vice versa.
  Fusion must degrade gracefully to whichever sensor is present, not crash.
- **Don't rig the balance.** `sigma_lidar` is a documented, fixed parameter — not tuned per
  scene to make fusion look good. If fusion only wins under a hand-tuned balance, that is
  itself the finding and gets reported.
- **A negative result is a real result.** If fusion adds little (or hurts), we report that
  plainly — the honest answer to "significantly, marginally, or not at all" may well be
  "marginally", and the cost overlay may say "not worth it".
- **Loose baseline uses equal weights**, never GT-derived weights.

## Testing

Pure-Python unit tests (`tests/test_fusion.py`, no Sionna, in the default suite):
- `run_fused_slam` recovers a known straight trajectory from synthetic WiFi detections +
  synthetic LiDAR scans of a box scene (ATE below tolerance).
- The fused map **contains points from both modalities** (union is real, not LiDAR-only).
- Graceful degradation: a frame with zero WiFi detections, and a frame with an empty scan,
  both run without error and still track.
- `fuse_loose` averages trajectories and unions maps on known inputs.
Scene-level fused runs are Sionna-gated (server), like models A/B.

## Non-goals

- No DL (RQ2). No new sensor models. No pose-graph/loop-closure variant (explicitly rejected
  above — it demotes WiFi and merely reproduces prior art).
- No hyper-parameter search to make fusion win.

## Acceptance

- `fusion.py` passes local unit tests (tight + loose + graceful degradation + union).
- `experiments/run_fusion.py` produces, for both scenes × LiDAR A/B, the six metrics for
  fused-tight and fused-loose, saved to `data/fusion_results.json`.
- `docs/results-paper2.md` gains a **"Fusion (RQ4)"** section: the comparison matrix, the
  significant/marginal/none verdict, and the **cost-normalized** "is it worth paying for
  both?" answer.
- Full test suite green.
