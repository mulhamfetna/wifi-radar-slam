# Paper 2, sub-project 4 — Learned map enhancement (RQ2)

**Date:** 2026-07-11
**Status:** approved (brainstorming), pending execution
**Paper:** 2 — *WiFi sensing as a drop-in LiDAR replacement for SLAM*
**Integration branch:** `paper2-wifi-vs-lidar`

## Scope

Answer **RQ2**: *is pure WiFi enough, or is deep-learning enhancement needed to reach
LiDAR-equivalent results?* Concretely: can a learned **path discriminator in the mapping
loop** close the WiFi mapping-coverage gap (realistic-CSI **IoU ≈ 0**) and approach LiDAR
(IoU 0.15–1.0)?

**NOT** in scope: end-to-end CSI→occupancy networks (deferred — see Non-goals); changing the
WiFi localization path; new sensor models.

## Why this attack

Paper 1 **proved the root cause and proved it is learnable** — realistic mapping is floored
by **path discrimination**, not by bandwidth or aperture (60 GHz + 16 antennas moved map
accuracy 4.8 m → 4.75 m), and a RandomForest classifies mapping-useful paths at F1 ≈ 0.9.
But paper 1 **never closed the loop**: the discriminator was never plugged back into the
mapping pipeline. This sub-project closes it and measures whether the map actually improves.

## Critical correction: the paper-1 discriminator uses an oracle feature

`discriminate.path_features` uses
`[range_m, excess_m, abs_azimuth, elevation, aoa_dev_from_ap]`.
Our realistic front-end is a **single ULA running 2-D (delay–azimuth) MUSIC** — it estimates
**azimuth only** and **never estimates elevation**. So `elevation` is a feature a commodity
2-D CSI receiver **cannot measure**, and paper 1's F1 ≈ 0.9 is therefore an **optimistic
upper bound**.

**Consequence (load-bearing):** the in-the-loop discriminator must be **retrained on only
MUSIC-observable features**. Reusing paper-1's five-feature model would leak an oracle
quantity into inference. We expect a *lower* F1 than 0.9 and will report it honestly as a
correction to paper 1's headline.

**MUSIC-observable feature set (4):** `path_len` (from the MUSIC delay), `excess`
(`path_len − |AP − pose|`), `abs_azimuth` (`|aoa|`), `aoa_dev` (angular deviation of the
arrival azimuth from the bearing to the AP). No elevation, no interaction type, no bounce
count.

## Labels: operational, not oracle-metadata

Paper 1 labelled paths by oracle metadata (`n_bounce == 1 & !is_floor`). But at inference we
do not have paths — we have **MUSIC detections**. So we label what we actually care about:

> A detection is **useful** iff the reflector it triangulates to lands **within τ = 1.0 m of
> a true facade** (the scene's ground-truth footprint map).

This is the operationally meaningful target (it *is* the definition of a good map point), it
needs no path matching, and **GT is used only at training time** — never at inference.

## The ladder (this is what answers RQ2)

Run the full progression and let it decide whether deep learning is *needed*:

| Rung | Filter | Machinery |
|------|--------|-----------|
| 0 | none (pure WiFi) | baseline — realistic-CSI IoU ≈ 0 |
| 1 | **physics heuristic** | the existing bistatic min-excess gate (`map_min_excess_m`) |
| 2 | **classical ML** | RandomForest on the 4 MUSIC-observable features |
| 3 | **neural net** | small MLP (sklearn `MLPClassifier`) on the same 4 features |

Target to approach: LiDAR IoU (0.149–1.000) and the fused result.

**If rung 1 or 2 already closes the gap, the honest answer to RQ2 is "deep learning is NOT
needed"** — a genuinely publishable negative result, and the reason we run the ladder rather
than assuming a network is required.

Model choice note: `MLPClassifier` (sklearn, already an optional dep — no new heavy
dependency) is a *shallow* network. If it too fails to close the gap, the honest conclusion
is that a deeper model **on raw CSI** (not on 4 hand-made features) would be the next step —
stated as future work, not claimed as a result.

## Filter the map, not the localization

WiFi localization already works (ATE 0.027–0.081). So the filter gates **only which
detections enter the map**; every detection still contributes to particle weighting. This
isolates the mapping improvement and cannot damage the one thing WiFi is already good at.

Implementation: `run_slam` gains an optional `map_filter` callable. When present, a detection
whose features the filter rejects still updates the particle weights but is **not** appended
to `mapped_points`. Default `None` ⇒ existing behaviour, so paper-1 results are unchanged.

## Train/test discipline (no leakage)

Two splits, both reported:
1. **Held-out frames** (temporal): train on the first 60 % of frames, test on the last 40 %
   of the same scene. Optimistic but leakage-free.
2. **Cross-scene** (generalization stress): train on `controlled_wall`, test on
   `street_canyon_metal`, and vice versa. This is the honest test of whether the learned
   discriminator generalizes or merely memorizes a scene.

If cross-scene collapses, that is the finding — a scene-specific discriminator is not a
deployable enhancement, and we say so.

## Architecture

```
src/wifi_radar_slam/map_filter.py   # music_features, label_from_gt, filters (heuristic/RF/MLP)
src/wifi_radar_slam/slam/particle_filter.py   # + optional `map_filter` arg (default None)
experiments/train_map_filter.py     # build (X,y), train RF + MLP, report F1 (both splits)
experiments/run_enhanced_map.py     # rebuild maps for rungs 0-3, emit the six metrics
tests/test_map_filter.py            # pure-Python unit tests
```

`map_filter.py` exposes:
- `music_features(dets, pose, ap_positions) -> np.ndarray (k, 4)` — MUSIC-observable only.
- `label_from_gt(dets, pose, ap_positions, gt_xy, tol=1.0) -> np.ndarray (k,)` — 1 iff the
  triangulated reflector lands within `tol` of a GT facade point (invalid triangulations → 0).
- `HeuristicFilter(min_excess_m)`, `SklearnFilter(model)` — both expose
  `__call__(X) -> np.ndarray[bool]` so `run_slam` treats them identically.

## Data flow

```
CSI ──► MUSIC ──► detections (path_len, aoa, ap)
                      │
      ┌───────────────┴───────────────┐
      │ (all detections)              │ (features)
      ▼                               ▼
 particle weighting            map_filter ──► keep / drop
 (localization UNCHANGED)             │
                                      ▼
                        triangulate ──► map ──► six metrics vs GT
```

## Honesty guards

- **No oracle features at inference.** Elevation is excluded. Report the corrected (lower)
  F1 against paper-1's 0.9, and state plainly that paper-1's figure assumed a feature a 2-D
  commodity receiver cannot measure.
- **GT only at training.** Labels use the GT map; inference uses none.
- **A negative result is a result.** "The heuristic suffices" or "even the NN cannot close
  the gap" are both legitimate, publishable RQ2 answers. Do not tune to manufacture a win.
- **Report coverage cost.** Filtering removes detections, so map *completeness* may fall even
  as *accuracy*/IoU rise. Report all six metrics, not just the flattering ones.
- **Cross-scene result is reported even if it collapses.**

## Testing

Pure-Python unit tests (no Sionna, default suite):
- `music_features` returns the 4 expected columns and contains **no elevation**.
- `label_from_gt` labels a detection whose reflector lies on a GT wall as 1, and one that
  triangulates far away (or fails) as 0.
- `HeuristicFilter` drops low-excess (LOS/floor-like) detections and keeps a genuine detour.
- `run_slam(map_filter=...)` shrinks the map versus `map_filter=None` while leaving the
  trajectory unchanged (proving the filter touches mapping only, not localization).

## Non-goals

- No end-to-end CSI→occupancy network (deferred: it needs a real training pipeline and risks
  learning the simulator rather than the physics — stated as future work).
- No change to the WiFi localization path, sensor models, or fusion.
- No hyper-parameter search to force a win.

## Acceptance

- `map_filter.py` + the `run_slam(map_filter=...)` hook pass local unit tests; existing
  paper-1 behaviour is unchanged when `map_filter=None`.
- `train_map_filter.py` reports F1 for RF and MLP on **both** splits (held-out frames and
  cross-scene), on MUSIC-observable features only.
- `run_enhanced_map.py` produces the six metrics for rungs 0–3 on both scenes →
  `data/enhanced_map_results.json`.
- `docs/results-paper2.md` gains an **"Enhancement (RQ2)"** section stating explicitly
  whether deep learning is **needed**, whether the gap **closes** to LiDAR parity, and the
  corrected discriminator F1 (with the elevation caveat against paper 1).
- Full test suite green.
