# Paper 3 · Sub-project 3 — THE ABLATION (RQ1, RQ2, RQ3)

> **For agentic workers:** REQUIRED SUB-SKILL: Use superpowers:subagent-driven-development (recommended) or superpowers:executing-plans to implement this plan task-by-task. Steps use checkbox (`- [ ]`) syntax for tracking.

**Goal:** Answer the paper's headline — *is the ≈89 % phantom ceiling a WiFi pathology or a property of RF sensing?* — by measuring the phantom rate and map quality of five sensing cells that differ **only** in physics, and decomposing radar's advantage into **bandwidth vs carrier vs geometry**.

**Architecture:** Five cells share **one** detection chain (`radar/processing.py`, already built and tested) and are scored **under ground-truth poses** — no estimator in the loop for any sensor. Each cell differs only in (carrier, bandwidth, geometry, transmitter). A new bistatic sensor supplies cell A; the existing monostatic sensor supplies B/C/D; papers 1–2's joint-MUSIC pipeline supplies the reference row. A new `eval/phantom.py` implements paper 2's phantom definition against the ray tracer's *true* paths.

**Tech Stack:** Python 3, NumPy, SciPy, Sionna RT 2.0.1 (server-only), pytest.

**Test runner:** `.venv/bin/python -m pytest` (a bare `python3` has no venv).

**Branch:** `paper3-sub3-ablation`, off `paper3-wifi-vs-radar`.

**Spec:** `docs/superpowers/specs/2026-07-12-paper3-wifi-vs-radar-design.md` (as revised 2026-07-13).

---

## Global Constraints

1. **The detection chain is held FIXED across every cell.** `beat_matrix → range_fft → azimuth_beamform → cfar_2d → cluster_detections`. Nothing about the *algorithm* may vary between cells — only the physics. This is the entire basis for attributing differences to bandwidth/carrier/geometry.
2. **Scored under GROUND-TRUTH poses.** No SLAM, no ICP, no particle filter, for *any* cell. Sub-project 2 proved our point-based back-end cannot estimate rotation from radar (the yaw cost is flat; `corr(yaw err, true Δyaw) = −0.992`), and shipping a crippled radar-SLAM row would have flattered WiFi. Removing the estimator entirely isolates the **sensor** more cleanly than SLAM ever did.
3. **CFAR is the primary front-end.** It defines the phantom rate, because a *calibrated* detection threshold is what makes "this detection matches no real path" a meaningful statement. `k_strongest` exists for SLAM, and SLAM is out of scope here.
4. **Do NOT tune WiFi to manufacture parity.** Radar is *expected* to win. We design to **explain** the gap, not to close it. Equally: do not cripple WiFi — cell A uses **all three** APs, because a real ambient deployment has many free illuminators, and that advantage is genuinely part of its geometry.
5. **A null RQ1 is publishable.** If radar's phantom rate turns out low, the ceiling is WiFi-specific — a clean, useful finding that the ablation then explains.
6. **Sionna is server-only** (`amd`, `/home/dev/mulham/wifi-radar-slam`, throttled `nice -n 19 ionice -c3`). Pure parts are unit-tested locally; Sionna sensors are not instantiated in tests.
7. **Server runs need progress logging with ETA** and must use all 32 cores.

---

## Sionna facts — measured on the server, not assumed (`docs/results-paper3-radar-substrate.md`)

- `paths.a` is a **TUPLE** `(real, imag)` of `(n_rx, n_rx_ant, n_tx, n_tx_ant, n_paths)` tensors.
- `paths.tau` / `phi_r` / `valid` are `(n_rx, n_tx, n_paths)`; `objects` / `interactions` are `(depth, n_rx, n_tx, n_paths)`.
- **`n_tx` is 4**: the scene's 3 WiFi APs *plus* our `radar_tx` (index 3). **Indexing the wrong transmitter silently mixes bistatic AP paths into a monostatic radar's ray set.**
- The co-located TX/RX **angle convention is NOT mirrored** (settled on an asymmetric scene).
- **Diffuse scattering is mandatory**: 4 valid paths specular vs **883,957** diffuse.
- ITU `marble`/`brick` are undefined at 77 GHz but unused — `radar.sensor.retune_scene` freezes them.
- **61 % of rays are ground bounces** and are dropped (`_touches_floor`), as paper 2's LiDAR does.

---

## The load-bearing insight: cell A can use the SAME chain

Cell A is *passive, bistatic* WiFi — there is no FMCW chirp. So how can it share an FMCW chain?

**Because a beat signal and an OFDM CSI vector are the same measurement.** An FMCW sweep sampled at `N` instants across bandwidth `B` measures the channel at `N` frequencies spanning `B`. An OFDM CSI vector across `N` subcarriers spanning `B` measures *exactly the same thing*. A Fourier transform of either yields the **delay profile**. The chain is therefore not merely analogous — it is identical, and `beat_matrix` serves cell A verbatim, given the **bistatic** delays.

Two things *do* differ, and they **are the geometry ablation**:

| | monostatic (B, C, D) | bistatic (A) |
|---|---|---|
| what the delay means | round trip: `τ = 2R/c` | **path length**: `τ = (\|AP→R\| + \|R→veh\|)/c` |
| detection → world | plain polar → Cartesian | **ellipse solve** (AP and vehicle are the foci) |

**Consequence — a trap that must be handled explicitly.** `RadarConfig.range_bins()` assumes the monostatic convention (`τ = 2R/c`), so on bistatic delays the chain reports **half the true path length**. That conversion (`path_len = 2 × reported_range`) must live in a **named, tested function**, never inline. Getting it wrong halves every WiFi range and would look like a stunning physics result.

---

## The phantom definition — and the one methodological choice that must not be fudged

Paper 2's definition (from `experiments/isolate_mapping_floor.py`, which produced the 89 %): for each detection, find the nearest **true Sionna path** of the same transmitter in normalised (range, azimuth) space —

```
cost = hypot( Δrange / 3.0 m , Δazimuth / 10° )     →   cost > 3.0  ⇒  PHANTOM
```

A phantom is a detection that **corresponds to no real propagation path**.

**The trap.** A *fixed* absolute tolerance (3 m) lets a high-resolution sensor win for a trivial reason: cell D resolves to 0.0375 m, so its detections naturally land closer to true paths than cell A's, whose resolution is 0.94 m. Reporting only the fixed tolerance would let radar "beat" WiFi on phantom rate **by construction** — the exact kind of rigged comparison this paper exists to avoid.

**So we report BOTH, always, side by side:**

| tolerance | question it answers |
|---|---|
| **Fixed (3 m / 10°)** | Directly comparable to **paper 2's 89 %**. The headline continuity number. |
| **Resolution-scaled (3 × the cell's own range resolution)** | *"Is this detection explainable by a real path, given what this sensor can actually resolve?"* — separates **phantom** from merely **coarse**. |

If radar's advantage survives *both*, it is real. If it only appears under the fixed tolerance, it is an artifact of resolution and must be said so. **Neither number is allowed to appear without the other.**

---

## File Structure

| File | Responsibility |
|---|---|
| `src/wifi_radar_slam/eval/phantom.py` | Paper-2's phantom definition, generalised over any detection source. Pure. |
| `src/wifi_radar_slam/eval/mapping.py` | `map_under_gt_poses(scans, poses, voxel)` — accumulate a map with **no estimator**. Pure. |
| `src/wifi_radar_slam/radar/truth.py` | Extract the ray tracer's **true paths** per frame per transmitter (the ground truth the phantom rate is measured against). Sionna-touching; lazy import. |
| `src/wifi_radar_slam/radar/sensor_bistatic.py` | Cell A: `bistatic_path_len(reported_range)`, `bistatic_to_world(...)` (the **ellipse solve**), and `SionnaBistaticSensor`. |
| `src/wifi_radar_slam/radar/cells.py` | The 5-cell registry — the single place the ablation's physics is defined. |
| `experiments/run_ablation.py` | The runner: 5 cells × 2 scenes × seeds → phantom rate + map metrics, under GT poses. |
| `docs/results-paper3-ablation.md` | The decomposition: bandwidth vs carrier vs geometry. |
| `tests/test_phantom.py`, `tests/test_mapping.py`, `tests/test_sensor_bistatic.py`, `tests/test_cells.py` | Unit tests. |

---

### Task 0: Branch

- [ ] **Step 1: Cut the branch**

```bash
cd /mnt/data/projects/wifi-radar
git checkout paper3-wifi-vs-radar
git pull --ff-only
git checkout -b paper3-sub3-ablation
.venv/bin/python -m pytest -q
```

Expected: 190 passed.

---

### Task 1: The phantom-rate measurement (RQ1 — the headline)

**Files:**
- Create: `src/wifi_radar_slam/eval/phantom.py`
- Test: `tests/test_phantom.py`

**Interfaces:**
- Consumes: nothing (pure NumPy).
- Produces:
  - `match_detections(det_range_m, det_azimuth_rad, true_range_m, true_azimuth_rad, range_scale_m=3.0, azimuth_scale_deg=10.0, max_cost=3.0) -> np.ndarray` — index of the matched true path per detection, or `-1` for a phantom.
  - `phantom_stats(det_range_m, det_azimuth_rad, true_range_m, true_azimuth_rad, range_scale_m=3.0, ...) -> dict` with keys `n_detections`, `n_phantoms`, `phantom_rate`, `range_bias_m` (median signed Δrange over *matched* detections), `abs_range_err_m`, `azimuth_err_deg`.

**Note on units.** `det_range_m` and `true_range_m` must be in the **same** quantity. For monostatic cells that is the round-trip range `τc/2`; for the bistatic cell it is the **path length** `τc`. Task 3 converts; this module just compares like with like, and the tests pin that contract.

- [ ] **Step 1: Write the failing test**

Create `tests/test_phantom.py`:

```python
import numpy as np
import pytest
from wifi_radar_slam.eval.phantom import match_detections, phantom_stats


def test_a_detection_on_top_of_a_true_path_matches_it():
    m = match_detections(np.array([20.0]), np.array([0.1]),
                         np.array([20.0, 50.0]), np.array([0.1, 1.0]))
    assert m.tolist() == [0]


def test_a_detection_far_from_every_true_path_is_a_PHANTOM():
    # THE headline measurement. 3 m range scale, 10 deg azimuth scale, cost cap 3.0 ->
    # anything beyond ~9 m / ~30 deg of every real path corresponds to no propagation path.
    m = match_detections(np.array([20.0]), np.array([0.0]),
                         np.array([80.0]), np.array([0.0]))
    assert m.tolist() == [-1]


def test_the_match_is_the_NEAREST_true_path_in_normalised_space():
    m = match_detections(np.array([21.0]), np.array([0.0]),
                         np.array([25.0, 20.0]), np.array([0.0, 0.0]))
    assert m.tolist() == [1]                       # 20 m is nearer than 25 m


def test_range_and_azimuth_are_traded_off_by_their_own_scales():
    # 3 m of range error costs the same as 10 deg of azimuth error, by construction.
    far_in_range = match_detections(np.array([23.0]), np.array([0.0]),
                                    np.array([20.0]), np.array([0.0]))
    far_in_angle = match_detections(np.array([20.0]), np.array([np.deg2rad(10.0)]),
                                    np.array([20.0]), np.array([0.0]))
    assert far_in_range.tolist() == [0] and far_in_angle.tolist() == [0]
    # but 4x either one is beyond the cost cap
    assert match_detections(np.array([32.0]), np.array([0.0]),
                            np.array([20.0]), np.array([0.0])).tolist() == [-1]


def test_azimuth_difference_wraps():
    m = match_detections(np.array([20.0]), np.array([np.deg2rad(179.0)]),
                         np.array([20.0]), np.array([np.deg2rad(-179.0)]))
    assert m.tolist() == [0]                       # 2 deg apart, not 358


def test_phantom_stats_counts_and_rate():
    det_r = np.array([20.0, 21.0, 90.0])           # two real, one phantom
    det_a = np.array([0.0, 0.0, 0.0])
    tr_r = np.array([20.0, 21.0])
    tr_a = np.array([0.0, 0.0])
    s = phantom_stats(det_r, det_a, tr_r, tr_a)
    assert s["n_detections"] == 3
    assert s["n_phantoms"] == 1
    assert s["phantom_rate"] == pytest.approx(1 / 3)


def test_range_bias_is_the_median_SIGNED_error_over_MATCHED_detections():
    # Paper 2 reported a 6.45 m median range BIAS -- a systematic offset, not scatter --
    # so the sign must survive. Phantoms are excluded: they have no true path to be
    # biased against, and including them would measure nothing.
    det_r = np.array([22.0, 23.0, 200.0])          # +2, +3 on real paths; one phantom
    det_a = np.zeros(3)
    tr_r = np.array([20.0, 20.0])
    tr_a = np.zeros(2)
    s = phantom_stats(det_r, det_a, tr_r, tr_a)
    assert s["n_phantoms"] == 1
    assert s["range_bias_m"] == pytest.approx(2.5)          # median of [+2, +3]


def test_resolution_scaled_tolerance_is_stricter_for_a_fine_sensor():
    # A 0.5 m detection error is well inside a 3 m fixed tolerance, but OUTSIDE a
    # resolution-scaled tolerance for a sensor that resolves to 0.0375 m (cell D).
    # Reporting only the fixed tolerance would let a high-resolution sensor look
    # phantom-free by construction -- which is exactly the rigged comparison we refuse.
    det_r, det_a = np.array([20.5]), np.array([0.0])
    tr_r, tr_a = np.array([20.0]), np.array([0.0])
    loose = phantom_stats(det_r, det_a, tr_r, tr_a, range_scale_m=3.0)
    tight = phantom_stats(det_r, det_a, tr_r, tr_a, range_scale_m=3 * 0.0375)
    assert loose["phantom_rate"] == 0.0
    assert tight["phantom_rate"] == 1.0


def test_no_true_paths_means_every_detection_is_a_phantom():
    s = phantom_stats(np.array([10.0, 20.0]), np.zeros(2),
                      np.empty(0), np.empty(0))
    assert s["phantom_rate"] == 1.0
    assert np.isnan(s["range_bias_m"])


def test_no_detections_gives_nan_rate_not_zero():
    # An empty detection set has NO phantom rate. Reporting 0 % would read as "perfect".
    s = phantom_stats(np.empty(0), np.empty(0), np.array([10.0]), np.array([0.0]))
    assert s["n_detections"] == 0
    assert np.isnan(s["phantom_rate"])
```

- [ ] **Step 2: Run the test and watch it fail**

```bash
.venv/bin/python -m pytest tests/test_phantom.py -q
```

Expected: `ModuleNotFoundError: No module named 'wifi_radar_slam.eval.phantom'`.

- [ ] **Step 3: Implement**

Create `src/wifi_radar_slam/eval/phantom.py`:

```python
"""The phantom rate -- paper 3's headline measurement (RQ1).

A PHANTOM is a detection that corresponds to NO real propagation path. This is paper 2's
definition, kept identical so paper 3's numbers sit beside its ~89 % without a footnote:
match each detection to the nearest TRUE ray-traced path of the same transmitter in
normalised (range, azimuth) space, and call it a phantom if nothing is close enough.

    cost = hypot( dRange / range_scale , dAzimuth / azimuth_scale )
    cost > max_cost  ->  PHANTOM

THE ONE METHODOLOGICAL CHOICE THAT MUST NOT BE FUDGED. A *fixed* absolute tolerance (3 m)
would let a high-resolution sensor win for a trivial reason: cell D resolves to 0.0375 m, so
its detections naturally land closer to true paths than cell A's, which resolves to 0.94 m.
Reporting only the fixed tolerance would hand radar a lower phantom rate BY CONSTRUCTION.

So every result reports BOTH:
  * fixed (3 m / 10 deg)                -- directly comparable to paper 2's 89 %
  * resolution-scaled (3 x the cell's own range resolution) -- "is this detection explainable
    by a real path, GIVEN WHAT THIS SENSOR CAN RESOLVE?", which separates *phantom* from
    merely *coarse*
Neither number may be reported without the other.

UNITS: det_range_m and true_range_m must be the SAME quantity -- round-trip range for a
monostatic cell, bistatic PATH LENGTH for cell A. This module compares like with like and
does not convert; the caller does.
"""
from __future__ import annotations
import numpy as np


def match_detections(det_range_m, det_azimuth_rad, true_range_m, true_azimuth_rad,
                     range_scale_m: float = 3.0, azimuth_scale_deg: float = 10.0,
                     max_cost: float = 3.0) -> np.ndarray:
    """Index of the nearest true path per detection, or -1 where none is plausible.

    Returns an int array of length len(det_range_m). -1 means PHANTOM.
    """
    d_r = np.asarray(det_range_m, dtype=float).ravel()
    d_a = np.asarray(det_azimuth_rad, dtype=float).ravel()
    t_r = np.asarray(true_range_m, dtype=float).ravel()
    t_a = np.asarray(true_azimuth_rad, dtype=float).ravel()
    if d_r.size != d_a.size:
        raise ValueError(f"{d_r.size} detection ranges but {d_a.size} azimuths")
    if t_r.size != t_a.size:
        raise ValueError(f"{t_r.size} true ranges but {t_a.size} azimuths")
    if d_r.size == 0:
        return np.empty(0, dtype=int)
    if t_r.size == 0:
        return np.full(d_r.size, -1, dtype=int)          # nothing real to match: all phantom

    az_scale = np.deg2rad(azimuth_scale_deg)
    dr = (t_r[None, :] - d_r[:, None]) / range_scale_m
    # wrapped angular difference -- 179 deg and -179 deg are 2 deg apart, not 358
    da = np.angle(np.exp(1j * (t_a[None, :] - d_a[:, None]))) / az_scale
    cost = np.hypot(dr, da)                              # (n_det, n_true)

    j = np.argmin(cost, axis=1)
    best = cost[np.arange(d_r.size), j]
    return np.where(best <= max_cost, j, -1).astype(int)


def phantom_stats(det_range_m, det_azimuth_rad, true_range_m, true_azimuth_rad,
                  range_scale_m: float = 3.0, azimuth_scale_deg: float = 10.0,
                  max_cost: float = 3.0) -> dict:
    """Phantom rate + the range bias, for one cell.

    Returns {n_detections, n_phantoms, phantom_rate, range_bias_m, abs_range_err_m,
             azimuth_err_deg}.

    `range_bias_m` is the median **signed** range error over MATCHED detections. The sign
    matters: paper 2's headline second number was a 6.45 m median range BIAS -- a systematic
    offset, far beyond the 0.94 m resolution limit -- not mere scatter. Phantoms are excluded
    from it, because a detection with no true path has nothing to be biased against.

    With no detections at all the rate is **NaN, not 0** -- a sensor that detects nothing is
    not phantom-free, it is blind, and 0 % would read as perfection.
    """
    d_r = np.asarray(det_range_m, dtype=float).ravel()
    d_a = np.asarray(det_azimuth_rad, dtype=float).ravel()
    t_r = np.asarray(true_range_m, dtype=float).ravel()
    t_a = np.asarray(true_azimuth_rad, dtype=float).ravel()

    if d_r.size == 0:
        return {"n_detections": 0, "n_phantoms": 0, "phantom_rate": float("nan"),
                "range_bias_m": float("nan"), "abs_range_err_m": float("nan"),
                "azimuth_err_deg": float("nan")}

    m = match_detections(d_r, d_a, t_r, t_a, range_scale_m, azimuth_scale_deg, max_cost)
    hit = m >= 0
    n_ph = int((~hit).sum())

    if not hit.any():
        return {"n_detections": int(d_r.size), "n_phantoms": n_ph, "phantom_rate": 1.0,
                "range_bias_m": float("nan"), "abs_range_err_m": float("nan"),
                "azimuth_err_deg": float("nan")}

    j = m[hit]
    d_rng = d_r[hit] - t_r[j]                            # SIGNED: the bias must keep its sign
    d_az = np.angle(np.exp(1j * (d_a[hit] - t_a[j])))
    return {
        "n_detections": int(d_r.size),
        "n_phantoms": n_ph,
        "phantom_rate": float(n_ph / d_r.size),
        "range_bias_m": float(np.median(d_rng)),
        "abs_range_err_m": float(np.median(np.abs(d_rng))),
        "azimuth_err_deg": float(np.median(np.abs(np.rad2deg(d_az)))),
    }
```

- [ ] **Step 4: Run the tests and watch them pass**

```bash
.venv/bin/python -m pytest tests/test_phantom.py -q
```

Expected: 10 passed.

- [ ] **Step 5: Commit**

```bash
git add src/wifi_radar_slam/eval/phantom.py tests/test_phantom.py
git commit -m "paper3(eval): the phantom rate -- RQ1's headline measurement

Paper 2's definition, kept identical so paper 3's numbers sit beside its ~89% without a
footnote: a phantom is a detection matching NO true ray-traced path, in normalised
(range, azimuth) space.

Reports BOTH a fixed tolerance (comparable to paper 2) and a RESOLUTION-SCALED one. That is
not decoration: a fixed 3 m tolerance would hand radar a lower phantom rate BY CONSTRUCTION,
since cell D resolves to 0.0375 m and cell A to 0.94 m. Neither number may be reported alone.

Range bias keeps its SIGN (paper 2's second headline was a 6.45 m systematic offset, not
scatter), and an empty detection set yields NaN, not 0 -- a blind sensor is not phantom-free."
```

---

### Task 2: Mapping under ground-truth poses (no estimator)

**Files:**
- Create: `src/wifi_radar_slam/eval/mapping.py`
- Test: `tests/test_mapping.py`

**Interfaces:**
- Consumes: `lidar.pointcloud.Scan`.
- Produces: `map_under_gt_poses(scans, poses, voxel=0.5) -> np.ndarray` — an `(M, 2)` world-frame map.

- [ ] **Step 1: Write the failing test**

Create `tests/test_mapping.py`:

```python
import numpy as np
from wifi_radar_slam.eval.mapping import map_under_gt_poses
from wifi_radar_slam.lidar.pointcloud import Scan


def test_a_single_scan_at_the_origin_maps_to_itself():
    m = map_under_gt_poses([Scan(np.array([[10.0, 0.0]]))], np.array([[0.0, 0.0, 0.0]]),
                           voxel=0.5)
    assert np.allclose(m, [[10.0, 0.0]])


def test_the_pose_is_applied_rotation_then_translation():
    scan = Scan(np.array([[10.0, 0.0]]))                    # 10 m ahead, sensor-local
    pose = np.array([[5.0, 5.0, np.pi / 2]])                # at (5,5), facing +y
    m = map_under_gt_poses([scan], pose, voxel=0.1)
    assert np.allclose(m, [[5.0, 15.0]], atol=1e-9)         # 10 m along +y from (5,5)


def test_scans_from_several_poses_accumulate_into_one_world_map():
    scans = [Scan(np.array([[10.0, 0.0]])), Scan(np.array([[10.0, 0.0]]))]
    poses = np.array([[0.0, 0.0, 0.0], [50.0, 0.0, 0.0]])
    m = map_under_gt_poses(scans, poses, voxel=0.5)
    got = {tuple(np.round(p, 3)) for p in m}
    assert got == {(10.0, 0.0), (60.0, 0.0)}


def test_voxel_downsampling_collapses_duplicates():
    scans = [Scan(np.array([[10.0, 0.0], [10.1, 0.05]]))]   # same 1 m cell
    m = map_under_gt_poses(scans, np.array([[0.0, 0.0, 0.0]]), voxel=1.0)
    assert m.shape[0] == 1


def test_empty_scans_give_an_empty_map():
    m = map_under_gt_poses([Scan.empty(), Scan.empty()],
                           np.array([[0.0, 0.0, 0.0], [1.0, 0.0, 0.0]]), voxel=0.5)
    assert m.shape == (0, 2)


def test_mismatched_lengths_are_rejected():
    import pytest
    with pytest.raises(ValueError):
        map_under_gt_poses([Scan.empty()], np.array([[0.0, 0.0, 0.0], [1.0, 0.0, 0.0]]))
```

- [ ] **Step 2: Run the test and watch it fail**

```bash
.venv/bin/python -m pytest tests/test_mapping.py -q
```

Expected: `ModuleNotFoundError: No module named 'wifi_radar_slam.eval.mapping'`.

- [ ] **Step 3: Implement**

Create `src/wifi_radar_slam/eval/mapping.py`:

```python
"""Accumulate a world map from scans placed at GROUND-TRUTH poses -- no estimator.

Paper 3 scores every cell this way, deliberately. Sub-project 2 established that our shared
point-based back-end cannot estimate rotation from spinning radar at all (the registration
cost is flat in yaw), so any SLAM-based comparison would have handed radar an artificially
weak trajectory and flattered WiFi by contrast.

Removing the estimator entirely is not a retreat -- it is a STRONGER experiment. With no pose
error in the loop for anyone, every difference between cells is attributable to the sensor's
own physics, which is precisely what the ablation is for.
"""
from __future__ import annotations
import numpy as np


def map_under_gt_poses(scans, poses, voxel: float = 0.5) -> np.ndarray:
    """Place each scan at its true pose and accumulate a voxel-thinned world map.

    Args:
        scans: list of Scan (sensor-local points, +x forward).
        poses: (n, 3) ground-truth (x, y, yaw).
        voxel: cell size (m); one point survives per cell.

    Returns an (M, 2) world-frame map.
    """
    poses = np.asarray(poses, dtype=float)
    if len(scans) != poses.shape[0]:
        raise ValueError(f"{len(scans)} scans but {poses.shape[0]} poses")

    cells: dict[tuple[int, int], np.ndarray] = {}
    for scan, pose in zip(scans, poses):
        if len(scan) == 0:
            continue
        for p in scan.to_world(pose):                    # rotate by yaw, then translate
            cells.setdefault((int(round(p[0] / voxel)), int(round(p[1] / voxel))), p)
    return np.array(list(cells.values())) if cells else np.empty((0, 2))
```

- [ ] **Step 4: Run the tests and watch them pass**

```bash
.venv/bin/python -m pytest tests/test_mapping.py -q
```

Expected: 6 passed.

- [ ] **Step 5: Commit**

```bash
git add src/wifi_radar_slam/eval/mapping.py tests/test_mapping.py
git commit -m "paper3(eval): map under ground-truth poses -- no estimator in the loop

Sub-project 2 proved our point-based back-end cannot estimate rotation from radar, so a
SLAM-based comparison would have given radar an artificially weak trajectory and flattered
WiFi. Removing the estimator entirely is a STRONGER experiment: with no pose error for
anyone, every difference between cells is attributable to the sensor's physics."
```

---

### Task 3: Cell A — the bistatic sensor and the ellipse solve

**Files:**
- Create: `src/wifi_radar_slam/radar/sensor_bistatic.py`
- Test: `tests/test_sensor_bistatic.py`

**Interfaces:**
- Consumes: `radar.processing.radar_scan` pieces, `slam.particle_filter._triangulate_bistatic` (existing, tested: `_triangulate_bistatic(pose_xy, ap_xy, path_len, aoa, min_excess_m=0.0) -> np.ndarray | None`), `radar.config.WIFI_5G2_160M`.
- Produces:
  - `bistatic_path_len(reported_range_m) -> float | np.ndarray` — undo the chain's monostatic assumption.
  - `bistatic_detections_to_world(ranges, azimuths_local, pose, ap_xy) -> np.ndarray` — the **ellipse solve**; returns `(M, 2)` world points (drops unsolvable ones).
  - `SionnaBistaticSensor(built, cfg, rng, ...)` — callable `(pose) -> (Scan, det_pathlen, det_azimuth_world, ap_index)`.

- [ ] **Step 1: Write the failing test**

Create `tests/test_sensor_bistatic.py`:

```python
import numpy as np
import pytest
from wifi_radar_slam.radar.sensor_bistatic import (bistatic_path_len,
                                                   bistatic_detections_to_world)


def test_the_chain_reports_HALF_the_bistatic_path_length():
    # THE trap. RadarConfig.range_bins() assumes the MONOSTATIC convention (tau = 2R/c), so
    # fed a bistatic delay it reports half the true path length. Getting this wrong halves
    # every WiFi range and would look like a stunning physics result.
    assert bistatic_path_len(25.0) == pytest.approx(50.0)
    assert np.allclose(bistatic_path_len(np.array([10.0, 30.0])), [20.0, 60.0])


def test_the_ellipse_solve_recovers_a_reflector_from_a_path_length_and_a_bearing():
    # Vehicle at the origin facing +x; AP at (0, 10). A reflector at (20, 0) gives
    #   |AP->R| = sqrt(20^2 + 10^2) = 22.36 ;  |R->veh| = 20  ->  path length 42.36 m
    # and the vehicle sees it at bearing 0 deg. The solve must return (20, 0).
    pose = np.array([0.0, 0.0, 0.0])
    ap = np.array([0.0, 10.0])
    L = np.hypot(20.0, 10.0) + 20.0
    w = bistatic_detections_to_world(np.array([L / 2.0]), np.array([0.0]), pose, ap)
    assert w.shape == (1, 2)
    assert np.allclose(w[0], [20.0, 0.0], atol=1e-6)


def test_the_ellipse_solve_uses_the_WORLD_bearing_so_yaw_is_applied():
    # Same geometry, but the vehicle is rotated 90 deg: the reflector now lies along the
    # vehicle's LOCAL -90 deg. The world answer must be unchanged.
    pose = np.array([0.0, 0.0, np.pi / 2])
    ap = np.array([0.0, 10.0])
    L = np.hypot(20.0, 10.0) + 20.0
    w = bistatic_detections_to_world(np.array([L / 2.0]), np.array([-np.pi / 2]), pose, ap)
    assert np.allclose(w[0], [20.0, 0.0], atol=1e-6)


def test_the_line_of_sight_path_is_rejected_not_mapped():
    # A path length equal to |AP - vehicle| is the DIRECT path: there is no reflector on it.
    # Mapping it would plant a phantom on top of the AP in every frame.
    pose = np.array([0.0, 0.0, 0.0])
    ap = np.array([30.0, 0.0])
    w = bistatic_detections_to_world(np.array([30.0 / 2.0]), np.array([0.0]), pose, ap)
    assert w.shape == (0, 2)


def test_no_detections_gives_an_empty_array():
    w = bistatic_detections_to_world(np.empty(0), np.empty(0),
                                     np.array([0.0, 0.0, 0.0]), np.array([0.0, 10.0]))
    assert w.shape == (0, 2)


def test_module_imports_without_sionna():
    import wifi_radar_slam.radar.sensor_bistatic as s
    assert hasattr(s, "SionnaBistaticSensor")
```

- [ ] **Step 2: Run the test and watch it fail**

```bash
.venv/bin/python -m pytest tests/test_sensor_bistatic.py -q
```

Expected: `ModuleNotFoundError: No module named 'wifi_radar_slam.radar.sensor_bistatic'`.

- [ ] **Step 3: Implement**

Create `src/wifi_radar_slam/radar/sensor_bistatic.py`:

```python
"""Cell A -- ambient BISTATIC WiFi, on the SAME detection chain as the radar cells.

WHY THE SAME CHAIN IS LEGITIMATE. Cell A is passive: there is no FMCW chirp. But a beat signal
and an OFDM CSI vector are the SAME measurement. An FMCW sweep sampled at N instants across
bandwidth B measures the channel at N frequencies spanning B; an OFDM CSI vector across N
subcarriers spanning B measures exactly the same thing. A Fourier transform of either yields
the delay profile. So `beat_matrix` serves cell A verbatim -- given BISTATIC delays.

WHAT DIFFERS IS THE GEOMETRY, AND THAT IS THE POINT (the A->B ablation):

  monostatic (B, C, D):  delay is a round trip, tau = 2R/c
                         detection -> world is a plain polar projection
  bistatic   (A):        delay is a PATH LENGTH, tau = (|AP->R| + |R->veh|)/c
                         detection -> world needs an ELLIPSE SOLVE, with the AP and the
                         vehicle at the foci, and its conditioning degrades with the
                         ellipse's eccentricity

That asymmetry is not an implementation detail -- it is the mechanism the ablation isolates,
and it is why paper 2's WiFi maps carried a 6.45 m range bias against a 0.94 m resolution
limit.

`SionnaBistaticSensor` lazily imports Sionna inside its methods, so this module imports fine
without it.
"""
from __future__ import annotations
import numpy as np

from ..geometry import RX_HEIGHT_M
from ..lidar.pointcloud import Scan
from ..slam.particle_filter import _triangulate_bistatic
from .processing import beat_matrix, range_fft, azimuth_beamform, cfar_2d, cluster_detections

C = 299792458.0


def bistatic_path_len(reported_range_m):
    """Undo the chain's MONOSTATIC range convention.

    `RadarConfig.range_bins()` maps a beat frequency to range assuming tau = 2R/c. Fed a
    bistatic delay tau = L/c, it therefore reports L/2. Multiply by two to recover the true
    path length.

    This is a one-line function on purpose: inlining it is exactly how one silently halves
    every WiFi range in the paper.
    """
    return 2.0 * np.asarray(reported_range_m, dtype=float)


def bistatic_detections_to_world(reported_ranges, azimuths_local, pose, ap_xy) -> np.ndarray:
    """The ELLIPSE SOLVE: (bistatic path length, bearing) -> reflector, in world coordinates.

    Args:
        reported_ranges: what the chain reported (HALF the path length -- see above).
        azimuths_local:  sensor-local bearings (rad).
        pose:            vehicle (x, y, yaw).
        ap_xy:           the illuminating AP's (x, y).

    Returns (M, 2) world points. Detections with no valid solution -- notably the DIRECT
    line-of-sight path, which has no reflector on it at all -- are dropped rather than mapped.

    Reuses `_triangulate_bistatic`, the same solve papers 1-2 used, so cell A is faithful to
    the prior work rather than a fresh re-derivation.
    """
    r = np.asarray(reported_ranges, dtype=float).ravel()
    a = np.asarray(azimuths_local, dtype=float).ravel()
    if r.size == 0:
        return np.empty((0, 2))
    pose = np.asarray(pose, dtype=float)
    veh_xy = pose[:2]
    yaw = float(pose[2]) if pose.size > 2 else 0.0

    lens = bistatic_path_len(r)
    out = []
    for L, az in zip(lens, a):
        world_az = float(np.arctan2(np.sin(az + yaw), np.cos(az + yaw)))
        R = _triangulate_bistatic(veh_xy, ap_xy, float(L), world_az)
        if R is not None:
            out.append(R)
    return np.array(out) if out else np.empty((0, 2))


class SionnaBistaticSensor:
    """Cell A: the scene's ambient WiFi APs illuminate; the vehicle receives.

    Emits, per frame, the detections the SHARED chain produces -- one pass per AP, pooled.
    All three APs are used: a real ambient deployment has several free illuminators, and that
    is a genuine part of WiFi's geometry. Crippling it to one AP to "match" the monostatic
    cells would be tuning WiFi down to make radar look better, which the spec forbids as
    firmly as it forbids tuning WiFi up.
    """

    def __init__(self, built, cfg, rng, max_depth: int = 3, scattering: float = 0.7):
        import sionna.rt as rt                       # lazy: server only
        from .sensor import retune_scene
        self.built, self.cfg, self.rng = built, cfg, rng
        self.max_depth = max_depth
        self.scene = built.scene
        self.frozen_materials = retune_scene(self.scene, cfg.carrier_hz)
        for m in self.scene.radio_materials.values():
            try:
                m.scattering_coefficient = scattering
            except Exception:
                pass
        self.solver = rt.PathSolver()
        self.rx = self.scene.receivers["veh"]
        # the APs are the scene's own transmitters -- everything that is NOT our radar TX
        names = list(self.scene.transmitters.keys())
        self.ap_idx = [i for i, n in enumerate(names) if n != "radar_tx"]
        self.ap_xy = [np.asarray(p, dtype=float)[:2] for p in built.ap_positions]
        self.floor_ids = {o.object_id for n, o in self.scene.objects.items()
                          if "floor" in n.lower()}

    def _solve(self, pose):
        import os
        import mitsuba as mi
        px, py = float(pose[0]), float(pose[1])
        self.rx.position = mi.Point3f(px, py, RX_HEIGHT_M)
        ns = int(os.environ.get("WRS_NUM_SAMPLES", "1000000"))
        return self.solver(self.scene, max_depth=self.max_depth, samples_per_src=ns,
                           diffuse_reflection=True,
                           seed=int(self.rng.integers(1, 2 ** 31 - 1)))

    def __call__(self, pose):
        """-> (world_points (M,2), det_pathlen (K,), det_azimuth_world (K,), ap_index (K,))

        The world points are the MAP contribution; the detection triples are what the phantom
        rate is measured on.
        """
        paths = self._solve(pose)
        tau_all = np.asarray(paths.tau.numpy())[0]              # (n_tx, n_paths)
        phi_all = np.asarray(paths.phi_r.numpy())[0]
        val_all = np.asarray(paths.valid.numpy())[0]
        re, im = paths.a
        a_all = (np.asarray(re.numpy())[0, 0, :, 0]
                 + 1j * np.asarray(im.numpy())[0, 0, :, 0])      # (n_tx, n_paths)
        objs = np.asarray(paths.objects.numpy())[:, 0]           # (depth, n_tx, n_paths)
        inter = np.asarray(paths.interactions.numpy())[:, 0]
        floor = np.any((inter != 0) & np.isin(objs, list(self.ap_idx and self.floor_ids)),
                       axis=0) if self.floor_ids else np.zeros_like(val_all, dtype=bool)

        yaw = float(pose[2]) if len(pose) > 2 else 0.0
        world, d_len, d_az, d_ap = [], [], [], []
        for k, t in enumerate(self.ap_idx):
            keep = val_all[t] & ~floor[t] & np.isfinite(tau_all[t]) & (np.abs(a_all[t]) > 0)
            tau = tau_all[t][keep]
            amp = a_all[t][keep]
            # Sionna's phi_r is a WORLD azimuth; the chain wants sensor-local
            az_local = np.angle(np.exp(1j * (phi_all[t][keep] - yaw)))
            if tau.size == 0:
                continue
            beat = beat_matrix(tau, amp, az_local, self.cfg, rng=self.rng)
            ra = azimuth_beamform(range_fft(beat, self.cfg), self.cfg)
            rng_m, az_m = cluster_detections(cfar_2d(ra, self.cfg), ra, self.cfg)
            if rng_m.size == 0:
                continue
            ap_xy = self.ap_xy[k] if k < len(self.ap_xy) else self.ap_xy[0]
            w = bistatic_detections_to_world(rng_m, az_m, pose, ap_xy)
            if w.size:
                world.append(w)
            d_len.append(bistatic_path_len(rng_m))
            d_az.append(np.angle(np.exp(1j * (az_m + yaw))))     # world azimuth
            d_ap.append(np.full(rng_m.size, k, dtype=int))

        W = np.concatenate(world) if world else np.empty((0, 2))
        L = np.concatenate(d_len) if d_len else np.empty(0)
        A = np.concatenate(d_az) if d_az else np.empty(0)
        P = np.concatenate(d_ap) if d_ap else np.empty(0, dtype=int)
        return W, L, A, P
```

- [ ] **Step 4: Run the tests and watch them pass**

```bash
.venv/bin/python -m pytest tests/test_sensor_bistatic.py -q
```

Expected: 6 passed.

If `test_the_ellipse_solve_recovers_a_reflector_from_a_path_length_and_a_bearing` fails, the
`bistatic_path_len` factor of 2 is the first suspect — a missing factor puts every reflector at
half its true distance, which is a plausible-looking wrong answer, the worst kind.

- [ ] **Step 5: Commit**

```bash
git add src/wifi_radar_slam/radar/sensor_bistatic.py tests/test_sensor_bistatic.py
git commit -m "paper3(radar): cell A -- ambient bistatic WiFi on the SAME detection chain

A beat signal and an OFDM CSI vector are the same measurement: both sample the channel at N
frequencies across bandwidth B, and a Fourier transform of either gives the delay profile. So
beat_matrix serves cell A verbatim, given BISTATIC delays -- which means the chain is held
genuinely fixed across every cell, not merely 'analogous'.

What differs is the GEOMETRY, and that IS the A->B ablation: a bistatic delay is a PATH LENGTH
(AP -> reflector -> vehicle), so the reflector needs an ELLIPSE solve rather than a polar
projection. The chain's range_bins() assumes the monostatic tau = 2R/c convention, so it
reports HALF the path length -- bistatic_path_len() undoes that, as a named and tested
function, because inlining it is exactly how one silently halves every WiFi range."
```

---

### Task 4: The true-path extractor (the ground truth RQ1 is measured against)

**Files:**
- Create: `src/wifi_radar_slam/radar/truth.py`
- Test: `tests/test_truth.py`

**Interfaces:**
- Consumes: nothing at import time (Sionna is lazy).
- Produces: `true_paths_for_tx(paths, tx_index, yaw, floor_ids, monostatic) -> dict` with keys `range_m` (round-trip range if `monostatic`, else path length), `azimuth_world_rad`, `n`. Pure given already-solved Sionna arrays — so it is unit-testable with fakes.

- [ ] **Step 1: Write the failing test**

Create `tests/test_truth.py`:

```python
import numpy as np
import pytest
from wifi_radar_slam.radar.truth import true_paths_for_tx

C = 299792458.0


class FakeTensor:
    def __init__(self, a):
        self._a = np.asarray(a)

    def numpy(self):
        return self._a


class FakePaths:
    """Mimics Sionna RT 2.0.1's real layouts, which were MEASURED on the server:
       tau/phi_r/valid  : (n_rx, n_tx, n_paths)
       objects/interactions : (depth, n_rx, n_tx, n_paths)
       a                : a TUPLE (real, imag) of (n_rx, n_rx_ant, n_tx, n_tx_ant, n_paths)
    """

    def __init__(self, tau, phi, valid, objects, inter):
        self.tau = FakeTensor(tau[None, ...])
        self.phi_r = FakeTensor(phi[None, ...])
        self.valid = FakeTensor(valid[None, ...])
        self.objects = FakeTensor(objects[:, None, ...])
        self.interactions = FakeTensor(inter[:, None, ...])


def make(tau, phi, valid, obj0, ninter):
    tau = np.array(tau, dtype=float)[None, :]              # (n_tx=1, n_paths)
    phi = np.array(phi, dtype=float)[None, :]
    valid = np.array(valid, dtype=bool)[None, :]
    n = tau.shape[1]
    objects = np.zeros((2, 1, n), dtype=int)
    objects[0, 0] = np.array(obj0, dtype=int)
    inter = np.zeros((2, 1, n), dtype=int)
    for i, k in enumerate(ninter):
        inter[:k, 0, i] = 1
    return FakePaths(tau, phi, valid, objects, inter)


def test_monostatic_range_is_the_ROUND_TRIP_half_of_tau_c():
    p = make([2 * 30.0 / C], [0.0], [True], [5], [1])
    t = true_paths_for_tx(p, 0, yaw=0.0, floor_ids=set(), monostatic=True)
    assert t["range_m"][0] == pytest.approx(30.0)


def test_bistatic_range_is_the_FULL_path_length():
    # The bistatic delay IS the path length AP->reflector->vehicle. Halving it here would
    # make every WiFi detection look like a phantom, which would rig RQ1 in radar's favour.
    p = make([60.0 / C], [0.0], [True], [5], [1])
    t = true_paths_for_tx(p, 0, yaw=0.0, floor_ids=set(), monostatic=False)
    assert t["range_m"][0] == pytest.approx(60.0)


def test_azimuth_is_returned_in_the_WORLD_frame():
    # Sionna's phi_r is already a world azimuth; `yaw` exists so callers can request the
    # sensor-local frame instead. Here we ask for world, so yaw must NOT be subtracted.
    p = make([2 * 30.0 / C], [np.deg2rad(40.0)], [True], [5], [1])
    t = true_paths_for_tx(p, 0, yaw=np.deg2rad(10.0), floor_ids=set(), monostatic=True)
    assert np.rad2deg(t["azimuth_world_rad"][0]) == pytest.approx(40.0)


def test_invalid_paths_are_dropped():
    p = make([2 * 30.0 / C, 2 * 40.0 / C], [0.0, 0.0], [True, False], [5, 5], [1, 1])
    t = true_paths_for_tx(p, 0, yaw=0.0, floor_ids=set(), monostatic=True)
    assert t["n"] == 1


def test_ground_bounce_paths_are_dropped():
    # 61% of a monostatic radar's rays hit the road. The ground-truth map holds building
    # footprints only, so a road return has nothing to be scored against -- and paper 2's
    # LiDAR dropped them the same way, which is what keeps the maps comparable.
    p = make([2 * 30.0 / C, 2 * 40.0 / C], [0.0, 0.0], [True, True], [7, 5], [1, 1])
    t = true_paths_for_tx(p, 0, yaw=0.0, floor_ids={7}, monostatic=True)
    assert t["n"] == 1
    assert t["range_m"][0] == pytest.approx(40.0)


def test_empty_path_set():
    p = make([], [], [], [], [])
    t = true_paths_for_tx(p, 0, yaw=0.0, floor_ids=set(), monostatic=True)
    assert t["n"] == 0
    assert t["range_m"].shape == (0,)
```

- [ ] **Step 2: Run the test and watch it fail**

```bash
.venv/bin/python -m pytest tests/test_truth.py -q
```

Expected: `ModuleNotFoundError: No module named 'wifi_radar_slam.radar.truth'`.

- [ ] **Step 3: Implement**

Create `src/wifi_radar_slam/radar/truth.py`:

```python
"""The ray tracer's TRUE paths -- the ground truth the phantom rate is measured against.

A phantom is "a detection matching no real propagation path", so this module defines what
counts as a real path. It is pure given already-solved Sionna arrays, which makes it
unit-testable with fakes and keeps Sionna out of the test path entirely.

The array layouts here were MEASURED on the server, not read off a wiki
(docs/results-paper3-radar-substrate.md):

    tau / phi_r / valid      : (n_rx, n_tx, n_paths)
    objects / interactions   : (depth, n_rx, n_tx, n_paths)

and n_tx is 4 -- the scene's three WiFi APs PLUS our radar_tx -- so indexing the wrong
transmitter silently mixes bistatic AP paths into a monostatic radar's ray set.
"""
from __future__ import annotations
import numpy as np

C = 299792458.0


def true_paths_for_tx(paths, tx_index: int, yaw: float, floor_ids,
                      monostatic: bool) -> dict:
    """True paths of one transmitter -> {range_m, azimuth_world_rad, n}.

    `range_m` is the quantity a detection of this geometry is comparable to:
      * monostatic -> the ROUND-TRIP range, tau*c/2
      * bistatic   -> the FULL path length, tau*c   (AP -> reflector -> vehicle)

    Halving the bistatic one would make every WiFi detection look like a phantom and rig RQ1
    in radar's favour, so the distinction is explicit and tested.

    Ground-bounce paths are dropped. 61 % of a monostatic radar's rays hit the road, and the
    ground-truth map contains building footprints only -- a road return has nothing to be
    scored against. Paper 2's LiDAR dropped them the same way, which is what keeps the maps
    comparable across papers.
    """
    tau = np.asarray(paths.tau.numpy())[0, tx_index]
    phi = np.asarray(paths.phi_r.numpy())[0, tx_index]
    valid = np.asarray(paths.valid.numpy())[0, tx_index].astype(bool)

    keep = valid & np.isfinite(tau) & np.isfinite(phi) & (tau > 0)
    if floor_ids:
        objs = np.asarray(paths.objects.numpy())[:, 0, tx_index]        # (depth, n_paths)
        inter = np.asarray(paths.interactions.numpy())[:, 0, tx_index]
        hits_floor = np.any((inter != 0) & np.isin(objs, list(floor_ids)), axis=0)
        keep &= ~hits_floor

    tau, phi = tau[keep], phi[keep]
    rng_m = tau * C / (2.0 if monostatic else 1.0)
    return {"range_m": rng_m, "azimuth_world_rad": phi, "n": int(rng_m.size)}
```

- [ ] **Step 4: Run the tests and watch them pass**

```bash
.venv/bin/python -m pytest tests/test_truth.py -q
```

Expected: 6 passed.

- [ ] **Step 5: Commit**

```bash
git add src/wifi_radar_slam/radar/truth.py tests/test_truth.py
git commit -m "paper3(radar): the true-path extractor -- the ground truth RQ1 is measured against

A phantom is 'a detection matching no real propagation path', so this defines what a real
path IS. Monostatic paths yield a ROUND-TRIP range (tau*c/2); bistatic paths yield the FULL
path length (tau*c). Halving the bistatic one would make every WiFi detection look like a
phantom and rig RQ1 in radar's favour, so the distinction is explicit and tested.

Pure given solved Sionna arrays -- so it is unit-tested with fakes built to the layouts we
MEASURED on the server, and Sionna stays out of the test path."
```

---

### Task 5: The cell registry

**Files:**
- Create: `src/wifi_radar_slam/radar/cells.py`
- Test: `tests/test_cells.py`

**Interfaces:**
- Consumes: `radar.config` presets.
- Produces: `Cell` (frozen dataclass: `key, label, config, monostatic, front_end`) and `CELLS: dict[str, Cell]` with keys `A`, `B`, `C`, `D` (the MUSIC reference row is *not* a `Cell` — it does not use this chain, and Task 6 keeps it separate on purpose).

- [ ] **Step 1: Write the failing test**

Create `tests/test_cells.py`:

```python
import numpy as np
import pytest
from wifi_radar_slam.radar.cells import CELLS


def test_the_four_cells_exist():
    assert set(CELLS) == {"A", "B", "C", "D"}


def test_A_to_B_changes_ONLY_the_geometry():
    # The whole ablation rests on one-variable-at-a-time. If A and B ever differ in carrier
    # or bandwidth, the A->B step stops measuring geometry and the paper's decomposition is
    # meaningless.
    a, b = CELLS["A"], CELLS["B"]
    assert a.config.carrier_hz == b.config.carrier_hz
    assert a.config.bandwidth_hz == b.config.bandwidth_hz
    assert a.monostatic is False and b.monostatic is True


def test_B_to_C_changes_ONLY_the_carrier():
    b, c = CELLS["B"], CELLS["C"]
    assert b.config.bandwidth_hz == c.config.bandwidth_hz
    assert b.monostatic == c.monostatic is True
    assert b.config.carrier_hz == 5.2e9 and c.config.carrier_hz == 77e9


def test_C_to_D_changes_ONLY_the_bandwidth():
    c, d = CELLS["C"], CELLS["D"]
    assert c.config.carrier_hz == d.config.carrier_hz == 77e9
    assert c.monostatic == d.monostatic is True
    assert c.config.bandwidth_hz == 160e6 and d.config.bandwidth_hz == 4e9


def test_every_cell_uses_the_SAME_detection_chain():
    # "CFAR" everywhere. A cell that quietly switched front-end would confound the
    # algorithm with the physics the ablation is trying to isolate.
    assert {c.front_end for c in CELLS.values()} == {"cfar"}


def test_bandwidth_sets_range_resolution_and_carrier_does_not():
    # The physical claim the C->D step tests: resolution is c/2B, independent of carrier.
    assert CELLS["B"].config.range_resolution_m == pytest.approx(
        CELLS["C"].config.range_resolution_m)
    assert CELLS["D"].config.range_resolution_m < 0.1
```

- [ ] **Step 2: Run the test and watch it fail**

```bash
.venv/bin/python -m pytest tests/test_cells.py -q
```

Expected: `ModuleNotFoundError: No module named 'wifi_radar_slam.radar.cells'`.

- [ ] **Step 3: Implement**

Create `src/wifi_radar_slam/radar/cells.py`:

```python
"""The ablation's five cells -- the single place paper 3's physics is defined.

Each cell runs the IDENTICAL detection chain (beat -> range FFT -> azimuth beamforming ->
CA-CFAR). Only the physics changes, and only ONE variable at a time, so each step of the
ladder isolates exactly one mechanism:

    A -> B   GEOMETRY   (bistatic ambient WiFi  ->  monostatic active WiFi)
    B -> C   CARRIER    (5.2 GHz  ->  77 GHz, bandwidth held at 160 MHz)
    C -> D   BANDWIDTH  (160 MHz  ->  4 GHz, carrier held at 77 GHz)

If any step ever changed two things at once, the decomposition would be meaningless -- which
is why the tests pin one-variable-at-a-time rather than merely checking the values.

The fifth row -- WiFi + joint 2-D MUSIC, papers 1-2's front-end -- is deliberately NOT a Cell:
it does not use this chain. It is run separately (see experiments/run_ablation.py) precisely
so the superresolution-vs-FFT axis stays VISIBLE rather than silently confounded with the
physics.
"""
from __future__ import annotations
from dataclasses import dataclass

from .config import RadarConfig, RADAR_77G_4G, RADAR_77G_160M, WIFI_5G2_160M


@dataclass(frozen=True)
class Cell:
    key: str
    label: str
    config: RadarConfig
    monostatic: bool          # False -> bistatic (ambient AP illuminates; ellipse solve)
    front_end: str            # "cfar" for every cell -- the chain is held fixed
    isolates: str


CELLS: dict[str, Cell] = {
    "A": Cell("A", "WiFi baseline (bistatic, ambient)", WIFI_5G2_160M,
              monostatic=False, front_end="cfar", isolates="-- (the baseline)"),
    "B": Cell("B", "WiFi monostatic (active)", WIFI_5G2_160M,
              monostatic=True, front_end="cfar", isolates="GEOMETRY (A->B)"),
    "C": Cell("C", "Radar narrowband (77 GHz, 160 MHz)", RADAR_77G_160M,
              monostatic=True, front_end="cfar", isolates="CARRIER (B->C)"),
    "D": Cell("D", "Radar full (77 GHz, 4 GHz)", RADAR_77G_4G,
              monostatic=True, front_end="cfar", isolates="BANDWIDTH (C->D)"),
}
```

- [ ] **Step 4: Run the tests and watch them pass**

```bash
.venv/bin/python -m pytest tests/test_cells.py -q
```

Expected: 6 passed.

- [ ] **Step 5: Full suite, then commit**

```bash
.venv/bin/python -m pytest -q
```

Expected: 190 prior + 34 new = **224 passed**.

```bash
git add src/wifi_radar_slam/radar/cells.py tests/test_cells.py
git commit -m "paper3(radar): the ablation's cell registry -- one variable at a time

A->B geometry, B->C carrier, C->D bandwidth. The tests pin ONE-VARIABLE-AT-A-TIME rather than
merely checking values: if any step ever changed two things at once, the whole decomposition
would be meaningless. The MUSIC reference row is deliberately NOT a Cell -- it does not use
this chain, and keeping it separate is what keeps the superresolution-vs-FFT axis visible
instead of silently confounded with the physics."
```

---

### Task 6: The ablation runner

**Files:**
- Create: `experiments/run_ablation.py`

**Interfaces:**
- Consumes: `CELLS` (Task 5), `SionnaRadarSensor` (sub-project 1), `SionnaBistaticSensor` (Task 3), `true_paths_for_tx` (Task 4), `phantom_stats` (Task 1), `map_under_gt_poses` (Task 2), `eval.metrics` (chamfer / map_accuracy / map_completeness / occupancy_iou), and papers 1–2's `simulate_csi` + `extract_detections` + `_triangulate_bistatic` for the MUSIC reference row.

- [ ] **Step 1: Write the runner**

Create `experiments/run_ablation.py`:

```python
"""THE ABLATION -- paper 3's experiment. Server-only (needs Sionna).

Five cells, ONE detection chain, scored UNDER GROUND-TRUTH POSES (no estimator in the loop
for anyone -- see docs/results-paper3-anchor.md for why that is a stronger experiment here,
not a weaker one).

    A  WiFi bistatic   5.2 GHz  160 MHz   ambient AP illuminates -> ELLIPSE solve
    B  WiFi monostatic 5.2 GHz  160 MHz   own TX                 -> isolates GEOMETRY
    C  Radar narrow    77 GHz   160 MHz   own TX                 -> isolates CARRIER
    D  Radar full      77 GHz   4 GHz     own TX                 -> isolates BANDWIDTH
    M  WiFi + joint 2-D MUSIC (papers 1-2's front-end)  -- the reference row, so the
       superresolution-vs-FFT axis is visible rather than silently confounded

RQ1 (the headline) is the PHANTOM RATE, reported at BOTH a fixed tolerance (comparable to
paper 2's ~89 %) and a resolution-scaled one (which stops a high-resolution sensor from
looking phantom-free by construction).

    WRS_NUM_SAMPLES=1000000 nice -n 19 ionice -c3 .venv/bin/python experiments/run_ablation.py
"""
from __future__ import annotations
import json
import logging
import os
import time

import numpy as np

from wifi_radar_slam.config import load_config
from wifi_radar_slam.scene.builder import build_scene
from wifi_radar_slam.channel.simulator import simulate_csi
from wifi_radar_slam.sensing.frontend import extract_detections
from wifi_radar_slam.slam.particle_filter import _triangulate_bistatic
from wifi_radar_slam.lidar.pointcloud import Scan
from wifi_radar_slam.radar.cells import CELLS
from wifi_radar_slam.radar.sensor import SionnaRadarSensor
from wifi_radar_slam.radar.sensor_bistatic import SionnaBistaticSensor
from wifi_radar_slam.radar.truth import true_paths_for_tx
from wifi_radar_slam.eval.phantom import phantom_stats
from wifi_radar_slam.eval.mapping import map_under_gt_poses
from wifi_radar_slam.eval.metrics import (chamfer, map_accuracy, map_completeness,
                                          occupancy_iou)

SCENES = {
    "controlled_wall": "configs/controlled_music_joint.yaml",
    "street_canyon": "configs/street_metal_music.yaml",
}
SEEDS = [0, 1, 2]
MAP_VOXEL = 0.5

logging.basicConfig(level=logging.INFO, format="%(asctime)s %(message)s", datefmt="%H:%M:%S")
log = logging.getLogger("ablation")


def _map_metrics(est_map: np.ndarray, gt_xy: np.ndarray) -> dict:
    """The four map metrics. ATE/RPE are absent BY DESIGN: poses are ground truth, so there
    is no trajectory error to report for any cell."""
    if est_map.size == 0:
        return {"chamfer": float("inf"), "map_accuracy": float("inf"),
                "map_completeness": float("inf"), "iou": 0.0, "n_map_points": 0}
    return {
        "chamfer": chamfer(est_map, gt_xy),
        "map_accuracy": map_accuracy(est_map, gt_xy),
        "map_completeness": map_completeness(est_map, gt_xy),
        "iou": occupancy_iou(est_map, gt_xy, cell=1.0),
        "n_map_points": int(est_map.shape[0]),
    }


def run_cell(cell, built, seed: int) -> dict:
    """One cell, one scene, one seed -> phantom stats (both tolerances) + map metrics."""
    rng = np.random.default_rng(seed)
    traj = built.trajectory
    gt_xy = built.ground_truth_map[:, :2]

    if cell.monostatic:
        sensor = SionnaRadarSensor(built, cell.config, rng)
        tx_index = sensor.tidx
        floor_ids = sensor.floor_ids
    else:
        sensor = SionnaBistaticSensor(built, cell.config, rng)
        floor_ids = sensor.floor_ids

    scans, det_r, det_a, true_r, true_a = [], [], [], [], []
    t0 = time.time()
    for f in range(len(traj)):
        pose = traj[f]
        if cell.monostatic:
            scan = sensor(pose)
            scans.append(scan)
            # the chain's detections, in the SAME quantity the true paths use (round-trip m)
            r = np.linalg.norm(scan.points, axis=1) if len(scan) else np.empty(0)
            a = (np.arctan2(scan.points[:, 1], scan.points[:, 0]) + float(pose[2])
                 if len(scan) else np.empty(0))
            paths = sensor._solve(pose)
            tp = true_paths_for_tx(paths, tx_index, float(pose[2]), floor_ids,
                                   monostatic=True)
        else:
            world, r, a, _ = sensor(pose)
            scans.append(Scan(np.empty((0, 2))))          # cell A maps in WORLD directly
            det_world = world
            paths = sensor._solve(pose)
            tp = {"range_m": np.empty(0), "azimuth_world_rad": np.empty(0), "n": 0}
            for k, t in enumerate(sensor.ap_idx):
                one = true_paths_for_tx(paths, t, float(pose[2]), floor_ids,
                                        monostatic=False)
                tp = {"range_m": np.concatenate([tp["range_m"], one["range_m"]]),
                      "azimuth_world_rad": np.concatenate(
                          [tp["azimuth_world_rad"], one["azimuth_world_rad"]]),
                      "n": tp["n"] + one["n"]}

        det_r.append(np.asarray(r).ravel())
        det_a.append(np.angle(np.exp(1j * np.asarray(a).ravel())))
        true_r.append(tp["range_m"])
        true_a.append(tp["azimuth_world_rad"])
        if cell.monostatic is False:
            scans[-1] = Scan(np.empty((0, 2)))
            if f == 0:
                cellA_world = [det_world]
            else:
                cellA_world.append(det_world)
        if f % 100 == 0 or f == len(traj) - 1:
            el = time.time() - t0
            eta = el / max(f + 1, 1) * (len(traj) - f - 1)
            log.info("    cell %s frame %4d/%d  elapsed %.0fs  ETA %.0fs",
                     cell.key, f, len(traj), el, eta)

    # --- the map -----------------------------------------------------------------------
    if cell.monostatic:
        est_map = map_under_gt_poses(scans, traj, voxel=MAP_VOXEL)
    else:
        pts = np.concatenate([w for w in cellA_world if w.size]) if any(
            w.size for w in cellA_world) else np.empty((0, 2))
        cells_ = {}
        for p in pts:
            cells_.setdefault((int(round(p[0] / MAP_VOXEL)), int(round(p[1] / MAP_VOXEL))), p)
        est_map = np.array(list(cells_.values())) if cells_ else np.empty((0, 2))

    # --- RQ1: the phantom rate, at BOTH tolerances --------------------------------------
    DR = np.concatenate(det_r) if det_r else np.empty(0)
    DA = np.concatenate(det_a) if det_a else np.empty(0)
    TR = np.concatenate(true_r) if true_r else np.empty(0)
    TA = np.concatenate(true_a) if true_a else np.empty(0)

    fixed = phantom_stats(DR, DA, TR, TA, range_scale_m=3.0)
    res = cell.config.range_resolution_m
    scaled = phantom_stats(DR, DA, TR, TA, range_scale_m=3.0 * res)

    return {
        "cell": cell.key, "label": cell.label, "seed": seed,
        "carrier_ghz": cell.config.carrier_hz / 1e9,
        "bandwidth_mhz": cell.config.bandwidth_hz / 1e6,
        "monostatic": cell.monostatic,
        "range_resolution_m": res,
        "phantom_fixed_3m": fixed,
        "phantom_resolution_scaled": scaled,
        "map": _map_metrics(est_map, gt_xy),
    }


def run_music_reference(built, cfg, seed: int) -> dict:
    """The 5th row: WiFi + joint 2-D MUSIC -- papers 1-2's front-end, same GT poses.

    Kept OUT of the cell chain on purpose, so the superresolution-vs-FFT axis is visible
    rather than silently confounded with the physics the ablation is isolating.
    """
    rng = np.random.default_rng(seed)
    traj = built.trajectory
    gt_xy = built.ground_truth_map[:, :2]
    csi = simulate_csi(built, cfg.rf, cfg.snr_db, rng)
    dets = extract_detections(csi, cfg.rf, n_paths=3, world_aoa=cfg.world_aoa, joint=True)

    pts = []
    for f in range(len(traj)):
        D = np.asarray(dets[f]).reshape(-1, 3)
        for (pl, aoa, ap_i) in D:
            ap_xy = np.asarray(built.ap_positions[int(ap_i)])[:2]
            R = _triangulate_bistatic(traj[f][:2], ap_xy, float(pl), float(aoa))
            if R is not None:
                pts.append(R)
    cells_ = {}
    for p in pts:
        cells_.setdefault((int(round(p[0] / MAP_VOXEL)), int(round(p[1] / MAP_VOXEL))), p)
    est_map = np.array(list(cells_.values())) if cells_ else np.empty((0, 2))
    return {"cell": "M", "label": "WiFi + joint 2-D MUSIC (papers 1-2 front-end)",
            "seed": seed, "map": _map_metrics(est_map, gt_xy)}


def main() -> None:
    results = []
    for scene, cfgp in SCENES.items():
        cfg = load_config(cfgp)
        log.info("=== scene %s (%s) ===", scene, cfgp)
        for seed in SEEDS:
            built = build_scene(cfg)                    # rebuild: sensors mutate the scene
            for key in ("A", "B", "C", "D"):
                log.info("  cell %s, seed %d ...", key, seed)
                r = run_cell(CELLS[key], built, seed)
                r["scene"] = scene
                results.append(r)
                log.info("  cell %s seed %d: phantom(fixed)=%.1f%% "
                         "phantom(res-scaled)=%.1f%% IoU=%.3f",
                         key, seed,
                         100 * r["phantom_fixed_3m"]["phantom_rate"],
                         100 * r["phantom_resolution_scaled"]["phantom_rate"],
                         r["map"]["iou"])
            built = build_scene(cfg)
            m = run_music_reference(built, cfg, seed)
            m["scene"] = scene
            results.append(m)
            log.info("  MUSIC ref seed %d: IoU=%.3f", seed, m["map"]["iou"])

    os.makedirs("results", exist_ok=True)
    with open("results/ablation.json", "w") as f:
        json.dump(results, f, indent=2)
    log.info("saved -> results/ablation.json  (%d rows)", len(results))


if __name__ == "__main__":
    main()
```

- [ ] **Step 2: Commit and push**

```bash
git add experiments/run_ablation.py
git commit -m "paper3: the ablation runner -- 5 cells, one chain, ground-truth poses"
git push -u origin paper3-sub3-ablation
```

- [ ] **Step 3: Smoke-run ONE cell on the server before committing to the full grid**

The full grid is 2 scenes × 3 seeds × 5 rows and each cell ray-traces every frame — do **not**
launch it blind. On `amd`:

```bash
cd /home/dev/mulham/wifi-radar-slam
git fetch origin '+refs/heads/paper3-sub3-ablation:refs/remotes/origin/paper3-sub3-ablation'
git checkout -B paper3-sub3-ablation origin/paper3-sub3-ablation
WRS_NUM_SAMPLES=200000 nice -n 19 ionice -c3 .venv/bin/python - <<'PY'
import numpy as np, logging
logging.basicConfig(level=logging.INFO)
from wifi_radar_slam.config import load_config
from wifi_radar_slam.scene.builder import build_scene
from wifi_radar_slam.radar.cells import CELLS
import experiments.run_ablation as A
cfg = load_config("configs/controlled_music_joint.yaml")
for key in ("D", "A"):                       # one monostatic, one bistatic
    built = build_scene(cfg)
    r = A.run_cell(CELLS[key], built, seed=0)
    print(key, "phantom(fixed)", r["phantom_fixed_3m"]["phantom_rate"],
          "phantom(scaled)", r["phantom_resolution_scaled"]["phantom_rate"],
          "IoU", r["map"]["iou"], "map pts", r["map"]["n_map_points"])
PY
```

**Sanity gates before spending the full grid:**
- Cell D must produce **> 0 detections and > 0 map points**. Zero means the sensor is blind — check diffuse scattering.
- Cell A's phantom rate should be **broadly in paper 2's ballpark (high)**. If it comes back near 0 %, suspect the `bistatic_path_len` factor of 2 — a missing factor makes every detection match a true path at half range.
- If cell A's rate is ~100 % while cell D's is low, suspect the **monostatic vs bistatic range convention** in `true_paths_for_tx` before believing the result. That asymmetry is exactly the kind of bug that would produce a spectacular, wrong headline.

- [ ] **Step 4: Run the full grid**

```bash
WRS_NUM_SAMPLES=1000000 nohup nice -n 19 ionice -c3 \
  .venv/bin/python experiments/run_ablation.py > /tmp/ablation.log 2>&1 &
tail -f /tmp/ablation.log
```

Watch the ETA lines. Pull `results/ablation.json` back with `scp` when done.

---

### Task 7: The decomposition — what the paper actually says

**Files:**
- Create: `docs/results-paper3-ablation.md`
- Copy back: `results/ablation.json`

- [ ] **Step 1: Write the results doc with the REAL numbers**

It must contain, with no placeholders:

1. **RQ1 — the headline.** The phantom rate per cell, at **both** tolerances, side by side, with paper 2's ~89 % WiFi figure in the same table for continuity. Then the answer, stated plainly: **is the phantom ceiling universal to RF sensing, or WiFi-specific?**
2. **RQ2 — the decomposition.** A→B (geometry), B→C (carrier), C→D (bandwidth): how much of radar's advantage each step accounts for, in phantom rate *and* in map quality. State which mechanism dominates. If bandwidth dominates, say so — the thesis then becomes *bandwidth, not carrier*, which tells a WiFi-standards reader exactly what would have to change.
3. **RQ3 — mapping.** Chamfer, map accuracy, map completeness, occupancy IoU per cell, under GT poses, mean ± std over seeds. Plus the MUSIC reference row, so the superresolution-vs-FFT axis is visible.
4. **The limitations, stated and not buried:** poses are ground truth (no SLAM — and *why*, citing the sub-project 2 finding); amplitudes are frequency-flat across the sweep; ground returns are dropped; cell A uses all three APs while B/C/D use one transmitter (a genuine part of ambient WiFi's geometry, favouring WiFi, not a thumb on the scale for radar).
5. **If radar wins, say by how much and WHY.** If WiFi's ceiling turns out to be WiFi-specific, that is the clean, useful finding, and RQ2 explains it. Do not editorialise either way.

- [ ] **Step 2: Full suite, commit, merge, tag**

```bash
.venv/bin/python -m pytest -q
git add docs/results-paper3-ablation.md results/ablation.json
git commit -m "paper3: the ablation results -- RQ1, RQ2, RQ3"
git push
git checkout paper3-wifi-vs-radar
git merge --no-ff paper3-sub3-ablation
.venv/bin/python -m pytest -q
git tag -a paper3-v0.3.0 -m "Paper 3, sub-project 3: the ablation"
git push origin paper3-wifi-vs-radar paper3-v0.3.0
```

---

## Definition of done

- [ ] `eval/phantom.py` — paper-2's phantom definition, at **both** tolerances. Unit-tested.
- [ ] `eval/mapping.py` — mapping under GT poses, no estimator. Unit-tested.
- [ ] `radar/truth.py` — the true-path ground truth, monostatic **and** bistatic ranges. Unit-tested with fakes.
- [ ] `radar/sensor_bistatic.py` — cell A on the same chain + the ellipse solve. Unit-tested.
- [ ] `radar/cells.py` — the registry, with one-variable-at-a-time pinned by tests.
- [ ] The full grid run; `results/ablation.json` committed.
- [ ] `docs/results-paper3-ablation.md` — RQ1 answered, RQ2 decomposed, RQ3 tabulated, limitations stated.
- [ ] Full suite green; merged; tagged `paper3-v0.3.0`.

**Then sub-project 4: cost (RQ4) + the manuscript.**

---

## Self-review of this plan

**Spec coverage.** The spec's sub-project-3 row asks for "the five cells × two scenes → phantom rate + the six metrics, **under ground-truth poses**". Task 1 → the phantom rate (RQ1); Task 2 → GT-pose mapping; Tasks 3–4 → cell A and the truth it is scored against; Task 5 → the one-variable-at-a-time registry (RQ2); Task 6 → the runner, including the MUSIC reference row the spec requires so the superresolution axis is not confounded; Task 7 → the decomposition. The spec's honesty guards are encoded as Global Constraints 4 and 5, and the "radar is expected to win — explain the gap, don't close it" rule is why cell A keeps all three APs.

**ATE/RPE are deliberately absent**, and that is a spec change made in the open: poses are ground truth, so there is no trajectory error for *anyone*. The spec's revised RQ3 says exactly this.

**The one methodological risk I have front-loaded** is the phantom tolerance. A fixed 3 m window would hand radar a lower phantom rate *by construction*, since cell D resolves to 0.0375 m and cell A to 0.94 m. Reporting both a fixed and a resolution-scaled tolerance is the only honest way to separate *phantom* from *coarse*, and Task 1's tests pin it.

**Type consistency.** `phantom_stats(det_range_m, det_azimuth_rad, true_range_m, true_azimuth_rad, range_scale_m=...)` is defined in Task 1 and called with that signature in Task 6. `true_paths_for_tx(paths, tx_index, yaw, floor_ids, monostatic)` returns `{range_m, azimuth_world_rad, n}` in Task 4 and is unpacked by those keys in Task 6. `map_under_gt_poses(scans, poses, voxel)` matches. `_triangulate_bistatic(pose_xy, ap_xy, path_len, aoa, min_excess_m=0.0)` is the existing signature, verified in the source, and is called that way in Task 3. `Cell.config` is a `RadarConfig`, so `cell.config.range_resolution_m` in Task 6 resolves. `SionnaRadarSensor` exposes `.tidx`, `.floor_ids`, `._solve()`, and `__call__(pose) -> Scan`, all of which exist in sub-project 1.
