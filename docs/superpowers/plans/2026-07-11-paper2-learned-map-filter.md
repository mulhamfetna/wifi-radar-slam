# Paper 2 Learned Map Enhancement (RQ2) Implementation Plan

> **For agentic workers:** REQUIRED SUB-SKILL: Use superpowers:subagent-driven-development (recommended) or superpowers:executing-plans to implement this plan task-by-task. Steps use checkbox (`- [ ]`) syntax for tracking.

**Goal:** Close paper-1's open loop — plug a learned path discriminator into the WiFi mapping pipeline and run the ladder (none → heuristic → RandomForest → MLP) to answer whether deep learning is *needed* to close the WiFi mapping gap (realistic-CSI IoU ≈ 0).

**Architecture:** A new `map_filter.py` with MUSIC-observable feature extraction, operational GT labels, and three interchangeable filters sharing one call contract. `run_slam` gains an optional `map_filter` hook that gates which detections enter the **map** while leaving particle weighting (localization) untouched. A training script fits RF + MLP with leakage-free splits; an eval script rebuilds maps for all four rungs and emits the six metrics.

**Tech Stack:** Python 3, NumPy, scikit-learn (the project's declared `ml` extra; installed locally — `RandomForestClassifier`, `MLPClassifier`, `joblib`). Sionna only for generating the CSI in the training/eval runs (server).

## Global Constraints

- **Branch:** all work on `paper2-map-filter`, cut from `paper2-wifi-vs-lidar`; merge back on completion. Never commit to `main` or any `paper1-*` ref.
- **NO ORACLE FEATURES AT INFERENCE.** The four features are exactly
  `[path_len, excess, abs_azimuth, aoa_dev]` — all computable from a MUSIC detection
  `(path_len, aoa, ap_index)` plus the current pose estimate. **`elevation` is EXCLUDED**:
  our single-ULA 2-D (delay–azimuth) MUSIC never estimates it, so paper-1's five-feature
  model (F1 ≈ 0.9) leaked an unmeasurable quantity. Report the corrected, lower F1.
- **Filter contract:** `filter(dets, pose, ap_positions) -> np.ndarray[bool] (k,)`.
  Feature extraction happens *inside* the filter, so `slam/particle_filter.py` never imports
  `map_filter` (avoids a circular import).
- **Filter gates MAPPING ONLY.** A rejected detection still updates the particle weights; it
  is merely not appended to `mapped_points`. `map_filter=None` (default) ⇒ byte-identical to
  today's behaviour, so **paper-1 results are unchanged**.
- **GT only at training.** Labels use the GT footprint map; inference uses none.
- **Train and infer at ESTIMATED poses.** Features/labels for training are computed at the
  poses from an unfiltered `run_slam` pass — exactly the poses the filter will see at
  inference. No GT-pose/estimated-pose distribution shift.
- **Leakage-free splits, both reported:** (1) held-out frames (first 60 % train / last 40 %
  test, same scene); (2) cross-scene (train controlled → test street, and vice versa).
- **A negative result is a result.** "The heuristic suffices" or "even the MLP cannot close
  the gap" are legitimate RQ2 answers. Do not tune to manufacture a win.
- **Report all six metrics** — filtering may raise accuracy/IoU while *lowering* completeness.

---

### Task 1: `map_filter.py` — features, labels, heuristic rung

**Files:**
- Create: `src/wifi_radar_slam/map_filter.py`
- Test: `tests/test_map_filter.py`

**Interfaces:**
- Consumes: `_triangulate_bistatic` (from `slam.particle_filter`).
- Produces:
  `FEATURE_NAMES = ["path_len_m", "excess_m", "abs_azimuth", "aoa_dev_from_ap"]`;
  `music_features(dets, pose, ap_positions) -> np.ndarray (k,4)`;
  `label_from_gt(dets, pose, ap_positions, gt_xy, tol=1.0) -> np.ndarray (k,) int`;
  `HeuristicFilter(min_excess_m=1.5)` with `__call__(dets, pose, ap_positions) -> bool (k,)`.

- [ ] **Step 1: Write the failing tests**

```python
# tests/test_map_filter.py
import numpy as np
from wifi_radar_slam.map_filter import (music_features, label_from_gt, HeuristicFilter,
                                        FEATURE_NAMES)

APS = [np.array([0.0, 20.0, 6.0])]


def _det_for(reflector, pose_xy, ap_xy=np.array([0.0, 20.0])):
    """Build the (path_len, aoa, ap) detection a perfect sensor would report."""
    d = np.asarray(reflector) - np.asarray(pose_xy)
    path = np.linalg.norm(ap_xy - np.asarray(reflector)) + np.linalg.norm(d)
    return [path, np.arctan2(d[1], d[0]), 0.0]


def test_features_are_music_observable_only():
    # exactly four features, and NO elevation (a 2-D ULA cannot measure it)
    assert FEATURE_NAMES == ["path_len_m", "excess_m", "abs_azimuth", "aoa_dev_from_ap"]
    assert not any("elev" in n for n in FEATURE_NAMES)
    dets = np.array([_det_for([10.0, 3.0], [0.0, 0.0])])
    X = music_features(dets, (0.0, 0.0, 0.0), APS)
    assert X.shape == (1, 4)
    # excess = path_len - |AP - pose| and must be positive for a real detour
    assert X[0, 1] > 0


def test_features_empty_detections():
    assert music_features(np.empty((0, 3)), (0.0, 0.0, 0.0), APS).shape == (0, 4)


def test_label_is_1_only_when_the_reflector_lands_on_a_facade():
    gt = np.array([[10.0, 3.0], [10.0, 3.5]])          # a "facade" at x=10
    good = _det_for([10.0, 3.0], [0.0, 0.0])           # triangulates onto the facade
    far = _det_for([40.0, -30.0], [0.0, 0.0])          # triangulates far from any facade
    y = label_from_gt(np.array([good, far]), (0.0, 0.0, 0.0), APS, gt, tol=1.0)
    assert y.tolist() == [1, 0]


def test_heuristic_drops_low_excess_paths():
    # a near-LOS path barely detours (low excess) -> dropped; a real reflection -> kept
    pose = (0.0, 0.0, 0.0)
    dist_ap = 20.0
    los_like = [dist_ap + 0.5, np.deg2rad(80.0), 0.0]      # excess 0.5 m
    genuine = _det_for([10.0, 3.0], [0.0, 0.0])            # large detour
    keep = HeuristicFilter(min_excess_m=1.5)(np.array([los_like, genuine]), pose, APS)
    assert keep.tolist() == [False, True]
```

- [ ] **Step 2: Run tests to verify they fail**

Run: `pytest tests/test_map_filter.py -v`
Expected: FAIL with `ModuleNotFoundError: No module named 'wifi_radar_slam.map_filter'`

- [ ] **Step 3: Write minimal implementation**

```python
# src/wifi_radar_slam/map_filter.py
"""Learned map enhancement (paper 2, RQ2): filter MUSIC detections before they enter
the map, so phantom/LOS/floor/multi-bounce returns stop polluting it.

CRITICAL: the features here are ONLY what a commodity 2-D (delay-azimuth) MUSIC
front-end can actually measure. Paper 1's discriminator also used `elevation`, which a
single ULA never estimates -- an oracle feature. Its F1 ~ 0.9 is therefore optimistic;
we retrain on the observable subset and report the corrected number.

Filter contract (shared by every rung):
    filter(dets, pose, ap_positions) -> np.ndarray[bool] of shape (k,)
Feature extraction happens inside the filter, so slam/particle_filter.py never imports
this module (no circular import).
"""
from __future__ import annotations
import numpy as np

from .slam.particle_filter import _triangulate_bistatic

FEATURE_NAMES = ["path_len_m", "excess_m", "abs_azimuth", "aoa_dev_from_ap"]


def music_features(dets, pose, ap_positions) -> np.ndarray:
    """(k,4) features from MUSIC detections [path_len, aoa, ap_index] + the pose.

    NO elevation, NO interaction type, NO bounce count -- nothing a real 2-D CSI
    receiver could not compute.
    """
    dets = np.asarray(dets, dtype=float).reshape(-1, 3)
    if dets.shape[0] == 0:
        return np.empty((0, 4))
    path_len, aoa = dets[:, 0], dets[:, 1]
    ap_idx = dets[:, 2].astype(int)
    pose_xy = np.asarray(pose, dtype=float)[:2]
    ap_xy = np.array([np.asarray(ap_positions[i], dtype=float)[:2] for i in ap_idx])
    dist_ap = np.linalg.norm(ap_xy - pose_xy, axis=1)
    excess = path_len - dist_ap
    bearing = np.arctan2(ap_xy[:, 1] - pose_xy[1], ap_xy[:, 0] - pose_xy[0])
    aoa_dev = np.abs((aoa - bearing + np.pi) % (2 * np.pi) - np.pi)
    return np.column_stack([path_len, excess, np.abs(aoa), aoa_dev])


def label_from_gt(dets, pose, ap_positions, gt_xy, tol: float = 1.0) -> np.ndarray:
    """Operational label: 1 iff this detection triangulates to a reflector within `tol`
    of a true facade. That IS the definition of a mapping-useful detection. Ground truth
    is used ONLY here (training); inference never sees it.
    """
    dets = np.asarray(dets, dtype=float).reshape(-1, 3)
    y = np.zeros(dets.shape[0], dtype=int)
    gt = np.asarray(gt_xy, dtype=float).reshape(-1, 2)
    if dets.shape[0] == 0 or gt.shape[0] == 0:
        return y
    pose_xy = np.asarray(pose, dtype=float)[:2]
    for k in range(dets.shape[0]):
        path_len, aoa, ap_i = dets[k]
        ap_xy = np.asarray(ap_positions[int(ap_i)], dtype=float)[:2]
        refl = _triangulate_bistatic(pose_xy, ap_xy, path_len, aoa)
        if refl is None:                       # degenerate/LOS solve -> not useful
            continue
        if np.min(np.linalg.norm(gt - refl, axis=1)) <= tol:
            y[k] = 1
    return y


class HeuristicFilter:
    """Rung 1 (physics): keep detections whose bistatic excess clears a threshold.

    LOS and floor-bounce paths barely detour past the direct AP distance; a genuine
    facade reflection detours a lot. This is the existing `map_min_excess_m` gate,
    expressed as a filter so it sits on the same ladder as the learned rungs.
    """

    def __init__(self, min_excess_m: float = 1.5):
        self.min_excess_m = float(min_excess_m)

    def __call__(self, dets, pose, ap_positions) -> np.ndarray:
        X = music_features(dets, pose, ap_positions)
        if X.shape[0] == 0:
            return np.zeros(0, dtype=bool)
        return X[:, 1] >= self.min_excess_m       # column 1 == excess_m
```

- [ ] **Step 4: Run tests to verify they pass**

Run: `pytest tests/test_map_filter.py -v`
Expected: PASS (4 tests)

- [ ] **Step 5: Commit**

```bash
git add src/wifi_radar_slam/map_filter.py tests/test_map_filter.py
git commit -m "paper2(rq2): MUSIC-observable features, operational GT labels, heuristic rung"
```

---

### Task 2: `SklearnFilter` + the `run_slam(map_filter=...)` hook

**Files:**
- Modify: `src/wifi_radar_slam/map_filter.py` (append `SklearnFilter`)
- Modify: `src/wifi_radar_slam/slam/particle_filter.py` (add the `map_filter` arg)
- Test: `tests/test_map_filter.py` (append)

**Interfaces:**
- Consumes: `music_features` (Task 1); a fitted sklearn classifier.
- Produces: `SklearnFilter(model, threshold=0.5)` with the same
  `__call__(dets, pose, ap_positions) -> bool (k,)` contract;
  `run_slam(..., map_filter=None)` — when set, rejected detections still weight the
  particles but do **not** enter the map.

- [ ] **Step 1: Write the failing tests**

```python
# append to tests/test_map_filter.py
from wifi_radar_slam.map_filter import SklearnFilter
from wifi_radar_slam.slam.particle_filter import run_slam


class _KeepNone:
    """A filter that rejects everything -> the map must end up empty."""
    def __call__(self, dets, pose, ap_positions):
        return np.zeros(len(np.asarray(dets).reshape(-1, 3)), dtype=bool)


def _straight_case(n=20, dt=0.05, speed=5.0):
    velocity = np.tile([speed, 0.0], (n, 1))
    aps = [np.array([0.0, 20.0, 6.0])]
    refl = np.array([10.0, 3.0])
    gt = np.array([[speed * dt * f, 0.0, 0.0] for f in range(n)])
    dets = [np.array([_det_for(refl, gt[f, :2])]) for f in range(n)]
    return gt, velocity, aps, dets


def test_map_filter_gates_the_map_but_not_the_trajectory():
    gt, vel, aps, dets = _straight_case()
    base_traj, base_map = run_slam(dets, aps, vel, 0.05, np.random.default_rng(0),
                                   init_pose=gt[0])
    filt_traj, filt_map = run_slam(dets, aps, vel, 0.05, np.random.default_rng(0),
                                   init_pose=gt[0], map_filter=_KeepNone())
    # rejecting every detection empties the MAP ...
    assert base_map.shape[0] > 0
    assert filt_map.shape[0] == 0
    # ... but leaves LOCALIZATION untouched (detections still weight the particles)
    assert np.allclose(base_traj, filt_traj)


def test_sklearn_filter_uses_predict_proba_threshold():
    class _Model:                      # stand-in classifier: score = excess (column 1)
        def predict_proba(self, X):
            p = (np.asarray(X)[:, 1] > 2.0).astype(float)
            return np.column_stack([1 - p, p])
    pose = (0.0, 0.0, 0.0)
    aps = [np.array([0.0, 20.0, 6.0])]
    low = [20.5, np.deg2rad(80.0), 0.0]                     # excess 0.5 -> reject
    high = _det_for([10.0, 3.0], [0.0, 0.0])                # big excess -> keep
    keep = SklearnFilter(_Model())(np.array([low, high]), pose, aps)
    assert keep.tolist() == [False, True]
```

- [ ] **Step 2: Run tests to verify they fail**

Run: `pytest tests/test_map_filter.py -v`
Expected: FAIL — `ImportError: cannot import name 'SklearnFilter'` (and `run_slam` has no
`map_filter` argument).

- [ ] **Step 3a: Append `SklearnFilter` to `map_filter.py`**

```python
class SklearnFilter:
    """Rungs 2-3 (learned): wrap a fitted sklearn classifier (RandomForest or MLP) in
    the same contract as HeuristicFilter, so run_slam treats every rung identically."""

    def __init__(self, model, threshold: float = 0.5):
        self.model = model
        self.threshold = float(threshold)

    def __call__(self, dets, pose, ap_positions) -> np.ndarray:
        X = music_features(dets, pose, ap_positions)
        if X.shape[0] == 0:
            return np.zeros(0, dtype=bool)
        proba = self.model.predict_proba(X)[:, 1]
        return proba >= self.threshold
```

- [ ] **Step 3b: Add the hook to `slam/particle_filter.py`**

Change the signature (add `map_filter=None`) and gate only the map append. Replace the
detection loop body so the weight update still runs for every detection:

```python
def run_slam(detections, ap_positions, velocity, timestep_s, rng,
             n_particles: int = 200, init_pose=None, map_min_support: int = 1,
             map_min_excess_m: float = 0.0, map_filter=None):
```

and inside the `if dets.shape[0] > 0:` block, immediately after `mean_pose` is computed:

```python
            mean_pose = np.average(particles, axis=0, weights=weights)
            # RQ2: an optional learned/heuristic filter gates which detections enter the
            # MAP. Rejected detections still update the particle weights, so localization
            # is unaffected (map_filter=None -> identical to the original behaviour).
            keep = (map_filter(dets, mean_pose, ap_positions)
                    if map_filter is not None else None)
            for j, (path_len, aoa, ap_i) in enumerate(dets):
                ap_xy = np.asarray(ap_positions[int(ap_i)])[:2]
                refl = _triangulate_bistatic(mean_pose[:2], ap_xy, path_len, aoa,
                                             min_excess_m=map_min_excess_m)
                if refl is None:                           # direct path / degenerate
                    continue
                if keep is None or keep[j]:
                    mapped_points.append(refl)
                # weight update: bistatic consistency of each particle (ALWAYS)
                pr = np.array([_reproject_bistatic(p[:2], ap_xy, refl) for p in particles])
                err = (pr[:, 0] - path_len) ** 2 + (pr[:, 1] - aoa) ** 2
                weights *= np.exp(-0.5 * err / (0.5 ** 2))
```

- [ ] **Step 4: Run tests to verify they pass**

Run: `pytest tests/test_map_filter.py -v` → PASS (6 tests)
Run: `pytest -q` → the whole suite, including the existing `test_particle_filter.py`, must
still pass (default `map_filter=None` keeps paper-1 behaviour identical).

- [ ] **Step 5: Commit**

```bash
git add src/wifi_radar_slam/map_filter.py src/wifi_radar_slam/slam/particle_filter.py \
        tests/test_map_filter.py
git commit -m "paper2(rq2): SklearnFilter + run_slam map_filter hook (gates map, not localization)"
```

---

### Task 3: Train the ladder, rebuild the maps, answer RQ2

**Files:**
- Create: `experiments/train_map_filter.py`
- Create: `experiments/run_enhanced_map.py`
- Modify: `docs/results-paper2.md` (append an "Enhancement (RQ2)" section)

**Interfaces:**
- Consumes: `music_features`, `label_from_gt`, `HeuristicFilter`, `SklearnFilter`,
  `run_slam(map_filter=...)`; `build_scene`, `simulate_csi`, `extract_detections`.
- Produces: `data/map_filter_rf.joblib`, `data/map_filter_mlp.joblib`,
  `data/map_filter_f1.json`, `data/enhanced_map_results.json`.

- [ ] **Step 1: Write the training script**

Features and labels are computed at the **estimated** poses of an unfiltered `run_slam`
pass — exactly the poses the filter sees at inference, so there is no train/infer shift.

```python
# experiments/train_map_filter.py
"""RQ2: train the learned map filter on MUSIC-OBSERVABLE features only.

Paper 1's discriminator also used `elevation`, which a single-ULA 2-D MUSIC front-end
cannot measure -- an oracle feature. We retrain on the observable 4 and report the
corrected F1. Two leakage-free splits: held-out frames, and cross-scene.

Server (needs sionna-rt):
    WRS_NUM_SAMPLES=1000000 nice -n 19 ionice -c3 python experiments/train_map_filter.py
"""
import json
import joblib
import numpy as np
from sklearn.ensemble import RandomForestClassifier
from sklearn.neural_network import MLPClassifier
from sklearn.metrics import f1_score, classification_report

from wifi_radar_slam.config import load_config
from wifi_radar_slam.scene.builder import build_scene
from wifi_radar_slam.channel.simulator import simulate_csi
from wifi_radar_slam.sensing.frontend import extract_detections
from wifi_radar_slam.geometry import velocity_from_poses
from wifi_radar_slam.slam.particle_filter import run_slam
from wifi_radar_slam.map_filter import music_features, label_from_gt

SCENES = {
    "controlled_wall": "configs/controlled_music_joint.yaml",
    "street_canyon": "configs/street_metal_music.yaml",
}


def collect(scene, cfgpath):
    """(X, y, frame_idx) at the ESTIMATED poses of an unfiltered run."""
    cfg = load_config(cfgpath)
    built = build_scene(cfg)
    gt, gt_xy = built.trajectory, built.ground_truth_map[:, :2]
    vel = velocity_from_poses(gt, cfg.trajectory.timestep_s)
    csi = simulate_csi(built, cfg.rf, cfg.snr_db, np.random.default_rng(cfg.seed))
    dets = extract_detections(csi, cfg.rf, n_paths=3, world_aoa=cfg.world_aoa,
                              joint=cfg.joint_estimation)
    est, _ = run_slam(dets, built.ap_positions, vel, cfg.trajectory.timestep_s,
                      np.random.default_rng(0), init_pose=gt[0],
                      map_min_support=cfg.map_min_support,
                      map_min_excess_m=cfg.map_min_excess_m)
    Xs, ys, fs = [], [], []
    for f in range(len(dets)):
        if dets[f].shape[0] == 0:
            continue
        Xs.append(music_features(dets[f], est[f], built.ap_positions))
        ys.append(label_from_gt(dets[f], est[f], built.ap_positions, gt_xy, tol=1.0))
        fs.append(np.full(dets[f].shape[0], f))
    return np.vstack(Xs), np.concatenate(ys), np.concatenate(fs)


def _fit_and_score(Xtr, ytr, Xte, yte, tag):
    out = {}
    for name, model in (("rf", RandomForestClassifier(n_estimators=300,
                                                      class_weight="balanced",
                                                      random_state=0, n_jobs=-1)),
                        ("mlp", MLPClassifier(hidden_layer_sizes=(32, 16), max_iter=2000,
                                              random_state=0))):
        if ytr.sum() == 0 or ytr.sum() == len(ytr):
            print(f"[{tag}/{name}] degenerate labels; skipping")
            continue
        model.fit(Xtr, ytr)
        f1 = f1_score(yte, model.predict(Xte), zero_division=0)
        out[name] = f1
        print(f"[{tag}/{name}] F1 = {f1:.3f}")
        print(classification_report(yte, model.predict(Xte), zero_division=0, digits=3))
    return out


def main() -> None:
    data = {s: collect(s, p) for s, p in SCENES.items()}
    report = {}

    for scene, (X, y, f) in data.items():
        print(f"\n=== {scene}: {len(y)} detections, useful = {int(y.sum())} "
              f"({100 * y.mean():.1f}%) ===")
        cut = np.quantile(f, 0.6)                      # held-out FRAMES (temporal split)
        tr, te = f <= cut, f > cut
        report[f"{scene}_heldout_frames"] = _fit_and_score(X[tr], y[tr], X[te], y[te],
                                                           f"{scene}/frames")

    names = list(SCENES)                               # cross-scene generalization
    for a, b in ((names[0], names[1]), (names[1], names[0])):
        Xa, ya, _ = data[a]
        Xb, yb, _ = data[b]
        report[f"train_{a}_test_{b}"] = _fit_and_score(Xa, ya, Xb, yb, f"{a}->{b}")

    # Ship models trained on the FULL data of each scene. The map evaluation applies
    # each scene's model to the *other* scene (cross-scene), so training on all frames
    # here introduces NO leakage -- and it is the honest deployment case (a filter
    # trained offline, deployed somewhere new).
    for scene, (X, y, _f) in data.items():
        rf = RandomForestClassifier(n_estimators=300, class_weight="balanced",
                                    random_state=0, n_jobs=-1).fit(X, y)
        mlp = MLPClassifier(hidden_layer_sizes=(32, 16), max_iter=2000,
                            random_state=0).fit(X, y)
        joblib.dump(rf, f"data/map_filter_rf_{scene}.joblib")
        joblib.dump(mlp, f"data/map_filter_mlp_{scene}.joblib")

    with open("data/map_filter_f1.json", "w") as fh:
        json.dump(report, fh, indent=2)
    print("\nsaved -> data/map_filter_f1.json + per-scene rf/mlp models")


if __name__ == "__main__":
    main()
```

- [ ] **Step 2: Write the evaluation script (the ladder)**

```python
# experiments/run_enhanced_map.py
"""RQ2: rebuild the WiFi map under each ladder rung and score it.

rung 0 = none (pure WiFi)   rung 1 = physics heuristic (min-excess gate)
rung 2 = RandomForest       rung 3 = MLP

LEAKAGE-FREE BY CONSTRUCTION: each scene's map is rebuilt using the model trained on the
*other* scene. So no frame the filter trained on contributes to the map it is scored on,
the full trajectory is kept (numbers stay comparable to the LiDAR/fusion rows), and this
is the honest deployment case -- a filter trained offline, deployed somewhere new.

Server (needs sionna-rt):
    WRS_NUM_SAMPLES=1000000 nice -n 19 ionice -c3 python experiments/run_enhanced_map.py
"""
import json
import joblib
import numpy as np

from wifi_radar_slam.config import load_config
from wifi_radar_slam.scene.builder import build_scene
from wifi_radar_slam.channel.simulator import simulate_csi
from wifi_radar_slam.sensing.frontend import extract_detections
from wifi_radar_slam.geometry import velocity_from_poses
from wifi_radar_slam.slam.particle_filter import run_slam
from wifi_radar_slam.map_filter import HeuristicFilter, SklearnFilter
from wifi_radar_slam.eval.metrics import (ate, rpe, chamfer, occupancy_iou,
                                          map_accuracy, map_completeness)

SCENES = {
    "controlled_wall": "configs/controlled_music_joint.yaml",
    "street_canyon": "configs/street_metal_music.yaml",
}


def _metrics(est, m, gt, gt_xy):
    return {"ate": ate(est, gt), "rpe": rpe(est, gt), "chamfer": chamfer(m, gt_xy),
            "map_accuracy": map_accuracy(m, gt_xy),
            "map_completeness": map_completeness(m, gt_xy),
            "iou": occupancy_iou(m, gt_xy, cell=1.0)}


def main() -> None:
    results = {}
    names = list(SCENES)
    other = {names[0]: names[1], names[1]: names[0]}    # cross-scene: no leakage
    for scene, cfgpath in SCENES.items():
        cfg = load_config(cfgpath)
        built = build_scene(cfg)
        gt, gt_xy = built.trajectory, built.ground_truth_map[:, :2]
        vel = velocity_from_poses(gt, cfg.trajectory.timestep_s)
        csi = simulate_csi(built, cfg.rf, cfg.snr_db, np.random.default_rng(cfg.seed))
        dets = extract_detections(csi, cfg.rf, n_paths=3, world_aoa=cfg.world_aoa,
                                  joint=cfg.joint_estimation)

        src = other[scene]          # model trained on the OTHER scene
        print(f"[{scene}] applying filters trained on '{src}' (leakage-free)")
        rungs = {
            "0_none": None,
            "1_heuristic": HeuristicFilter(min_excess_m=1.5),
            "2_random_forest": SklearnFilter(
                joblib.load(f"data/map_filter_rf_{src}.joblib")),
            "3_mlp": SklearnFilter(
                joblib.load(f"data/map_filter_mlp_{src}.joblib")),
        }
        results[scene] = {}
        for name, filt in rungs.items():
            est, m = run_slam(dets, built.ap_positions, vel, cfg.trajectory.timestep_s,
                              np.random.default_rng(0), init_pose=gt[0],
                              map_min_support=cfg.map_min_support,
                              map_min_excess_m=cfg.map_min_excess_m, map_filter=filt)
            results[scene][name] = _metrics(est, m, gt, gt_xy)
            print(f"[{scene}/{name}] {results[scene][name]}")

    with open("data/enhanced_map_results.json", "w") as f:
        json.dump(results, f, indent=2)
    print("saved -> data/enhanced_map_results.json")


if __name__ == "__main__":
    main()
```

- [ ] **Step 3: Parse-check locally and run the full suite**

Run: `python -m py_compile experiments/train_map_filter.py experiments/run_enhanced_map.py && echo OK`
Run: `pytest -q` → all pass.

- [ ] **Step 4: Run both on the amd server, throttled**

`WRS_NUM_SAMPLES=1000000 nice -n 19 ionice -c3 python experiments/train_map_filter.py`
then
`WRS_NUM_SAMPLES=1000000 nice -n 19 ionice -c3 python experiments/run_enhanced_map.py`
Expected: F1 for RF/MLP on both splits, then the six metrics for rungs 0–3 on both scenes.

- [ ] **Step 5: Write the RQ2 section and answer the question**

Append `## Enhancement (RQ2)` to `docs/results-paper2.md` with: the corrected discriminator
F1 (held-out frames **and** cross-scene) against paper-1's 0.9, with the **elevation-is-an-
oracle-feature** caveat stated plainly; the ladder table (rungs 0–3 × six metrics × both
scenes); and an explicit verdict —
**(a)** does any rung close the gap to LiDAR IoU (0.149–1.000)?
**(b)** is **deep learning needed**, or does the heuristic/RF suffice?
**(c)** what did filtering cost in map *completeness*?
If cross-scene F1 collapses, say so: a scene-specific filter is not a deployable enhancement.

```bash
git add experiments/train_map_filter.py experiments/run_enhanced_map.py \
        data/map_filter_f1.json data/enhanced_map_results.json docs/results-paper2.md
git commit -m "paper2(rq2): learned map-filter ladder results + RQ2 verdict"
```

---

## After this plan

Merge `paper2-map-filter` into `paper2-wifi-vs-lidar`, record the RQ2 verdict in the DOSSIER,
and tag `paper2-v0.4.0`. **All five research questions are then answered** — the next step is
assembling paper 2's manuscript (its own cycle), reusing the paper-1 IEEEtran scaffold.
