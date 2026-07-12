import numpy as np
from wifi_radar_slam.map_filter import (music_features, label_from_gt, HeuristicFilter,
                                        SklearnFilter, FEATURE_NAMES)
from wifi_radar_slam.slam.particle_filter import run_slam

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
