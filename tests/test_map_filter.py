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
