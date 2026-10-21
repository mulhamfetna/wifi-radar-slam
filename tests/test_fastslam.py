"""Tests for the virtual-anchor FastSLAM back-end and the noisy-odometry model.

The key regression guard: with *noisy* odometry, real WiFi measurements must reduce the
trajectory error relative to dead reckoning. The legacy filter failed this (its update was
self-referential), which is why it exists.
"""
import numpy as np

from wifi_radar_slam.odometry import OdometryNoise, noisy_velocity, dead_reckon
from wifi_radar_slam.slam.fastslam import run_fastslam, FastSlamParams
from wifi_radar_slam.eval.metrics import ate

DT = 0.05


def _straight(n=240, speed=5.0):
    gt = np.zeros((n, 3))
    gt[:, 0] = np.arange(n) * speed * DT - 30.0
    vel = np.zeros((n, 2)); vel[:, 0] = speed
    return gt, vel


def _wall_detections(gt, aps, walls_y, rng=None, sig_r=0.0, sig_a=0.0):
    """Exact single-bounce detections off horizontal walls y=w: the virtual anchor is the
    AP mirrored in the wall; path length = |VA - p|, AoA = bearing to VA."""
    dets = []
    for p in gt:
        rows = []
        for i, a in enumerate(aps):
            for w in walls_y:
                if (a[1] - w) * (p[1] - w) <= 0:      # AP and vehicle on same side only
                    continue
                va = np.array([a[0], 2 * w - a[1]])
                d = va - p[:2]
                r, b = np.linalg.norm(d), np.arctan2(d[1], d[0])
                if rng is not None:
                    r += rng.normal(0, sig_r); b += rng.normal(0, sig_a)
                rows.append([r, b, i])
        dets.append(np.array(rows) if rows else np.empty((0, 3)))
    return dets


APS = np.array([[0.0, 6.0, 3.0], [20.0, -6.0, 3.0]])
WALLS = [12.0, -12.0]


def test_noisy_velocity_drifts_and_is_seeded():
    gt, vel = _straight()
    noise = OdometryNoise()
    v1 = noisy_velocity(vel, DT, noise, np.random.default_rng(3))
    v2 = noisy_velocity(vel, DT, noise, np.random.default_rng(3))
    assert np.array_equal(v1, v2)
    dr = dead_reckon(v1, DT, gt[0])
    assert ate(dr, gt) > 0.1                       # wheel-odometry-grade drift is visible


def test_zero_odometry_noise_dead_reckoning_is_exact():
    gt, vel = _straight()
    dr = dead_reckon(vel, DT, gt[0])
    assert np.allclose(dr[:, :2], gt[:, :2])


def test_wifi_reduces_drift_under_noisy_odometry():
    """REGRESSION GUARD: measurements must carry position information."""
    gt, vel = _straight()
    dets = _wall_detections(gt, APS, WALLS, np.random.default_rng(9), 0.2, np.deg2rad(1))
    gains = []
    for seed in range(4):
        v = noisy_velocity(vel, DT, OdometryNoise(), np.random.default_rng(seed))
        dr = ate(dead_reckon(v, DT, gt[0]), gt)
        est, _, _ = run_fastslam(dets, APS, v, DT, np.random.default_rng(100 + seed),
                                 init_pose=gt[0])
        gains.append(dr - ate(est, gt))
    assert np.mean(gains) > 0, gains
    assert np.mean([g > 0 for g in gains]) >= 0.75


def test_no_detections_reduces_to_dead_reckoning():
    gt, vel = _straight()
    v = noisy_velocity(vel, DT, OdometryNoise(), np.random.default_rng(1))
    empty = [np.empty((0, 3))] * len(gt)
    est, emap, info = run_fastslam(empty, APS, v, DT, np.random.default_rng(2),
                                   init_pose=gt[0])
    dr = dead_reckon(v, DT, gt[0])
    assert abs(ate(est, gt) - ate(dr, gt)) < 0.05
    assert len(emap) == 0 and info["n_confirmed"] == 0


def test_landmarks_recover_virtual_anchors_and_promotion_threshold():
    gt, vel = _straight()
    dets = _wall_detections(gt, APS, WALLS)
    est, emap, info = run_fastslam(dets, APS, vel, DT, np.random.default_rng(0),
                                   init_pose=gt[0], params=FastSlamParams(promote_min_obs=5))
    true_va = np.array([[a[0], 2 * w - a[1]] for a in APS for w in WALLS
                        if (a[1] - w) * (0 - w) > 0])
    lm = info["landmarks"]
    assert len(lm) == len(true_va)
    for va in true_va:
        assert np.min(np.linalg.norm(lm - va, axis=1)) < 0.5
    # mapped reflection points lie on the walls
    assert len(emap) > 0
    assert np.all(np.min(np.abs(emap[:, 1][:, None] - np.array(WALLS)[None]), axis=1) < 0.5)


def test_isolated_phantoms_never_promoted():
    gt, vel = _straight()
    dets = _wall_detections(gt, APS, WALLS)
    rng = np.random.default_rng(5)
    noisy = []
    for f, d in enumerate(dets):           # add 3 uniformly random phantoms per frame
        ph = np.column_stack([rng.uniform(5, 60, 3), rng.uniform(-np.pi, np.pi, 3),
                              rng.integers(0, 2, 3)])
        noisy.append(np.vstack([d, ph]))
    _, _, info = run_fastslam(noisy, APS, vel, DT, np.random.default_rng(0),
                              init_pose=gt[0], params=FastSlamParams(promote_min_obs=5))
    assert info["n_confirmed"] <= 4 + 1     # the 4 real anchors (+ at most one fluke)


def _excess_wall_detections(gt, aps, walls_y, rx_h=1.5):
    """Commodity-CSI convention: path length EXCESS over the LOS arrival, 3-D."""
    dets = []
    for p in gt:
        rows = []
        for i, a in enumerate(aps):
            dh = a[2] - rx_h
            los = np.sqrt(np.sum((a[:2] - p[:2]) ** 2) + dh ** 2)
            for w in walls_y:
                if (a[1] - w) * (p[1] - w) <= 0:
                    continue
                va = np.array([a[0], 2 * w - a[1]])
                d = va - p[:2]
                rows.append([np.sqrt(d @ d + dh ** 2) - los, np.arctan2(d[1], d[0]), i])
        dets.append(np.array(rows) if rows else np.empty((0, 3)))
    return dets


def test_excess_delay_mode_localizes_and_maps():
    gt, vel = _straight()
    dets = _excess_wall_detections(gt, APS, WALLS)
    gains = []
    for seed in range(3):
        v = noisy_velocity(vel, DT, OdometryNoise(), np.random.default_rng(seed))
        est, emap, info = run_fastslam(dets, APS, v, DT, np.random.default_rng(50 + seed),
                                       init_pose=gt[0], excess_delay=True)
        gains.append(ate(dead_reckon(v, DT, gt[0]), gt) - ate(est, gt))
    assert np.mean(gains) > 0, gains
    assert info["n_confirmed"] >= 3
    assert np.median(np.min(np.abs(emap[:, 1][:, None] - np.array(WALLS)[None]), axis=1)) < 0.5


def test_ambiguous_azimuth_recovers_folded_detections():
    """Single-ULA reports only the forward branch: fold every bearing to (-pi/2, pi/2]."""
    gt, vel = _straight()
    dets = _excess_wall_detections(gt, APS, WALLS)
    folded = [d.copy() for d in dets]
    for d in folded:
        if len(d):
            d[:, 1] = np.arcsin(np.sin(d[:, 1]))
    res = {}
    for amb in (False, True):
        g = []
        for seed in range(3):
            v = noisy_velocity(vel, DT, OdometryNoise(), np.random.default_rng(seed))
            est, _, _ = run_fastslam(folded, APS, v, DT, np.random.default_rng(70 + seed),
                                     init_pose=gt[0], excess_delay=True, ambiguous_azimuth=amb)
            g.append(ate(est, gt))
        res[amb] = np.mean(g)
    assert res[True] < res[False], res
