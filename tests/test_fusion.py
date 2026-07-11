import numpy as np
from scipy.spatial import cKDTree
from wifi_radar_slam.fusion import _lidar_likelihood, fuse_loose, run_fused_slam
from wifi_radar_slam.lidar.pointcloud import Scan


def test_lidar_likelihood_peaks_at_the_true_pose():
    # map: a wall of points at x=5; scan (local) sees that wall from the origin
    wall = np.array([[5.0, y] for y in np.linspace(-2, 2, 21)])
    tree = cKDTree(wall)
    scan_local = wall.copy()                      # sensor at origin, yaw 0 -> local == world
    # particle 0 is the true pose; particle 1 is displaced 3 m
    particles = np.array([[0.0, 0.0, 0.0], [3.0, 0.0, 0.0]])
    lik = _lidar_likelihood(particles, scan_local, tree, sigma=0.5)
    assert lik.shape == (2,)
    assert lik[0] > lik[1]                        # true pose is far more likely
    assert lik[0] > 0.9                           # near-perfect match


def test_fuse_loose_averages_traj_and_unions_maps():
    wifi_traj = np.array([[0.0, 0.0, 0.0], [2.0, 0.0, 0.0]])
    lidar_traj = np.array([[0.0, 2.0, 0.1], [4.0, 2.0, 0.1]])
    wifi_map = np.array([[10.0, 3.0]])
    lidar_map = np.array([[5.0, 0.0], [5.0, 1.0]])
    traj, m = fuse_loose(wifi_traj, wifi_map, lidar_traj, lidar_map, voxel=0.5)
    # x,y are the equal-weight average; yaw comes from the LiDAR back-end
    assert np.allclose(traj[:, :2], [[0.0, 1.0], [3.0, 1.0]])
    assert np.allclose(traj[:, 2], [0.1, 0.1])
    # map is the union of both modalities
    assert m.shape[0] == 3
    assert any(np.allclose(p, [10.0, 3.0]) for p in m)     # the WiFi reflector survived
    assert any(np.allclose(p, [5.0, 0.0]) for p in m)      # a LiDAR point survived


def test_fuse_loose_handles_empty_maps():
    traj, m = fuse_loose(np.zeros((2, 3)), np.empty((0, 2)),
                         np.zeros((2, 3)), np.empty((0, 2)))
    assert traj.shape == (2, 3)
    assert m.shape == (0, 2)


def _synthetic_case(n=12, dt=0.1, speed=3.0, with_wifi=True, with_lidar=True):
    """Straight +x drive past a box of walls, with one WiFi reflector at (10, 3)."""
    gt = np.array([[speed * dt * f, 0.0, 0.0] for f in range(n)])
    velocity = np.tile([speed, 0.0], (n, 1))
    aps = [np.array([0.0, 20.0, 6.0])]
    ap_xy = aps[0][:2]
    refl = np.array([10.0, 3.0])
    # box of wall points enclosing the drive (gives scan-match full constraint)
    xs, ys = np.linspace(-2, 12, 60), np.linspace(-4, 4, 40)
    box = np.vstack([np.column_stack([xs, np.full_like(xs, -4.0)]),
                     np.column_stack([xs, np.full_like(xs, 4.0)]),
                     np.column_stack([np.full_like(ys, -2.0), ys]),
                     np.column_stack([np.full_like(ys, 12.0), ys])])
    detections, scans = [], []
    for f in range(n):
        if with_wifi:
            d = refl - gt[f, :2]
            path = np.linalg.norm(ap_xy - refl) + np.linalg.norm(d)
            detections.append(np.array([[path, np.arctan2(d[1], d[0]), 0.0]]))
        else:
            detections.append(np.empty((0, 3)))
        scans.append(Scan(box - gt[f, :2]) if with_lidar else Scan.empty())
    return gt, velocity, aps, detections, scans, refl, box


def test_fused_slam_recovers_straight_trajectory():
    gt, vel, aps, dets, scans, _, _ = _synthetic_case()
    est, _ = run_fused_slam(dets, scans, aps, vel, 0.1, np.random.default_rng(0),
                            init_pose=gt[0])
    ate = np.sqrt(np.mean(np.sum((est[:, :2] - gt[:, :2]) ** 2, axis=1)))
    assert ate < 0.5


def test_fused_map_contains_both_modalities():
    gt, vel, aps, dets, scans, refl, box = _synthetic_case()
    _, est_map = run_fused_slam(dets, scans, aps, vel, 0.1, np.random.default_rng(0),
                                init_pose=gt[0])
    assert est_map.shape[0] > 0
    # a WiFi-triangulated reflector near (10,3) survived the union
    assert np.min(np.linalg.norm(est_map - refl, axis=1)) < 1.0
    # and LiDAR wall points survived too (e.g. the y=-4 wall)
    assert np.min(np.linalg.norm(est_map - np.array([5.0, -4.0]), axis=1)) < 1.0


def test_graceful_degradation_when_a_modality_is_missing():
    # LiDAR only (no WiFi detections at all)
    gt, vel, aps, dets, scans, _, _ = _synthetic_case(with_wifi=False)
    est, _ = run_fused_slam(dets, scans, aps, vel, 0.1, np.random.default_rng(0),
                            init_pose=gt[0])
    assert np.all(np.isfinite(est))
    # WiFi only (all scans empty)
    gt, vel, aps, dets, scans, _, _ = _synthetic_case(with_lidar=False)
    est, _ = run_fused_slam(dets, scans, aps, vel, 0.1, np.random.default_rng(0),
                            init_pose=gt[0])
    assert np.all(np.isfinite(est))
