import numpy as np
from scipy.spatial import cKDTree
from wifi_radar_slam.fusion import _lidar_likelihood, fuse_loose


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
