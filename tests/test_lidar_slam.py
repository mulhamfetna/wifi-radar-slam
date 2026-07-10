import numpy as np
from wifi_radar_slam.lidar.pointcloud import Scan
from wifi_radar_slam.lidar.slam_icp import run_lidar_slam


def test_recovers_straight_trajectory_from_scans():
    # world: a wall of points at x = 6, y in [-4, 4]; vehicle drives +x from origin
    rng = np.random.default_rng(0)
    wall = np.array([[6.0, y] for y in np.linspace(-4, 4, 40)])
    n, dt, speed = 12, 0.1, 3.0
    gt = np.array([[speed * dt * f, 0.0, 0.0] for f in range(n)])
    velocity = np.tile([speed, 0.0], (n, 1))
    # each scan = wall expressed in the sensor-local frame at the GT pose
    scans = []
    for f in range(n):
        rel = wall - gt[f, :2]                          # yaw=0 -> local == world-shifted
        scans.append(Scan(rel + rng.normal(0, 1e-3, rel.shape)))
    est_traj, est_map = run_lidar_slam(scans, velocity, dt, rng, init_pose=gt[0])
    ate = np.sqrt(np.mean(np.sum((est_traj[:, :2] - gt[:, :2]) ** 2, axis=1)))
    assert ate < 0.5                                    # tracks the straight drive
    assert est_map.shape[0] > 0
    # mapped points sit on the wall (x ~ 6)
    assert np.abs(est_map[:, 0].mean() - 6.0) < 0.5
