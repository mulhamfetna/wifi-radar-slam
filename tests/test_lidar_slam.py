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


def test_adaptive_velocity_none_tracks_straight_trajectory():
    # velocity=None -> adaptive constant-velocity motion model (frame-agnostic).
    # Use a box of walls (features on all sides) so pure ICP is well-constrained.
    rng = np.random.default_rng(0)
    ys = np.linspace(-4, 4, 40)
    xs = np.linspace(-2, 12, 60)
    box = np.vstack([np.column_stack([xs, np.full_like(xs, -4.0)]),
                     np.column_stack([xs, np.full_like(xs, 4.0)]),
                     np.column_stack([np.full_like(ys, -2.0), ys]),
                     np.column_stack([np.full_like(ys, 12.0), ys])])
    n, dt, speed = 12, 0.1, 3.0
    gt = np.array([[speed * dt * f, 0.0, 0.0] for f in range(n)])
    scans = [Scan(box - gt[f, :2] + rng.normal(0, 1e-3, box.shape)) for f in range(n)]
    est, _ = run_lidar_slam(scans, None, dt, rng, init_pose=gt[0])
    ate = np.sqrt(np.mean(np.sum((est[:, :2] - gt[:, :2]) ** 2, axis=1)))
    assert ate < 0.5


def test_progress_callback_invoked_per_frame():
    rng = np.random.default_rng(0)
    wall = np.array([[6.0, y] for y in np.linspace(-4, 4, 40)])
    n, dt, speed = 5, 0.1, 3.0
    gt = np.array([[speed * dt * f, 0.0, 0.0] for f in range(n)])
    velocity = np.tile([speed, 0.0], (n, 1))
    scans = [Scan(wall - gt[f, :2]) for f in range(n)]
    seen = []
    run_lidar_slam(scans, velocity, dt, rng, init_pose=gt[0],
                   progress=lambda f, nn, ns, nm: seen.append((f, nn, ns, nm)))
    assert [s[0] for s in seen] == [1, 2, 3, 4]          # one call per frame after 0
    assert all(s[1] == n and s[2] > 0 and s[3] > 0 for s in seen)
