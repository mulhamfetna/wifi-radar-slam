import numpy as np
from wifi_radar_slam.lidar.kitti import (load_velodyne_scan, load_gt_trajectory,
                                        align_2d_ate)


def test_velodyne_scan_slices_z_band(tmp_path):
    # 3 points: two inside z-band [-0.5,0.5], one above it
    pts = np.array([[1, 2, 0.0, 0.1], [3, 4, 0.4, 0.2], [5, 6, 2.0, 0.3]], dtype=np.float32)
    f = tmp_path / "000000.bin"
    pts.tofile(f)
    scan = load_velodyne_scan(str(f))
    assert len(scan) == 2
    assert np.allclose(np.sort(scan.points[:, 0]), [1.0, 3.0])


def test_velodyne_scan_voxel_downsamples(tmp_path):
    # 3 in-band points, two within one 0.5 m voxel -> downsample collapses them
    pts = np.array([[0.01, 0.0, 0.0, 0.1], [0.02, 0.01, 0.0, 0.2],
                    [9.0, 9.0, 0.0, 0.3]], dtype=np.float32)
    f = tmp_path / "000000.bin"
    pts.tofile(f)
    assert len(load_velodyne_scan(str(f))) == 3               # no downsample by default
    assert len(load_velodyne_scan(str(f), voxel=0.5)) == 2    # two collapse into one cell


def test_gt_trajectory_identity_calib():
    # two frames: identity rotation, translations along world x then z.
    # Tr = identity rotation, zero offset -> velo origin == pose translation.
    poses = "1 0 0 0 0 1 0 0 0 0 1 0\n1 0 0 2 0 1 0 0 0 0 1 5\n"
    calib = ("P0: 0 0 0 0 0 0 0 0 0 0 0 0\n"
             "Tr: 1 0 0 0 0 1 0 0 0 0 1 0\n")
    gt = load_gt_trajectory(poses, calib)
    assert gt.shape == (2, 2)
    assert np.allclose(gt[0], [0.0, 0.0])
    assert np.allclose(gt[1], [2.0, 5.0])          # (x, z)


def test_align_2d_ate_recovers_after_rigid_offset():
    gt = np.array([[0, 0], [1, 0], [2, 0], [2, 1]], dtype=float)
    # est = gt rotated 0.3 rad + translated; alignment should drive ATE ~0
    th = 0.3
    R = np.array([[np.cos(th), -np.sin(th)], [np.sin(th), np.cos(th)]])
    est = gt @ R.T + np.array([3.0, -2.0])
    assert align_2d_ate(est, gt) < 1e-6
