import numpy as np
from wifi_radar_slam.lidar.config import LidarConfig
from wifi_radar_slam.lidar.sensor_sionna import _voxel_downsample, vertices_to_scan


def _cfg(sigma=0.0):
    return LidarConfig(angular_res_deg=2.0, fov_deg=360.0, max_range_m=50.0,
                       min_range_m=0.5, range_sigma_m=sigma)


def test_voxel_downsample_collapses_duplicates():
    pts = np.array([[0.01, 0.0], [0.02, 0.01], [5.0, 5.0]])   # first two share a 0.2 cell
    out = _voxel_downsample(pts, 0.2)
    assert out.shape[0] == 2


def test_vertices_to_scan_world_to_local_and_range_filter():
    # hits at 5 m ahead and 80 m away; sensor at origin facing +x
    hits = np.array([[5.0, 0.0], [80.0, 0.0]])
    scan = vertices_to_scan(hits, (0.0, 0.0, 0.0), _cfg(), np.random.default_rng(0))
    # far hit dropped by max_range; near hit kept, local == world here
    assert len(scan) == 1
    assert np.allclose(scan.points, [[5.0, 0.0]], atol=1e-9)


def test_vertices_to_scan_respects_yaw():
    # hit 5 m north in world; sensor facing +y (yaw=90deg) -> straight ahead locally
    hits = np.array([[0.0, 5.0]])
    scan = vertices_to_scan(hits, (0.0, 0.0, np.pi / 2), _cfg(), np.random.default_rng(0))
    assert np.allclose(scan.points, [[5.0, 0.0]], atol=1e-9)


def test_vertices_to_scan_empty_is_empty():
    assert len(vertices_to_scan(np.empty((0, 2)), (0, 0, 0), _cfg(),
                                np.random.default_rng(0))) == 0
