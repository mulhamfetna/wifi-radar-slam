import numpy as np
from wifi_radar_slam.lidar.config import LidarConfig
from wifi_radar_slam.lidar.sensor_geo import _ray_segments_scan


def _cfg():
    return LidarConfig(angular_res_deg=2.0, fov_deg=360.0, max_range_m=100.0,
                       min_range_m=0.5, range_sigma_m=0.0)


def test_beam_hits_wall_at_correct_range():
    # vertical wall segment at x=5, y in [-2,2]; sensor at origin facing +x
    wall = np.array([[[5.0, -2.0], [5.0, 2.0]]])
    scan = _ray_segments_scan(wall, (0.0, 0.0, 0.0), _cfg(), np.random.default_rng(0))
    world = scan.to_world((0.0, 0.0, 0.0))
    assert len(scan) > 0
    # the forward (bearing~0) beam return sits on the wall at x=5
    fwd = world[np.argmin(np.abs(world[:, 1]))]
    assert np.isclose(fwd[0], 5.0, atol=1e-6)


def test_nearest_hit_wins_occlusion():
    # two parallel walls at x=5 and x=8; the near one occludes the far one
    walls = np.array([[[5.0, -2.0], [5.0, 2.0]], [[8.0, -2.0], [8.0, 2.0]]])
    scan = _ray_segments_scan(walls, (0.0, 0.0, 0.0), _cfg(), np.random.default_rng(0))
    world = scan.to_world((0.0, 0.0, 0.0))
    fwd = world[np.argmin(np.abs(world[:, 1]))]
    assert np.isclose(fwd[0], 5.0, atol=1e-6)          # never 8


def test_no_segments_returns_empty():
    scan = _ray_segments_scan(np.empty((0, 2, 2)), (0.0, 0.0, 0.0), _cfg(),
                              np.random.default_rng(0))
    assert len(scan) == 0
