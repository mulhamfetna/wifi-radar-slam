import numpy as np
from wifi_radar_slam.lidar.config import LidarConfig
from wifi_radar_slam.lidar.sensor_ref import ReferenceSensor, reference_sensor


def _cfg():
    return LidarConfig(angular_res_deg=2.0, fov_deg=360.0, max_range_m=100.0,
                       min_range_m=0.5, range_sigma_m=0.0)


def test_scan_sees_a_wall_at_correct_range():
    # dense wall of points at x = 5, y in [-2, 2]; sensor at origin facing +x
    wall = np.array([[5.0, y] for y in np.linspace(-2, 2, 41)])
    sensor = ReferenceSensor(wall, _cfg(), np.random.default_rng(0))
    scan = sensor((0.0, 0.0, 0.0))
    assert len(scan) > 0
    world = scan.to_world((0.0, 0.0, 0.0))
    # every returned point lies on the wall (x ~ 5) within the perpendicular gate
    assert np.all(np.abs(world[:, 0] - 5.0) < 0.6)


def test_factory_reads_ground_truth_map():
    class _Built:
        ground_truth_map = np.array([[5.0, 0.0, 0.0], [5.0, 1.0, 0.0]])
    s = reference_sensor(_Built(), _cfg(), np.random.default_rng(0))
    assert isinstance(s, ReferenceSensor)
    assert s((-1.0, 0.0, 0.0)) is not None
