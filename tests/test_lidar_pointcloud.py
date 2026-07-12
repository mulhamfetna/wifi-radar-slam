import numpy as np
from wifi_radar_slam.lidar.pointcloud import Scan


def test_to_world_rotates_then_translates():
    # a single local point 2 m straight ahead (+x local)
    scan = Scan(np.array([[2.0, 0.0]]))
    # pose at (1,1) facing +y (yaw=90deg): forward maps to +y in world
    w = scan.to_world((1.0, 1.0, np.pi / 2))
    assert np.allclose(w, [[1.0, 3.0]], atol=1e-9)


def test_from_ranges_drops_non_finite():
    bearings = np.array([0.0, np.pi / 2])
    ranges = np.array([np.inf, 4.0])          # first beam is a miss
    scan = Scan.from_ranges(bearings, ranges)
    assert len(scan) == 1
    assert np.allclose(scan.points, [[0.0, 4.0]], atol=1e-9)


def test_empty_scan():
    assert len(Scan.empty()) == 0
    assert Scan.empty().points.shape == (0, 2)
