import numpy as np
from wifi_radar_slam.lidar.slam_icp import icp_align, _apply


def test_icp_recovers_known_transform():
    rng = np.random.default_rng(0)
    # an L-shaped point set (rotation is observable, not degenerate)
    source = np.array([[0, 0], [1, 0], [2, 0], [0, 1], [0, 2]], dtype=float)
    # small transform -> safely inside the point-to-point ICP basin from identity
    # (in SLAM, ICP is always seeded with the motion-predicted pose, i.e. close)
    gx, gy, gyaw = 0.4, -0.3, 0.15
    target = _apply(source, gx, gy, gyaw) + rng.normal(0, 1e-4, source.shape)
    x, y, yaw = icp_align(source, target, init=(0.0, 0.0, 0.0))
    assert np.isclose(x, gx, atol=1e-2)
    assert np.isclose(y, gy, atol=1e-2)
    assert np.isclose(yaw, gyaw, atol=1e-2)


def test_apply_identity():
    pts = np.array([[1.0, 2.0], [3.0, 4.0]])
    assert np.allclose(_apply(pts, 0.0, 0.0, 0.0), pts)
