import numpy as np
from wifi_radar_slam.lidar.config import LidarConfig, OUSTER_OS1


def test_bearings_span_and_count():
    cfg = LidarConfig(angular_res_deg=2.0, fov_deg=180.0, max_range_m=100.0,
                      min_range_m=0.5, range_sigma_m=0.03)
    b = cfg.bearings()
    assert cfg.n_beams == 90
    assert b.shape == (90,)
    # centred on 0, within +/- half-FOV
    assert np.isclose(b.mean(), 0.0, atol=1e-9)
    assert b.min() >= np.deg2rad(-90.0) - 1e-9
    assert b.max() <= np.deg2rad(90.0) + 1e-9


def test_ouster_preset_is_datasheet_pinned():
    assert OUSTER_OS1.max_range_m == 120.0
    assert OUSTER_OS1.range_sigma_m == 0.03
    assert OUSTER_OS1.fov_deg == 360.0
