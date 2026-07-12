import numpy as np
import pytest
from wifi_radar_slam.radar.kstrongest import k_strongest, k_strongest_from_cfg
from wifi_radar_slam.radar.config import RADAR_77G_4G
from wifi_radar_slam.lidar.pointcloud import Scan


def grids(n_az=8, n_rg=100, res=1.0):
    azimuths = np.linspace(-np.pi / 2, np.pi / 2, n_az)
    ranges = (np.arange(n_rg) + 0.5) * res
    return azimuths, ranges


def test_picks_the_single_strongest_return_per_azimuth():
    az, rg = grids()
    power = np.zeros((len(az), len(rg)))
    power[:, 30] = 5.0                      # every azimuth has one target at bin 30
    scan = k_strongest(power, rg, az, k=1)
    assert isinstance(scan, Scan)
    assert len(scan) == len(az)             # exactly one point per azimuth
    r = np.linalg.norm(scan.points, axis=1)
    assert np.allclose(r, rg[30], atol=1e-9)


def test_returns_at_most_k_per_azimuth():
    az, rg = grids()
    rng = np.random.default_rng(0)
    power = rng.random((len(az), len(rg)))  # dense noise: every bin is a candidate
    scan = k_strongest(power, rg, az, k=3)
    assert len(scan) <= 3 * len(az)


def test_the_strongest_win():
    az, rg = grids(n_az=1)
    power = np.zeros((1, len(rg)))
    power[0, 10] = 1.0
    power[0, 50] = 9.0                      # strongest
    power[0, 80] = 5.0
    scan = k_strongest(power, rg, az, k=2)
    r = np.sort(np.linalg.norm(scan.points, axis=1))
    assert np.allclose(r, [rg[50], rg[80]], atol=1e-9)   # the 1.0 peak is dropped


def test_azimuth_maps_to_the_right_bearing():
    az = np.array([0.0, np.pi / 2])
    rg = (np.arange(50) + 0.5) * 1.0
    power = np.zeros((2, 50))
    power[0, 9] = 1.0                       # 9.5 m dead ahead
    power[1, 9] = 1.0                       # 9.5 m to the left (+y)
    scan = k_strongest(power, rg, az, k=1)
    pts = scan.points[np.argsort(scan.points[:, 1])]
    assert np.allclose(pts[1], [0.0, 9.5], atol=1e-6)     # +90 deg -> +y
    assert np.allclose(pts[0], [9.5, 0.0], atol=1e-6)     # 0 deg   -> +x


def test_range_gating():
    az, rg = grids(n_rg=200)
    power = np.zeros((len(az), 200))
    power[:, 0] = 9.0                       # 0.5 m -- inside the blind zone
    power[:, 150] = 9.0                     # 150.5 m
    scan = k_strongest(power, rg, az, k=2, min_range_m=2.0, max_range_m=100.0)
    assert len(scan) == 0                   # both gated out


def test_z_min_threshold_rejects_the_noise_floor():
    az, rg = grids()
    power = np.full((len(az), len(rg)), 0.1)    # a flat noise floor
    power[:, 40] = 7.0                          # one real target
    scan = k_strongest(power, rg, az, k=5, z_min=1.0)
    assert len(scan) == len(az)                 # only the real target survives, once per azimuth


def test_empty_power_map_gives_an_empty_scan():
    az, rg = grids()
    scan = k_strongest(np.zeros((len(az), len(rg))), rg, az, k=4, z_min=0.5)
    assert isinstance(scan, Scan) and len(scan) == 0


def test_from_cfg_uses_the_configs_own_grids():
    # The simulated radar must go through the IDENTICAL extractor as the real radar, or the
    # front-end becomes confounded with the sensor difference we are trying to measure.
    cfg = RADAR_77G_4G
    ra = np.zeros((cfg.n_azimuth, cfg.n_range))
    i = int(np.argmin(np.abs(cfg.range_bins() - 30.0)))
    ra[cfg.n_azimuth // 2, i] = 10.0
    scan = k_strongest_from_cfg(ra, cfg, k=1)
    assert len(scan) == 1
    assert np.linalg.norm(scan.points[0]) == pytest.approx(30.0, abs=cfg.range_resolution_m)


def test_mismatched_grid_shapes_are_rejected():
    az, rg = grids()
    with pytest.raises(ValueError):
        k_strongest(np.zeros((3, 7)), rg, az, k=1)
