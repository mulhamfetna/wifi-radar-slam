import numpy as np
import pytest
from wifi_radar_slam.radar.config import (RadarConfig, RADAR_77G_4G,
                                          RADAR_77G_160M, WIFI_5G2_160M)

C = 299792458.0


def test_range_resolution_is_c_over_2b():
    # The textbook FMCW range resolution. 4 GHz -> ~3.7 cm; 160 MHz -> ~94 cm.
    assert RADAR_77G_4G.range_resolution_m == pytest.approx(C / (2 * 4e9))
    assert RADAR_77G_4G.range_resolution_m == pytest.approx(0.0375, abs=1e-3)
    assert RADAR_77G_160M.range_resolution_m == pytest.approx(0.937, abs=1e-2)


def test_narrowband_radar_and_wifi_cell_share_a_range_resolution():
    # This is the whole point of the ablation: cells C and B differ ONLY in carrier,
    # so their range resolution must be identical.
    assert RADAR_77G_160M.range_resolution_m == pytest.approx(
        WIFI_5G2_160M.range_resolution_m)


def test_carrier_separates_the_cells():
    assert RADAR_77G_4G.carrier_hz == 77e9
    assert RADAR_77G_160M.carrier_hz == 77e9
    assert WIFI_5G2_160M.carrier_hz == 5.2e9


def test_wavelength():
    assert RADAR_77G_4G.wavelength_m == pytest.approx(C / 77e9)
    assert RADAR_77G_4G.wavelength_m == pytest.approx(0.0039, abs=1e-4)


def test_sweep_slope_and_sample_rate():
    cfg = RadarConfig(carrier_hz=77e9, bandwidth_hz=4e9, chirp_time_s=200e-6,
                      n_samples=8192, n_chirps=64, n_rx=16, rx_spacing_frac=0.5,
                      n_azimuth=181, fov_deg=180.0, max_range_m=100.0, min_range_m=1.0,
                      cfar_guard_range=2, cfar_train_range=8,
                      cfar_guard_azimuth=2, cfar_train_azimuth=4,
                      pfa=1e-4, noise_sigma=0.0)
    assert cfg.sweep_slope_hz_per_s == pytest.approx(4e9 / 200e-6)
    assert cfg.sample_rate_hz == pytest.approx(8192 / 200e-6)


def test_a_radar_that_aliases_its_own_max_range_is_rejected_at_construction():
    # 256 ADC samples cannot sample the beat frequency of a 100 m target under a 4 GHz
    # sweep: it would fold back and appear at a SHORT range where nothing exists. That is
    # a phantom manufactured by our own ADC, and the phantom rate is this paper's headline
    # measurement -- so the config must not be constructible at all.
    with pytest.raises(ValueError, match="aliases its own max range"):
        RadarConfig(carrier_hz=77e9, bandwidth_hz=4e9, chirp_time_s=200e-6,
                    n_samples=256, n_chirps=64, n_rx=16, rx_spacing_frac=0.5,
                    n_azimuth=181, fov_deg=180.0, max_range_m=100.0, min_range_m=1.0,
                    cfar_guard_range=2, cfar_train_range=8,
                    cfar_guard_azimuth=2, cfar_train_azimuth=4,
                    pfa=1e-4, noise_sigma=0.0)


def test_the_adc_sample_count_is_forced_by_range_times_bandwidth():
    # n_samples >= 4 * R_max * B / c, with the chirp time cancelling out. This is the
    # real hardware trade-off behind cell D and the reason the presets carry 8192 samples.
    for cfg in (RADAR_77G_4G, RADAR_77G_160M, WIFI_5G2_160M):
        need = 4.0 * cfg.max_range_m * cfg.bandwidth_hz / C
        assert cfg.n_samples >= need


def test_the_sample_rate_stays_physically_plausible():
    # A 41 MHz ADC is what a high-end automotive part delivers. If a preset ever demands
    # hundreds of MHz, it has stopped modelling real hardware.
    for cfg in (RADAR_77G_4G, RADAR_77G_160M, WIFI_5G2_160M):
        assert cfg.sample_rate_hz <= 50e6


def test_range_bins_are_monotone_and_start_at_zero():
    bins = RADAR_77G_4G.range_bins()
    assert bins.shape == (RADAR_77G_4G.n_range,)
    assert bins[0] == pytest.approx(0.0)
    assert np.all(np.diff(bins) > 0)


def test_range_bin_spacing_equals_range_resolution():
    # An n_samples-point FFT over the sweep gives bins spaced exactly c/(2B).
    bins = RADAR_77G_4G.range_bins()
    assert np.diff(bins)[0] == pytest.approx(RADAR_77G_4G.range_resolution_m, rel=1e-9)


def test_max_beat_range_covers_the_configured_max_range():
    # If the ADC cannot sample the beat frequency of a target at max_range_m,
    # that target aliases. The presets must not be self-contradictory.
    for cfg in (RADAR_77G_4G, RADAR_77G_160M, WIFI_5G2_160M):
        assert cfg.max_beat_range_m >= cfg.max_range_m


def test_azimuth_grid_spans_the_fov_symmetrically():
    grid = RADAR_77G_4G.azimuth_grid()
    assert grid.shape == (RADAR_77G_4G.n_azimuth,)
    assert grid[0] == pytest.approx(-np.deg2rad(RADAR_77G_4G.fov_deg) / 2)
    assert grid[-1] == pytest.approx(np.deg2rad(RADAR_77G_4G.fov_deg) / 2)
    assert np.all(np.diff(grid) > 0)


def test_config_is_frozen():
    with pytest.raises(Exception):
        RADAR_77G_4G.carrier_hz = 1.0
