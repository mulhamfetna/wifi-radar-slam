import dataclasses
import numpy as np
import pytest
from wifi_radar_slam.radar.config import RadarConfig
from wifi_radar_slam.radar.processing import beat_matrix, range_fft

C = 299792458.0


def cfg_small(noise=0.0):
    """A small, fast config for unit tests. 1 GHz -> 15 cm range resolution.

    n_samples is 1024, not a token 128: the RadarConfig invariant forces
    n_samples >= 4*max_range*B/c = 800 here, or a 60 m target would alias back to a
    short range and manufacture a phantom out of the ADC alone.
    """
    return RadarConfig(carrier_hz=77e9, bandwidth_hz=1e9, chirp_time_s=40e-6,
                       n_samples=1024, n_chirps=1, n_rx=8, rx_spacing_frac=0.5,
                       n_azimuth=91, fov_deg=180.0, max_range_m=60.0, min_range_m=1.0,
                       cfar_guard_range=2, cfar_train_range=6,
                       cfar_guard_azimuth=2, cfar_train_azimuth=3,
                       pfa=1e-3, noise_sigma=noise)


def tau_of(range_m):
    """Monostatic round-trip delay for a target at `range_m`."""
    return 2.0 * range_m / C


# --- beat-signal synthesis ------------------------------------------------------

def test_beat_matrix_shape_and_dtype():
    cfg = cfg_small()
    b = beat_matrix([tau_of(20.0)], [1.0 + 0j], [0.0], cfg)
    assert b.shape == (cfg.n_rx, cfg.n_samples)
    assert np.iscomplexobj(b)


def test_beat_frequency_matches_the_target_range():
    # THE core physical check: a target at range R must produce a beat tone at
    # f_b = 2*R*S/c. We read the tone straight off an FFT of one antenna's row.
    cfg = cfg_small()
    R = 20.0
    b = beat_matrix([tau_of(R)], [1.0 + 0j], [0.0], cfg)
    spec = np.abs(np.fft.fft(b[0]))[: cfg.n_range]
    peak_bin = int(np.argmax(spec))
    f_b_expected = 2.0 * R * cfg.sweep_slope_hz_per_s / C
    bin_expected = f_b_expected * cfg.n_samples / cfg.sample_rate_hz
    assert peak_bin == pytest.approx(bin_expected, abs=1.0)


def test_a_farther_target_beats_higher():
    cfg = cfg_small()
    peaks = []
    for R in (10.0, 40.0):
        b = beat_matrix([tau_of(R)], [1.0 + 0j], [0.0], cfg)
        peaks.append(int(np.argmax(np.abs(np.fft.fft(b[0]))[: cfg.n_range])))
    assert peaks[1] > peaks[0]


def test_azimuth_appears_as_a_linear_phase_ramp_across_the_array():
    # A target at azimuth theta imposes phase 2*pi*d/lambda*m*sin(theta) on element m.
    # With half-wavelength spacing that is pi*m*sin(theta).
    cfg = cfg_small()
    theta = np.deg2rad(30.0)
    b = beat_matrix([tau_of(20.0)], [1.0 + 0j], [theta], cfg)
    d_phase = np.angle(b[1:, :] * np.conj(b[:-1, :]))
    expected = 2 * np.pi * cfg.rx_spacing_frac * np.sin(theta)
    assert np.mean(d_phase) == pytest.approx(expected, abs=1e-6)


def test_boresight_target_gives_a_flat_phase_front():
    cfg = cfg_small()
    b = beat_matrix([tau_of(20.0)], [1.0 + 0j], [0.0], cfg)
    d_phase = np.angle(b[1:, :] * np.conj(b[:-1, :]))
    assert np.allclose(d_phase, 0.0, atol=1e-9)


def test_paths_superpose_linearly():
    cfg = cfg_small()
    p1 = ([tau_of(10.0)], [1.0 + 0j], [0.0])
    p2 = ([tau_of(30.0)], [0.5 + 0j], [np.deg2rad(20)])
    both = beat_matrix(p1[0] + p2[0], p1[1] + p2[1], p1[2] + p2[2], cfg)
    assert np.allclose(both, beat_matrix(*p1, cfg) + beat_matrix(*p2, cfg))


def test_empty_path_list_gives_noise_only():
    cfg = cfg_small()
    b = beat_matrix([], [], [], cfg)
    assert b.shape == (cfg.n_rx, cfg.n_samples)
    assert np.allclose(b, 0.0)


def test_ragged_ray_arrays_are_rejected():
    cfg = cfg_small()
    with pytest.raises(ValueError, match="ragged"):
        beat_matrix([1e-7, 2e-7], [1.0 + 0j], [0.0], cfg)


def test_coherent_integration_scales_noise_by_sqrt_n_chirps():
    # n_chirps is the coherent-integration factor: 100x the chirps must cut the noise
    # std by 10x. This is the analytic stand-in for the chirp axis, exact on a static
    # scene -- and it is why we never depend on Sionna's synthetic within-CPI evolution.
    base = cfg_small(noise=1.0)
    many = dataclasses.replace(base, n_chirps=100)
    n1 = beat_matrix([], [], [], base, rng=np.random.default_rng(0))
    n2 = beat_matrix([], [], [], many, rng=np.random.default_rng(0))
    assert np.std(n2) == pytest.approx(np.std(n1) / 10.0, rel=1e-9)


# --- range FFT ------------------------------------------------------------------

def test_range_fft_shape():
    cfg = cfg_small()
    b = beat_matrix([tau_of(20.0)], [1.0 + 0j], [0.0], cfg)
    rf = range_fft(b, cfg)
    assert rf.shape == (cfg.n_rx, cfg.n_range)
    assert np.iscomplexobj(rf)


def test_range_fft_peak_lands_on_the_true_range():
    # The end-to-end range check: put a target at 20 m, read 20 m back out.
    cfg = cfg_small()
    R = 20.0
    rf = range_fft(beat_matrix([tau_of(R)], [1.0 + 0j], [0.0], cfg), cfg)
    power = np.abs(rf).sum(axis=0)
    peak_range = cfg.range_bins()[int(np.argmax(power))]
    assert peak_range == pytest.approx(R, abs=cfg.range_resolution_m)


def test_range_fft_resolves_two_targets_separated_by_more_than_a_cell():
    cfg = cfg_small()                       # 15 cm range cells
    rf = range_fft(beat_matrix([tau_of(20.0), tau_of(25.0)],
                               [1.0 + 0j, 1.0 + 0j], [0.0, 0.0], cfg), cfg)
    power = np.abs(rf).sum(axis=0)
    bins = cfg.range_bins()
    near20 = power[np.argmin(np.abs(bins - 20.0))]
    near25 = power[np.argmin(np.abs(bins - 25.0))]
    assert near20 > 0.5 * power.max() and near25 > 0.5 * power.max()


def test_windowing_suppresses_sidelobes():
    # A Hann window trades main-lobe width for sidelobe suppression -- essential, because
    # a strong target's rectangular-window sidelobes would otherwise trip CFAR and
    # masquerade as ghosts, contaminating the very rate we are measuring.
    cfg = cfg_small()
    b = beat_matrix([tau_of(20.0)], [1.0 + 0j], [0.0], cfg)
    windowed = np.abs(range_fft(b, cfg)).sum(axis=0)
    raw = np.abs(np.fft.fft(b, axis=1)[:, : cfg.n_range]).sum(axis=0)
    peak = int(np.argmax(windowed))
    far = np.r_[0:max(peak - 10, 0), min(peak + 11, cfg.n_range):cfg.n_range]
    assert (windowed[far].max() / windowed[peak]) < (raw[far].max() / raw[peak])
