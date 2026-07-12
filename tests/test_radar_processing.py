import dataclasses
import numpy as np
import pytest
from wifi_radar_slam.radar.config import RadarConfig
from wifi_radar_slam.radar.processing import (beat_matrix, range_fft, azimuth_beamform,
                                              cfar_2d, cluster_detections)

C = 299792458.0


def cfg_small(noise=0.0):
    """A small, fast config for unit tests. 1 GHz -> 15 cm range resolution.

    n_samples is 1024, not a token 128: the RadarConfig invariant forces
    n_samples >= 4*max_range*B/c = 800 here, or a 60 m target would alias back to a
    short range and manufacture a phantom out of the ADC alone.

    cfar_guard_azimuth is 10, not 2, for the same class of reason: this array is only 8
    elements, so its tapered beam is 0.40 wide in u = sin(azimuth), and a guard band
    narrower than half the beam lets a target mask itself and fragment into several
    detections. RadarConfig enforces both invariants.
    """
    return RadarConfig(carrier_hz=77e9, bandwidth_hz=1e9, chirp_time_s=40e-6,
                       n_samples=1024, n_chirps=1, n_rx=8, rx_spacing_frac=0.5,
                       n_azimuth=91, fov_deg=180.0, max_range_m=60.0, min_range_m=1.0,
                       cfar_guard_range=4, cfar_train_range=8,
                       cfar_guard_azimuth=10, cfar_train_azimuth=8,
                       pfa=1e-5, noise_sigma=noise)


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


# --- azimuth beamforming --------------------------------------------------------

def _ra_map(cfg, ranges, azimuths, amps=None, rng=None):
    amps = amps if amps is not None else [1.0 + 0j] * len(ranges)
    b = beat_matrix([tau_of(r) for r in ranges], amps, azimuths, cfg, rng=rng)
    return azimuth_beamform(range_fft(b, cfg), cfg)


def test_beamform_shape_and_realness():
    cfg = cfg_small()
    ra = _ra_map(cfg, [20.0], [0.0])
    assert ra.shape == (cfg.n_azimuth, cfg.n_range)
    assert np.isrealobj(ra)
    assert np.all(ra >= 0)


def test_beamform_peak_lands_on_the_true_range_and_azimuth():
    # THE end-to-end check for the whole front half of the chain.
    cfg = cfg_small()
    R, th = 25.0, np.deg2rad(20.0)
    ra = _ra_map(cfg, [R], [th])
    j, i = np.unravel_index(int(np.argmax(ra)), ra.shape)
    assert cfg.range_bins()[i] == pytest.approx(R, abs=cfg.range_resolution_m)
    # an 8-element half-wavelength ULA has a coarse beam (~13 deg at boresight, worse
    # off-boresight), so allow a beamwidth of slack
    assert np.rad2deg(cfg.azimuth_grid()[j]) == pytest.approx(20.0, abs=8.0)


def test_beamform_separates_two_targets_at_the_same_range():
    cfg = cfg_small()
    ra = _ra_map(cfg, [25.0, 25.0], [np.deg2rad(-40.0), np.deg2rad(40.0)])
    i = int(np.argmin(np.abs(cfg.range_bins() - 25.0)))
    col = ra[:, i]
    az = np.rad2deg(cfg.azimuth_grid())
    left = col[az < 0].max()
    right = col[az > 0].max()
    middle = col[np.abs(az) < 10].max()
    assert left > 3 * middle and right > 3 * middle     # two lobes, a null between


def test_beamform_is_symmetric_in_azimuth():
    # A sign error in sin(theta) would mirror every map left-right and put every
    # reflector on the wrong side of the road. Catch it here, not downstream.
    cfg = cfg_small()
    up = _ra_map(cfg, [25.0], [np.deg2rad(30.0)])
    dn = _ra_map(cfg, [25.0], [np.deg2rad(-30.0)])
    assert np.allclose(up, dn[::-1, :], atol=1e-9)


# --- CA-CFAR and detection clustering -------------------------------------------

def test_cfar_fires_on_a_target_and_not_on_empty_noise():
    cfg = cfg_small(noise=0.05)
    ra = _ra_map(cfg, [25.0], [0.0], rng=np.random.default_rng(0))
    mask = cfar_2d(ra, cfg)
    assert mask.dtype == bool
    assert mask.shape == ra.shape
    assert mask.any(), "CFAR must detect a clean, strong target"
    j, i = np.unravel_index(int(np.argmax(np.where(mask, ra, 0))), ra.shape)
    assert cfg.range_bins()[i] == pytest.approx(25.0, abs=1.0)


def test_cfar_false_alarm_rate_on_pure_noise_is_near_pfa():
    # An empty scene: every detection is BY DEFINITION a false alarm. This is the test
    # that makes the phantom-rate measurement trustworthy -- if the CFAR threshold were
    # miscalibrated, RQ1's headline number would be measuring our own processing.
    cfg = cfg_small(noise=1.0)
    ra = _ra_map(cfg, [], [], rng=np.random.default_rng(1))
    rate = cfar_2d(ra, cfg).mean()
    assert rate < 20 * cfg.pfa, f"CFAR false-alarm rate {rate:.2e} >> Pfa {cfg.pfa:.0e}"


def test_cfar_adapts_to_a_raised_noise_floor():
    # CA-CFAR's defining property: scale the whole map and the mask is unchanged.
    cfg = cfg_small(noise=0.05)
    ra = _ra_map(cfg, [25.0], [0.0], rng=np.random.default_rng(0))
    assert np.array_equal(cfar_2d(ra, cfg), cfar_2d(100.0 * ra, cfg))


def test_cluster_detections_collapses_one_target_to_one_detection():
    # A target spreads over several range/azimuth cells; without clustering it would be
    # counted as many detections and every downstream rate would be wrong.
    cfg = cfg_small(noise=0.02)
    ra = _ra_map(cfg, [25.0], [0.0], rng=np.random.default_rng(0))
    ranges, azimuths = cluster_detections(cfar_2d(ra, cfg), ra, cfg)
    assert len(ranges) == 1
    assert ranges[0] == pytest.approx(25.0, abs=1.0)
    assert np.rad2deg(azimuths[0]) == pytest.approx(0.0, abs=8.0)


def test_cluster_detections_finds_two_separated_targets():
    cfg = cfg_small(noise=0.02)
    ra = _ra_map(cfg, [15.0, 40.0], [np.deg2rad(-30.0), np.deg2rad(30.0)],
                 rng=np.random.default_rng(0))
    ranges, azimuths = cluster_detections(cfar_2d(ra, cfg), ra, cfg)
    assert len(ranges) == 2
    order = np.argsort(ranges)
    assert ranges[order][0] == pytest.approx(15.0, abs=1.5)
    assert ranges[order][1] == pytest.approx(40.0, abs=1.5)
    assert np.rad2deg(azimuths[order][0]) == pytest.approx(-30.0, abs=10.0)
    assert np.rad2deg(azimuths[order][1]) == pytest.approx(30.0, abs=10.0)


def test_cluster_detections_on_an_empty_mask_returns_empty_arrays():
    cfg = cfg_small()
    ra = _ra_map(cfg, [], [])
    ranges, azimuths = cluster_detections(np.zeros_like(ra, dtype=bool), ra, cfg)
    assert len(ranges) == 0 and len(azimuths) == 0


def test_off_boresight_targets_do_not_spawn_twins_at_the_field_of_view_edge():
    # REGRESSION GUARD for the CFAR azimuth boundary. The azimuth axis is PERIODIC (a
    # lambda/2 ULA cannot distinguish u = -1 from u = +1), so CFAR must wrap there. When it
    # treated the edge as a hard boundary, each of these two targets spawned a spurious
    # twin out at +/-80 deg -- pure fiction that would have been counted as a radar phantom
    # and would have corrupted RQ1. Exactly two targets in, exactly two detections out.
    cfg = cfg_small(noise=0.02)
    ra = _ra_map(cfg, [15.0, 40.0], [np.deg2rad(-30.0), np.deg2rad(30.0)],
                 rng=np.random.default_rng(0))
    ranges, azimuths = cluster_detections(cfar_2d(ra, cfg), ra, cfg)
    assert len(ranges) == 2
    edge = [np.rad2deg(a) for a in azimuths if abs(np.rad2deg(a)) > 60.0]
    assert not edge, f"field-of-view-edge phantoms at {edge} deg"


def test_one_strong_target_does_not_spawn_sidelobe_phantoms():
    # REGRESSION GUARD, and it guards the paper's headline. Without the Chebyshev array
    # taper, a single point target at 25 m / 0 deg lit up CFAR at +/-22, +/-39 and +/-61
    # degrees -- all at 25 m, all pure fiction, all manufactured by our own beamformer.
    # Those would have been counted as radar "phantoms" and would have corrupted RQ1's
    # answer to "is the 89 % phantom ceiling universal?". Exactly one detection, please.
    cfg = cfg_small(noise=0.02)
    ra = _ra_map(cfg, [25.0], [0.0], rng=np.random.default_rng(0))
    ranges, azimuths = cluster_detections(cfar_2d(ra, cfg), ra, cfg)
    assert len(ranges) == 1
    spurious = [np.rad2deg(a) for a in azimuths if abs(np.rad2deg(a)) > 12.0]
    assert not spurious, f"beamformer sidelobes became phantom detections at {spurious} deg"


def test_the_array_taper_actually_suppresses_sidelobes():
    # Pin the taper's design sidelobe level. Measured on a DENSE u = sin(theta) grid, not
    # on the coarse azimuth bins, so it tests the aperture rather than the sampling.
    from wifi_radar_slam.radar.processing import steering_matrix, TAPER_SIDELOBE_DB
    cfg = cfg_small()
    taper = np.abs(steering_matrix(cfg)[cfg.n_azimuth // 2])   # the window itself
    m = np.arange(cfg.n_rx)
    u = np.linspace(-1, 1, 20001)
    af = np.abs(np.exp(2j * np.pi * cfg.rx_spacing_frac * m[None, :] * u[:, None]) @ taper)
    af /= af.max()
    c = int(np.argmax(af))
    i = c
    while i + 1 < len(af) and af[i + 1] < af[i]:
        i += 1                                  # descend to the first null
    peak_sidelobe_db = 20 * np.log10(af[i + 1:].max())
    assert peak_sidelobe_db <= -TAPER_SIDELOBE_DB + 1.0
