"""The FMCW radar detection chain: beat signal -> range FFT -> azimuth beamforming -> CFAR.

PURE NumPy/SciPy. This module MUST NOT import Sionna or Mitsuba, so the whole chain is
unit-testable without the simulator.

Why the chain exists at all, in full: paper 3's headline question is whether the ~89 %
phantom-detection rate paper 2 measured on WiFi is a WiFi pathology or a property of RF
sensing. Reading ray-traced paths out of the simulator directly -- as the LiDAR model does
-- would hand radar **zero ghosts by construction** and rig that comparison. Ghosts and
false alarms have to *emerge* from finite bandwidth, a finite aperture and a calibrated
CFAR threshold, exactly as they do on real hardware. Hence: no shortcuts.
"""
from __future__ import annotations
import numpy as np
from scipy import ndimage
from scipy.signal.windows import chebwin

C = 299792458.0

# Paths are synthesized in blocks so that the intermediate (n_paths, n_samples) matrix
# stays bounded: a Sionna monostatic solve returns thousands of paths, and at 8192 ADC
# samples the full matrix would run to hundreds of megabytes.
_PATH_BLOCK = 256

# Dolph-Chebyshev array-taper sidelobe level. Set by measurement -- see steering_matrix.
TAPER_SIDELOBE_DB = 80.0


def beat_matrix(taus, amps, azimuths, cfg, rng=None) -> np.ndarray:
    """Synthesize the dechirped (beat) signal across the RX array.

    Args:
        taus:     round-trip propagation delays (s), one per ray, as a monostatic radar
                  measures them (i.e. 2*R/c for a single-bounce target at range R).
        amps:     complex path amplitudes, one per ray.
        azimuths: sensor-local arrival azimuths (rad), +x forward, positive toward +y.
        cfg:      RadarConfig.
        rng:      numpy Generator for receiver noise (None -> no noise).

    Returns:
        (cfg.n_rx, cfg.n_samples) complex beat matrix.

    An FMCW sweep of slope S, mixed with its own echo at delay tau, leaves the beat tone

        s(t) = a * exp(j*2*pi*(S*tau*t + f_c*tau - 0.5*S*tau**2))

    -- a tone whose frequency S*tau encodes range, plus a carrier phase f_c*tau. The
    residual-video-phase term -0.5*S*tau**2 is small but kept: it is free, and dropping it
    would be a silent approximation.

    There is no chirp axis. The scenes are static, so every chirp in the CPI carries an
    identical signal and differs only in noise; coherently integrating cfg.n_chirps of them
    is analytically identical to generating the signal once with the noise standard
    deviation divided by sqrt(cfg.n_chirps), which is what we do. This is exact here, is
    cfg.n_chirps times cheaper, and it means we never rely on Sionna's synthetic
    within-CPI time evolution (a documented pitfall) at all.
    """
    taus = np.asarray(taus, dtype=float).ravel()
    amps = np.asarray(amps, dtype=complex).ravel()
    azimuths = np.asarray(azimuths, dtype=float).ravel()
    if not (taus.shape == amps.shape == azimuths.shape):
        raise ValueError(f"ragged rays: {taus.shape}, {amps.shape}, {azimuths.shape}")

    t = np.arange(cfg.n_samples) / cfg.sample_rate_hz      # fast time (n_samples,)
    m = np.arange(cfg.n_rx)                                # array elements (n_rx,)

    beat = np.zeros((cfg.n_rx, cfg.n_samples), dtype=complex)
    S = cfg.sweep_slope_hz_per_s
    for lo in range(0, taus.size, _PATH_BLOCK):
        tb = taus[lo:lo + _PATH_BLOCK]
        ab = amps[lo:lo + _PATH_BLOCK]
        zb = azimuths[lo:lo + _PATH_BLOCK]
        # (n_block, n_samples): each ray's beat tone over fast time, amplitude folded in
        phase_t = 2 * np.pi * (S * tb[:, None] * t[None, :]
                               + cfg.carrier_hz * tb[:, None]
                               - 0.5 * S * tb[:, None] ** 2)
        tone = ab[:, None] * np.exp(1j * phase_t)
        # (n_rx, n_block): each ray's ULA steering phase
        steer = np.exp(2j * np.pi * cfg.rx_spacing_frac
                       * m[:, None] * np.sin(zb)[None, :])
        beat += steer @ tone                               # (n_rx, n_samples)

    if rng is not None and cfg.noise_sigma > 0:
        # Coherent integration over the CPI: sigma -> sigma / sqrt(n_chirps).
        sigma = cfg.noise_sigma / np.sqrt(cfg.n_chirps)
        beat = beat + (sigma / np.sqrt(2)) * (rng.normal(size=beat.shape)
                                              + 1j * rng.normal(size=beat.shape))
    return beat


def range_fft(beat: np.ndarray, cfg) -> np.ndarray:
    """Windowed FFT along fast time -> a complex range profile per array element.

    Returns (cfg.n_rx, cfg.n_range); bin i is at range cfg.range_bins()[i].

    The Hann window is not cosmetic. With a rectangular window, a strong target's spectral
    sidelobes (-13 dB, decaying slowly) sit well above the noise floor and trip CFAR at
    ranges where nothing exists -- manufacturing "ghosts" that are artifacts of our own
    processing rather than of the physics. Since the phantom rate is this paper's headline
    measurement, that contamination is disqualifying. Hann drops the first sidelobe to
    -31 dB and rolls off fast, at the cost of a ~2x wider main lobe.
    """
    w = np.hanning(cfg.n_samples)
    return np.fft.fft(beat * w[None, :], axis=1)[:, : cfg.n_range]


def steering_matrix(cfg) -> np.ndarray:
    """(n_azimuth, n_rx) TAPERED ULA steering vectors over the azimuth grid.

    The array taper is as load-bearing as the Hann window in range, and for the same
    reason. An untapered uniform aperture has -13 dB azimuth sidelobes: a single strong
    reflector then lights up CFAR at *its own range* but at half a dozen *wrong bearings*.
    We measured exactly that -- a noise-free point target at 25 m / 0 deg produced spurious
    detections at +/-22, +/-39 and +/-61 degrees, all at 25 m.

    Those are phantoms manufactured by our own beamformer, not by the physics. Letting them
    through would inflate radar's phantom rate for a reason that has nothing to do with RF
    sensing, and RQ1 -- is the ~89 % phantom ceiling universal? -- would end up measuring
    our sloppiness instead of the world.

    Dolph-Chebyshev at TAPER_SIDELOBE_DB, chosen by measurement rather than habit. On the
    single-target case above: uniform left 6 spurious detections, Hamming 2, Chebyshev-60
    one, Chebyshev-80 none. Chebyshev-100 removed nothing further and only widened the
    beam, so 80 dB is the minimum that does the job. The cost is real and is paid
    deliberately: it widens the 16-element array's 3 dB beam from 6.3 to 11.8 degrees. That
    is not crippling radar -- it is what real automotive arrays, which taper for exactly
    this reason, actually achieve -- and over-tapering would understate radar's angular
    resolution and bias the WiFi-vs-radar comparison in WiFi's favour.
    """
    m = np.arange(cfg.n_rx)
    u = cfg.u_grid()                      # uniform in sin(azimuth) -- the array's own axis
    taper = chebwin(cfg.n_rx, at=TAPER_SIDELOBE_DB)
    a = np.exp(2j * np.pi * cfg.rx_spacing_frac * m[None, :] * u[:, None])
    return a * taper[None, :]


def azimuth_beamform(rf: np.ndarray, cfg) -> np.ndarray:
    """Conventional (Bartlett) beamforming across the array -> a range-azimuth power map.

    Returns (cfg.n_azimuth, cfg.n_range), real and non-negative.

    Deliberately NOT a superresolution beamformer (MUSIC/Capon). Papers 1-2 used MUSIC on
    WiFi, and the whole point of this ablation is to hold the *detection algorithm* fixed
    across cells so that any difference between them is **physical** -- carrier, bandwidth,
    geometry -- and not an artifact of a different estimator. FFT+CFAR is also what real
    automotive radar actually runs. MUSIC-on-WiFi survives separately as the 5th reference
    row, which is precisely where the superresolution-vs-FFT axis becomes visible instead
    of silently confounded with the physics.
    """
    A = steering_matrix(cfg)                 # (n_azimuth, n_rx), tapered
    return np.abs(A.conj() @ rf) ** 2        # (n_azimuth, n_range)


def cfar_2d(ra_map: np.ndarray, cfg) -> np.ndarray:
    """2-D cell-averaging CFAR over the range-azimuth map. Returns a bool detection mask.

    Each cell under test is compared against the mean power of a rectangular ring of
    training cells surrounding it, with a guard band excluded so a target's own energy
    cannot inflate its own noise estimate. The threshold multiplier for an N-training-cell
    CA-CFAR at design false-alarm probability Pfa is the standard

        alpha = N * (Pfa**(-1/N) - 1)

    which makes the false-alarm rate *constant* regardless of the absolute noise level --
    the whole reason a radar uses CFAR instead of a fixed threshold. That property is
    load-bearing here: the phantom rate reported for RQ1 is only meaningful if the
    threshold is **calibrated**, not tuned to produce a pleasing number.

    Implemented as two box filters (the full window minus the guard window) rather than a
    per-cell loop -- an exact vectorized identity, and orders of magnitude faster on the
    181 x 4096 maps the presets produce.
    """
    gr, tr = cfg.cfar_guard_range, cfg.cfar_train_range
    ga, ta = cfg.cfar_guard_azimuth, cfg.cfar_train_azimuth
    full = (2 * (ga + ta) + 1, 2 * (gr + tr) + 1)      # (azimuth, range)
    guard = (2 * ga + 1, 2 * gr + 1)

    n_full = full[0] * full[1]
    n_guard = guard[0] * guard[1]
    n_train = n_full - n_guard
    if n_train <= 0:
        raise ValueError("CFAR training region is empty; increase cfar_train_*")

    # Boundary handling differs per axis, and getting it wrong manufactures phantoms.
    # AZIMUTH is PERIODIC: a lambda/2 ULA's array factor depends on the elements only
    # through u = sin(theta), and u in [-1, 1] spans exactly one period -- u = -1 and
    # u = +1 are literally the same steering vector. So the correct extension is "wrap".
    # Treating it as a hard edge ("nearest") corrupts the noise estimate near u = +/-1 and
    # fires CFAR there: we measured it, and two targets at +/-30 deg each spawned a
    # spurious twin at the far edge of the field of view. RANGE is NOT periodic -- bin 0
    # and bin n are unrelated -- so it stays "nearest".
    modes = ("wrap", "nearest")
    sum_full = ndimage.uniform_filter(ra_map, size=full, mode=modes) * n_full
    sum_guard = ndimage.uniform_filter(ra_map, size=guard, mode=modes) * n_guard
    noise = (sum_full - sum_guard) / n_train

    alpha = n_train * (cfg.pfa ** (-1.0 / n_train) - 1.0)
    return ra_map > alpha * noise


def cluster_detections(mask: np.ndarray, ra_map: np.ndarray, cfg):
    """Collapse each connected blob of CFAR hits to one power-weighted centroid detection.

    Returns (ranges_m, azimuths_rad), both 1-D, one entry per blob.

    A single physical target lights up several range and azimuth cells -- the main lobe is
    wider than one cell by construction (see the Hann window in range_fft). Counting those
    cells individually would multiply every detection count by the beam footprint and
    corrupt the phantom rate, so blobs are merged before anything downstream sees them.
    """
    if not mask.any():
        return np.empty(0), np.empty(0)
    labels, n = ndimage.label(mask)
    idx = np.arange(1, n + 1)
    weights = np.where(mask, ra_map, 0.0)
    cent = ndimage.center_of_mass(weights, labels, idx)    # [(az_idx, rng_idx), ...]
    az_i = np.array([c[0] for c in cent])
    rg_i = np.array([c[1] for c in cent])
    # fractional bin indices -> physical units by linear interpolation on the grids
    ranges = np.interp(rg_i, np.arange(cfg.n_range), cfg.range_bins())
    azimuths = np.interp(az_i, np.arange(cfg.n_azimuth), cfg.azimuth_grid())
    return ranges, azimuths
