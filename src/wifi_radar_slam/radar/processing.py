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

C = 299792458.0

# Paths are synthesized in blocks so that the intermediate (n_paths, n_samples) matrix
# stays bounded: a Sionna monostatic solve returns thousands of paths, and at 8192 ADC
# samples the full matrix would run to hundreds of megabytes.
_PATH_BLOCK = 256


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
    """(n_azimuth, n_rx) conventional ULA steering vectors over the azimuth grid."""
    m = np.arange(cfg.n_rx)
    th = cfg.azimuth_grid()
    return np.exp(2j * np.pi * cfg.rx_spacing_frac * m[None, :] * np.sin(th)[:, None])


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
    A = steering_matrix(cfg)                 # (n_azimuth, n_rx)
    return np.abs(A.conj() @ rf) ** 2        # (n_azimuth, n_range)
