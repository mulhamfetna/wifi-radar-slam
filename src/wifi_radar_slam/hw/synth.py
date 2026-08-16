"""Synthetic ESP32 CSI — the ground truth for rung 0.

We build CSI from a KNOWN set of taps, add every corruption the real chip is documented to
impose, and then rung 0 demands the pipeline recover the taps we injected. If it cannot
recover its own injected taps, the bug is ours and no hardware would have helped.

Corruptions modelled (all [V] in the design doc Part 5 / Part 7):
  - the 3-bin DC notch and the 11 edge guard bins are zeroed (dead subcarriers);
  - an S-shaped receiver phase distortion + M-shaped amplitude ripple — the receiver's own
    frequency response, which MANUFACTURES a phantom tap and is STATIC per device (so it
    cancels in the plate-in/plate-out differential);
  - per-packet random common phase (residual CFO + PLL state) — subcarrier-independent;
  - per-packet random STO (packet-detection jitter) — a LINEAR phase ramp across
    subcarriers, i.e. a pure time shift of the whole CIR;
  - 8-bit I/Q quantisation (~48 dB raw dynamic range).
"""
from __future__ import annotations

from dataclasses import dataclass

import numpy as np

from .config import CSIConfig


@dataclass(frozen=True)
class Tap:
    """One propagation path: complex amplitude and delay (seconds)."""

    amplitude: complex
    delay_s: float


def ideal_csi(taps: list[Tap], cfg: CSIConfig) -> np.ndarray:
    """The clean channel: H[k] = sum_i a_i * exp(-j 2 pi k df tau_i).

    No notch, no distortion, no noise — the physical channel only. Returned in fftshifted
    index order (k in [-64, 63]).
    """
    k = cfg.k_grid.astype(float)
    df = cfg.subcarrier_spacing_hz
    h = np.zeros(cfg.n_subcarriers, dtype=complex)
    for t in taps:
        h += t.amplitude * np.exp(-2j * np.pi * k * df * t.delay_s)
    return h


def receiver_response(cfg: CSIConfig, strength: float = 0.35) -> np.ndarray:
    """The receiver's own frequency response G(f) — the phantom generator.

    An M-shaped amplitude ripple and an S-shaped (cubic) phase across the band. NOT linear
    in subcarrier index, so it is NOT removed by STO de-ramping — it survives a single
    recording and only cancels in the differential (because it is static per device). This
    is the ``Hdist`` / "causes a phantom object" effect (PicoScenes; arXiv:2605.26836).

    ``strength`` scales both the ripple depth and the phase-curve amplitude.
    """
    k = cfg.k_grid.astype(float)
    kn = k / (cfg.n_subcarriers / 2)                   # normalised to [-1, 1)
    amp = 1.0 + strength * np.cos(2.0 * np.pi * kn)     # M-shaped ripple
    phase = strength * np.pi * (kn ** 3)               # S-shaped (cubic) phase
    return amp * np.exp(1j * phase)


def apply_dead_bins(h: np.ndarray, cfg: CSIConfig) -> np.ndarray:
    """Zero the DC notch and the edge guard bins — what the ESP32 effectively delivers."""
    out = h.copy()
    out[cfg.dead_mask] = 0.0
    return out


def quantise_8bit(h: np.ndarray, full_scale: float) -> np.ndarray:
    """8-bit signed I/Q quantisation. ``full_scale`` maps to +127.

    Values are clipped to [-128, 127] per component, exactly as an 8-bit ADC would. The
    ~48 dB raw dynamic range this imposes is why a single packet cannot see a -46 dB echo
    and coherent averaging is mandatory (Part 4.5).
    """
    scale = 127.0 / full_scale
    i = np.clip(np.round(h.real * scale), -128, 127)
    q = np.clip(np.round(h.imag * scale), -128, 127)
    return (i + 1j * q) / scale


def synth_packet(
    taps: list[Tap],
    cfg: CSIConfig,
    *,
    rng: np.random.Generator,
    rx_response: np.ndarray | None = None,
    sto_std_s: float = 5e-9,
    noise_std: float = 0.0,
    quantise: bool = True,
    common_phase: bool = True,
) -> np.ndarray:
    """One packet of synthetic CSI, with the full per-packet corruption stack.

    Order matters and mirrors physics: channel -> receiver response -> STO ramp -> common
    phase -> additive noise -> dead bins -> quantise.

    ``rx_response`` lets the caller pin the SAME receiver response across packets and across
    the plate-in/plate-out recordings (it is static per device); pass ``None`` for the
    default. ``sto_std_s`` is the per-packet timing jitter; ``noise_std`` the per-subcarrier
    complex-noise std BEFORE any averaging.
    """
    if rx_response is None:
        rx_response = receiver_response(cfg)

    h = ideal_csi(taps, cfg)
    h = h * rx_response

    # STO: a pure time shift tau_o -> a linear phase ramp exp(-j 2 pi k df tau_o).
    tau_o = rng.normal(0.0, sto_std_s)
    k = cfg.k_grid.astype(float)
    h = h * np.exp(-2j * np.pi * k * cfg.subcarrier_spacing_hz * tau_o)

    # Common phase: subcarrier-independent rotation (residual CFO + PLL).
    if common_phase:
        h = h * np.exp(1j * rng.uniform(-np.pi, np.pi))

    if noise_std > 0.0:
        h = h + (rng.normal(0.0, noise_std, h.shape)
                 + 1j * rng.normal(0.0, noise_std, h.shape))

    h = apply_dead_bins(h, cfg)

    if quantise:
        full_scale = float(np.abs(h).max()) or 1.0
        h = quantise_8bit(h, full_scale)
        h = apply_dead_bins(h, cfg)   # quantisation can nudge a zeroed bin off zero

    return h


def synth_recording(
    taps: list[Tap],
    cfg: CSIConfig,
    *,
    n_packets: int,
    rng: np.random.Generator,
    **kwargs,
) -> np.ndarray:
    """A stack of ``n_packets`` packets sharing ONE receiver response (static per device).

    Returns an array of shape (n_packets, n_subcarriers). Every packet gets its own random
    STO and common phase; the receiver response is fixed so it cancels in a differential.
    """
    rx_response = kwargs.pop("rx_response", None)
    if rx_response is None:
        rx_response = receiver_response(cfg)
    return np.stack([
        synth_packet(taps, cfg, rng=rng, rx_response=rx_response, **kwargs)
        for _ in range(n_packets)
    ])
