"""The delay-domain pipeline: CSI -> aligned |CIR|. THE ORDER IS NOT OPTIONAL (doc Part 5).

Per packet:    prepare_packet  = to CIR -> find LOS peak -> divide by it -> shift LOS to bin 0
Per recording: coherent_average = mean of prepared CIRs
Differential:  averaged(plate_in) - averaged(plate_out)

Why align to the LOS PEAK, not the mean phase slope:
Each packet carries an independent random common phase (CFO + PLL) and an independent STO
(a pure time shift of the whole CIR). We must remove both so packets are mutually coherent.
Dividing by the LOS tap's complex value kills the common phase AND the amplitude scaling;
circularly shifting the LOS to bin 0 kills the STO time shift. Crucially, aligning to the
LOS *peak* is robust to a strong echo — a mean-slope de-ramp is pulled by the echo and would
shift plate-in and plate-out differently, misaligning the very subtraction the method relies
on. This is the LoS-referencing method (WiROS / arXiv:2602.05344), validated at 160 MHz.

After alignment the LOS sits at delay 0, so every detected tap position IS its excess delay.
"""
from __future__ import annotations

import numpy as np
from scipy.signal.windows import hann

from .config import CSIConfig


def _interpolate_dead(h: np.ndarray, cfg: CSIConfig) -> np.ndarray:
    """Linearly interpolate the DC-notch bins across the gap (real and imag separately).

    Guard bins stay zero (the window tapers them out); only the 3-bin centre notch is
    filled, exactly as ESPARGOS does, so the IFFT sees a contiguous centre.
    """
    out = h.copy()
    k = cfg.k_grid
    notch = np.isin(k, np.array(cfg.dc_notch_k))
    if not notch.any():
        return out
    good = ~notch & cfg.active_mask
    xi = np.where(good)[0]
    xo = np.where(notch)[0]
    out[xo] = (np.interp(xo, xi, h[good].real)
               + 1j * np.interp(xo, xi, h[good].imag))
    return out


def raw_cir(h: np.ndarray, cfg: CSIConfig, zero_pad: int = 16) -> np.ndarray:
    """Interpolate the notch, Hann-window the active band, zero-pad, IFFT -> complex CIR.

    No referencing or alignment — the plain channel impulse response of one CSI vector.
    Used by the ordering sanity check (Rung 0.5): an empty corridor must show one tap.
    """
    h = _interpolate_dead(h, cfg)
    w = np.zeros(cfg.n_subcarriers)
    w[cfg.active_mask] = hann(int(cfg.active_mask.sum()))
    hw = np.fft.ifftshift(h * w)          # -> FFT order [f0..f63, f-64..f-1]

    # Zero-pad IN THE MIDDLE (at Nyquist), NOT at the end. np.fft.ifft(a, n>len) appends
    # zeros to the end of the array, which for a two-sided spectrum lands them AFTER the
    # negative frequencies and corrupts them -- shifting every tap by a distance-dependent
    # amount. The correct interpolation inserts the zeros between the positive and negative
    # halves so the negative frequencies stay at the high end of the padded array.
    n = cfg.n_subcarriers * zero_pad
    half = cfg.n_subcarriers // 2
    padded = np.concatenate([hw[:half], np.zeros(n - cfg.n_subcarriers), hw[half:]])
    return np.fft.ifft(padded) * zero_pad   # * zero_pad keeps the peak amplitude constant


def prepare_packet(h: np.ndarray, cfg: CSIConfig, zero_pad: int = 16) -> np.ndarray:
    """One packet -> a complex CIR referenced and aligned to its LOS peak.

    LOS = the strongest tap. Divide the CIR by its complex value (kills common phase +
    amplitude scaling; sets LOS phase to 0) and circularly shift it to bin 0 (kills STO).
    The returned CIRs from different packets are then mutually coherent and safe to average.
    """
    cir = raw_cir(h, cfg, zero_pad)
    p = int(np.argmax(np.abs(cir)))
    g = cir[p]
    if g == 0:
        return cir
    return np.roll(cir / g, -p)


def coherent_average(packets: np.ndarray, cfg: CSIConfig, zero_pad: int = 16) -> np.ndarray:
    """Prepare every packet, then average the aligned CIRs coherently.

    N packets buy 10 log10(N) dB of processing gain — the +30 dB (N=1000) that lifts the
    echo above the 8-bit quantisation floor (Part 4.5). Returns a complex CIR.
    """
    prepared = np.stack([prepare_packet(p, cfg, zero_pad) for p in packets])
    return prepared.mean(axis=0)


def delay_profile(packets_or_cir, cfg: CSIConfig, zero_pad: int = 16) -> np.ndarray:
    """|CIR| delay profile. Accepts either a packet stack (n_packets, n_sub) or a CIR.

    A packet stack is prepared + averaged; a 1-D CSI vector is treated as a single packet;
    a complex CIR (length n_sub*zero_pad) is used as-is.
    """
    arr = np.asarray(packets_or_cir)
    if arr.ndim == 2:
        return np.abs(coherent_average(arr, cfg, zero_pad))
    if arr.size == cfg.n_subcarriers:
        return np.abs(prepare_packet(arr, cfg, zero_pad))
    return np.abs(arr)


def differential(
    plate_in: np.ndarray,
    plate_out: np.ndarray,
    cfg: CSIConfig,
    zero_pad: int = 16,
) -> np.ndarray:
    """Coherent background subtraction: |averaged(plate_in) - averaged(plate_out)|.

    Both inputs are packet stacks (n_packets, n_subcarriers), aligned to LOS at bin 0. The
    subtraction cancels the static receiver response (the phantom generator) and the LOS,
    isolating the plate's echo at its excess-delay position. This is a bistatic RCS
    measurement with the empty room as background (IEEE Std 1502).
    """
    avg_in = coherent_average(plate_in, cfg, zero_pad)
    avg_out = coherent_average(plate_out, cfg, zero_pad)
    return np.abs(avg_in - avg_out)
