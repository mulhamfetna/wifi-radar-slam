"""The phantom rate -- paper 3's headline measurement (RQ1).

A PHANTOM is a detection that corresponds to NO real propagation path. This is paper 2's
definition, kept identical so paper 3's numbers sit beside its ~89 % without a footnote: match
each detection to the nearest TRUE ray-traced path of the same transmitter in normalised
(range, azimuth) space, and call it a phantom if nothing is close enough.

    cost = hypot( dRange / range_scale , dAzimuth / azimuth_scale )
    cost > max_cost  ->  PHANTOM

THE ONE METHODOLOGICAL CHOICE THAT MUST NOT BE FUDGED. A *fixed* absolute tolerance (3 m) would
let a high-resolution sensor win for a trivial reason: cell D resolves to 0.0375 m, so its
detections naturally land closer to true paths than cell A's, which resolves to 0.94 m.
Reporting only the fixed tolerance would hand radar a lower phantom rate BY CONSTRUCTION.

So every result reports BOTH:
  * fixed (3 m / 10 deg) -- directly comparable to paper 2's ~89 %
  * resolution-scaled (3 x the cell's own range resolution) -- "is this detection explainable
    by a real path, GIVEN WHAT THIS SENSOR CAN RESOLVE?", which separates *phantom* from
    merely *coarse*
Neither number may be reported without the other.

UNITS: det_range_m and true_range_m must be the SAME quantity -- round-trip range for a
monostatic cell, bistatic PATH LENGTH for cell A. This module compares like with like and does
not convert; the caller does.
"""
from __future__ import annotations
import numpy as np


def match_detections(det_range_m, det_azimuth_rad, true_range_m, true_azimuth_rad,
                     range_scale_m: float = 3.0, azimuth_scale_deg: float = 10.0,
                     max_cost: float = 3.0) -> np.ndarray:
    """Index of the nearest true path per detection, or -1 where none is plausible.

    Returns an int array of length len(det_range_m). **-1 means PHANTOM.**
    """
    d_r = np.asarray(det_range_m, dtype=float).ravel()
    d_a = np.asarray(det_azimuth_rad, dtype=float).ravel()
    t_r = np.asarray(true_range_m, dtype=float).ravel()
    t_a = np.asarray(true_azimuth_rad, dtype=float).ravel()
    if d_r.size != d_a.size:
        raise ValueError(f"{d_r.size} detection ranges but {d_a.size} azimuths")
    if t_r.size != t_a.size:
        raise ValueError(f"{t_r.size} true ranges but {t_a.size} azimuths")
    if d_r.size == 0:
        return np.empty(0, dtype=int)
    if t_r.size == 0:
        return np.full(d_r.size, -1, dtype=int)          # nothing real to match: all phantom

    az_scale = np.deg2rad(azimuth_scale_deg)
    dr = (t_r[None, :] - d_r[:, None]) / range_scale_m
    # wrapped angular difference -- 179 deg and -179 deg are 2 deg apart, not 358
    da = np.angle(np.exp(1j * (t_a[None, :] - d_a[:, None]))) / az_scale
    cost = np.hypot(dr, da)                              # (n_det, n_true)

    j = np.argmin(cost, axis=1)
    best = cost[np.arange(d_r.size), j]
    return np.where(best <= max_cost, j, -1).astype(int)


def phantom_stats(det_range_m, det_azimuth_rad, true_range_m, true_azimuth_rad,
                  range_scale_m: float = 3.0, azimuth_scale_deg: float = 10.0,
                  max_cost: float = 3.0) -> dict:
    """Phantom rate + range bias, for one cell.

    Returns {n_detections, n_phantoms, phantom_rate, range_bias_m, abs_range_err_m,
             azimuth_err_deg}.

    `range_bias_m` is the median **signed** range error over MATCHED detections. The sign
    matters: paper 2's second headline number was a 6.45 m median range BIAS -- a systematic
    offset far beyond the 0.94 m resolution limit -- not mere scatter. Phantoms are excluded
    from it, because a detection with no true path has nothing to be biased against.

    With no detections at all the rate is **NaN, not 0** -- a sensor that detects nothing is
    not phantom-free, it is blind, and 0 % would read as perfection.
    """
    d_r = np.asarray(det_range_m, dtype=float).ravel()
    d_a = np.asarray(det_azimuth_rad, dtype=float).ravel()
    t_r = np.asarray(true_range_m, dtype=float).ravel()
    t_a = np.asarray(true_azimuth_rad, dtype=float).ravel()

    if d_r.size == 0:
        return {"n_detections": 0, "n_phantoms": 0, "phantom_rate": float("nan"),
                "range_bias_m": float("nan"), "abs_range_err_m": float("nan"),
                "azimuth_err_deg": float("nan")}

    m = match_detections(d_r, d_a, t_r, t_a, range_scale_m, azimuth_scale_deg, max_cost)
    hit = m >= 0
    n_ph = int((~hit).sum())

    if not hit.any():
        return {"n_detections": int(d_r.size), "n_phantoms": n_ph, "phantom_rate": 1.0,
                "range_bias_m": float("nan"), "abs_range_err_m": float("nan"),
                "azimuth_err_deg": float("nan")}

    j = m[hit]
    d_rng = d_r[hit] - t_r[j]                            # SIGNED: the bias must keep its sign
    d_az = np.angle(np.exp(1j * (d_a[hit] - t_a[j])))
    return {
        "n_detections": int(d_r.size),
        "n_phantoms": n_ph,
        "phantom_rate": float(n_ph / d_r.size),
        "range_bias_m": float(np.median(d_rng)),
        "abs_range_err_m": float(np.median(np.abs(d_rng))),
        "azimuth_err_deg": float(np.median(np.abs(np.rad2deg(d_az)))),
    }
