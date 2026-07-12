"""The k-strongest-per-azimuth front-end -- the CFEAR-style extractor.

PURE NumPy. Generic over the power map, so the SAME extractor serves the real Boreas radar and
our simulated radar. That is not a convenience, it is a requirement: a front-end that differed
between sensors would be confounded with the sensor difference we are trying to measure.

WHY THIS EXISTS ALONGSIDE CFAR. Sub-project 1 measured only ~1-5 CA-CFAR detections per frame on
a diffusely-scattering scene -- far too sparse for scan-to-map ICP. That is not a bug: CFAR is
built to find *point targets in noise*, but a real street (or a diffuse simulation of one)
returns a near-*continuum*, so the local background a wall cell is compared against is the wall
itself, and almost nothing clears the threshold.

Radar odometry has always known this. CFEAR -- the SOTA baseline we anchor against -- does not use
CFAR at all; per its own abstract it "keeps the strongest returns per azimuth", precisely because
radar targets are *extended*, not point-like. So the paper carries both front-ends: CFAR defines
the phantom rate (RQ1, where a calibrated detection threshold is what makes "this detection
matches no real path" a meaningful claim), and k-strongest drives SLAM (RQ3). Both are applied
identically to every ablation cell.
"""
from __future__ import annotations
import numpy as np
from ..lidar.pointcloud import Scan


def k_strongest(power: np.ndarray, ranges: np.ndarray, azimuths: np.ndarray,
                k: int = 12, min_range_m: float = 2.0,
                max_range_m: float | None = None, z_min: float = 0.0) -> Scan:
    """Keep the k strongest range bins in each azimuth -> a sensor-local Scan.

    Args:
        power:       (n_azimuth, n_range) real power / intensity.
        ranges:      (n_range,) range of each bin, metres.
        azimuths:    (n_azimuth,) bearing of each row, radians (+x forward, +y at +90 deg).
        k:           returns kept per azimuth.
        min_range_m: blind zone -- a monostatic radar hears itself at short range.
        max_range_m: gate beyond this (None -> no upper gate).
        z_min:       absolute power floor; bins at or below it are never returned.

    Returns a Scan in the sensor-local frame. Points are an ordinary polar -> Cartesian
    projection: the geometry is monostatic, so the measured range is an honest round trip.
    """
    power = np.asarray(power, dtype=float)
    ranges = np.asarray(ranges, dtype=float).ravel()
    azimuths = np.asarray(azimuths, dtype=float).ravel()
    if power.shape != (azimuths.size, ranges.size):
        raise ValueError(
            f"power {power.shape} does not match grids "
            f"(n_azimuth={azimuths.size}, n_range={ranges.size})")

    gate = ranges >= min_range_m
    if max_range_m is not None:
        gate &= ranges <= max_range_m
    if not gate.any():
        return Scan.empty()

    p = power[:, gate]
    r_gated = ranges[gate]

    kk = int(min(k, p.shape[1]))
    # argpartition puts the kk largest of each row in the last kk slots -- O(n) per row, and
    # we do not care about their order among themselves.
    idx = np.argpartition(p, -kk, axis=1)[:, -kk:]           # (n_azimuth, kk)
    rows = np.repeat(np.arange(p.shape[0]), kk)
    cols = idx.ravel()
    vals = p[rows, cols]

    keep = vals > z_min
    if not keep.any():
        return Scan.empty()
    rows, cols = rows[keep], cols[keep]

    r = r_gated[cols]
    a = azimuths[rows]
    return Scan(np.stack([r * np.cos(a), r * np.sin(a)], axis=1))


def k_strongest_from_cfg(ra_map: np.ndarray, cfg, k: int = 12) -> Scan:
    """k_strongest on OUR simulated radar, using the RadarConfig's own grids.

    The simulated cells go through this identical extractor so the front-end is held fixed
    across every ablation cell (A-D) and across real-vs-simulated data.
    """
    return k_strongest(ra_map, cfg.range_bins(), cfg.azimuth_grid(), k=k,
                       min_range_m=cfg.min_range_m, max_range_m=cfg.max_range_m)
