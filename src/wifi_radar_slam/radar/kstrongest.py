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
from scipy.ndimage import maximum_filter1d

from ..lidar.pointcloud import Scan

# Minimum range separation between two returns in the same azimuth, in metres. Below this
# they are two samples of ONE extended target, not two targets. See k_strongest.
DEFAULT_MIN_SEPARATION_M = 1.0


def k_strongest(power: np.ndarray, ranges: np.ndarray, azimuths: np.ndarray,
                k: int = 12, min_range_m: float = 2.0,
                max_range_m: float | None = None, z_min: float = 0.0,
                min_separation_m: float = DEFAULT_MIN_SEPARATION_M) -> Scan:
    """Keep the k strongest DISTINCT returns in each azimuth -> a sensor-local Scan.

    Args:
        power:            (n_azimuth, n_range) real power / intensity.
        ranges:           (n_range,) range of each bin, metres.
        azimuths:         (n_azimuth,) bearing of each row, radians (+x forward, +y at +90).
        k:                returns kept per azimuth.
        min_range_m:      blind zone -- a monostatic radar hears itself at short range.
        max_range_m:      gate beyond this (None -> no upper gate).
        z_min:            absolute power floor; bins at or below it are never returned.
        min_separation_m: two returns closer than this in range are the SAME target; only the
                          stronger survives. 0 disables the suppression.

    Returns a Scan in the sensor-local frame. Points are an ordinary polar -> Cartesian
    projection: the geometry is monostatic, so the measured range is an honest round trip.

    WHY THE SEPARATION IS LOAD-BEARING -- this cost us the credibility gate once.

    A radar target is EXTENDED: a wall lights up a run of adjacent range bins. So the k
    strongest *bins* are, overwhelmingly, k samples of ONE target rather than k targets.
    Measured on real Boreas data: the 12 picks in an azimuth spanned a median of just 0.7 m,
    and 96 % of consecutive picks sat under 0.15 m apart. A nominal 4,800-point cloud
    therefore carried only ~400 independent measurements, each smeared into a short RADIAL
    STREAK pointing away from the sensor.

    Point-to-point ICP handles that badly: it can slide along a radial streak almost for
    free, and its correspondences get dominated by matching duplicates to duplicates. Handed
    the exact frame-to-frame motion as its starting guess, it still converged 0.62 m away
    from it -- on a 2 m step. Enforcing 1 m of separation cut that to 0.13 m, and it is the
    difference between a radar baseline that tracks and one that does not.

    This is also what the name has always promised. CFEAR keeps the strongest RETURNS per
    azimuth; returns are targets, not ADC bins.
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

    if min_separation_m > 0 and r_gated.size > 1:
        # Non-maximum suppression along range: a bin survives only if it is the strongest
        # within +/- min_separation_m of itself, so what reaches the top-k below is a set of
        # distinct target PEAKS, not a run of samples off one target's flank.
        bin_m = float(np.median(np.diff(r_gated)))
        w = max(int(round(min_separation_m / max(bin_m, 1e-9))), 1)
        peak = p >= maximum_filter1d(p, size=2 * w + 1, axis=1, mode="nearest")
        p = np.where(peak, p, -np.inf)

    kk = int(min(k, p.shape[1]))
    # argpartition puts the kk largest of each row in the last kk slots -- O(n) per row, and
    # we do not care about their order among themselves.
    idx = np.argpartition(p, -kk, axis=1)[:, -kk:]           # (n_azimuth, kk)
    rows = np.repeat(np.arange(p.shape[0]), kk)
    cols = idx.ravel()
    vals = p[rows, cols]

    keep = np.isfinite(vals) & (vals > z_min)                # -inf = suppressed by NMS
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
