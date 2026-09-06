"""Loaders for the Boreas spinning-radar benchmark (Navtech, 4 Hz, 360 deg).

Pure NumPy + Pillow. NO NETWORK here -- fetching is experiments/fetch_boreas.py, mirroring the
lidar/kitti.py + experiments/fetch_kitti.py split.

Boreas, not Oxford Radar RobotCar, for one decisive reason: Boreas is served over ANONYMOUS
public HTTPS, while Oxford requires a registration that cannot be automated.

EVERY CONSTANT BELOW WAS VERIFIED BY DECODING A REAL FILE (2026-07-12), not read off a wiki:

    PIL.Image.open(scan) -> uint8 (400, 3371)
        cols 0..7   int64  timestamp of THAT azimuth, microseconds
        cols 8..9   uint16 rotation-encoder count
        col  10     uint8  valid flag (255 = valid)
        cols 11..   uint8  power, 3360 range bins

    azimuth_rad = encoder * pi / 2800     (encoder steps by 14 -> 0.9 deg; 400 x 0.9 = 360)
    range_m     = (bin + 0.5) * 0.0596    (3360 bins -> 200.3 m)

A full rotation spans ~249 ms and every azimuth carries its own timestamp -- which is what makes
motion compensation both possible and necessary (see `motion_compensate`).
"""
from __future__ import annotations
import numpy as np
from ..lidar.pointcloud import Scan
from .kstrongest import k_strongest

BOREAS_RANGE_RES_M = 0.0596        # metres per range bin
BOREAS_N_METADATA_COLS = 11        # 8 (timestamp) + 2 (encoder) + 1 (valid)
BOREAS_ENCODER_PER_REV = 5600      # encoder counts per revolution -> az = enc * pi / 2800


def range_bins(n_range: int) -> np.ndarray:
    """Range (m) at the centre of each bin."""
    return (np.arange(n_range) + 0.5) * BOREAS_RANGE_RES_M


def decode_polar(img: np.ndarray):
    """Split a raw Boreas polar image into (power, azimuths, timestamps_us, valid).

    `img` is the uint8 (n_azimuth, 11 + n_range) array straight out of the PNG.

    The 11 metadata columns are NOT power. Feeding them to the front-end would shift every range
    by 11 bins (0.66 m) and plant a bright fictional target at zero range in every azimuth.
    """
    img = np.asarray(img)
    if img.dtype != np.uint8 or img.ndim != 2:
        raise ValueError(f"expected a 2-D uint8 image, got {img.dtype} {img.shape}")
    if img.shape[1] <= BOREAS_N_METADATA_COLS:
        raise ValueError(f"image has no range bins: {img.shape}")

    ts = np.ascontiguousarray(img[:, 0:8]).view(np.int64).ravel()
    enc = np.ascontiguousarray(img[:, 8:10]).view(np.uint16).ravel()
    valid = img[:, 10] == 255
    power = img[:, BOREAS_N_METADATA_COLS:].astype(float)
    azimuths = enc.astype(float) * np.pi / (BOREAS_ENCODER_PER_REV / 2.0)
    return power, azimuths, ts, valid


def motion_compensate(points: np.ndarray, azimuth_times_us: np.ndarray,
                      scan_time_us: int, velocity_xy) -> np.ndarray:
    """Undistort a spinning-radar scan for the ego-motion that happened during the sweep.

    Args:
        points:           (n, 2) sensor-local points, one per return.
        azimuth_times_us: (n,) timestamp of the azimuth EACH point came from.
        scan_time_us:     the scan's reference timestamp (we use the sweep's last azimuth).
        velocity_xy:      (vx, vy) ego velocity in the SENSOR-LOCAL frame, m/s.

    Returns the (n, 2) points expressed as though every one had been measured at `scan_time_us`.

    THIS IS NOT OPTIONAL, AND HERE IS WHY. A Navtech scan sweeps 360 degrees in ~249 ms. At
    15 m/s the vehicle covers 3.7 m in that time -- sixty times the 0.06 m range resolution. A
    scan treated as instantaneous is therefore smeared by metres, and the smear ROTATES with the
    beam, so it does not cancel out. CFEAR compensates for exactly this.

    Skipping it would give us a badly-drifting back-end and a FAILED credibility gate for a
    reason that has nothing to do with our back-end -- a true-looking negative, which is the
    worst outcome an experiment can produce.

    Our SIMULATED radar has no such distortion (its scan is instantaneous), so this correction
    applies only to real spinning-radar data. That is a property of the DATA, not of the
    estimator -- the shared back-end stays untouched, as the whole argument requires.
    """
    points = np.asarray(points, dtype=float).reshape(-1, 2)
    t = np.asarray(azimuth_times_us, dtype=np.int64).ravel()
    if t.size != points.shape[0]:
        raise ValueError(f"{points.shape[0]} points but {t.size} azimuth timestamps")
    if points.size == 0:
        return points
    vx, vy = float(velocity_xy[0]), float(velocity_xy[1])
    dt = (t.astype(float) - float(scan_time_us)) * 1e-6        # seconds; negative before ref
    # A point measured dt seconds BEFORE the reference was taken from a position the ego has
    # since left, so in the reference frame it sits v*dt along the travel direction.
    return points + np.stack([vx * dt, vy * dt], axis=1)


def load_radar_scan(path: str, k: int = 12, min_range_m: float = 2.0,
                    max_range_m: float = 100.0, velocity_xy=None) -> Scan:
    """Decode one Boreas radar PNG and extract a Scan with the k-strongest front-end.

    If `velocity_xy` (m/s, sensor-local) is given, the scan is motion-compensated for the
    ~249 ms sweep -- see `motion_compensate`, which explains why that is mandatory on real
    spinning radar.

    max_range_m defaults to 100 m rather than the sensor's full 200 m: the far half of a Navtech
    scan is sparse and noisy, and CFEAR-class methods likewise work on a cropped range. Stated,
    not silent.
    """
    from PIL import Image                        # lazy: keeps import cost off the test path
    img = np.array(Image.open(path))
    power, azimuths, times, valid = decode_polar(img)
    if not valid.all():                          # drop azimuths the sensor flagged bad
        power, azimuths, times = power[valid], azimuths[valid], times[valid]
    if power.shape[0] == 0:
        return Scan.empty()

    scan = k_strongest(power, range_bins(power.shape[1]), azimuths, k=k,
                       min_range_m=min_range_m, max_range_m=max_range_m)
    if velocity_xy is None or len(scan) == 0:
        return scan

    # Recover each point's azimuth row by matching its bearing back onto the azimuth grid, so
    # every point is undistorted by the motion that occurred at ITS OWN measurement time.
    bearings = np.arctan2(scan.points[:, 1], scan.points[:, 0])
    row = np.abs(np.angle(np.exp(1j * (bearings[:, None] - azimuths[None, :])))).argmin(axis=1)
    return Scan(motion_compensate(scan.points, times[row], int(times[-1]), velocity_xy))


def load_gt_poses(csv_text: str):
    """Parse applanix/radar_poses.csv -> (timestamps_us (n,), poses (n,3) as x, y, yaw).

    Columns (the real header, verified):
        GPSTime,easting,northing,altitude,vel_east,vel_north,vel_up,
        roll,pitch,heading,angvel_z,angvel_y,angvel_x

    GPSTime is EXACTLY the radar PNG's filename, so scans join to poses by name -- no
    interpolation, no nearest-neighbour matching, and no chance of an off-by-one frame shift.

    We take (easting, northing) as (x, y) and `heading` as yaw. The yaw CONVENTION is not
    assumed here -- experiments/run_radar_anchor.py verifies it against the direction of travel
    before a single drift number is trusted.
    """
    lines = [ln for ln in csv_text.strip().splitlines() if ln.strip()]
    header = [c.strip() for c in lines[0].split(",")]
    for want in ("GPSTime", "easting", "northing", "heading"):
        if want not in header:
            raise ValueError(f"radar_poses.csv missing column {want!r}; got {header}")
    i_t, i_x = header.index("GPSTime"), header.index("easting")
    i_y, i_h = header.index("northing"), header.index("heading")

    ts, poses = [], []
    for ln in lines[1:]:
        f = ln.split(",")
        ts.append(int(f[i_t]))
        poses.append([float(f[i_x]), float(f[i_y]), float(f[i_h])])
    return np.array(ts, dtype=np.int64), np.array(poses, dtype=float)
