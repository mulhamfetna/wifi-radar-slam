"""KITTI-protocol odometry drift: translational % and rotational deg/100 m.

This is the metric the radar-odometry literature is actually scored on -- CFEAR reports
1.09 % on Oxford, DRO 0.26 % on Boreas -- and a radar paper reporting only ATE would be
marked down. It measures error over sub-sequences of fixed *length*, so it is invariant to
where the trajectory sits in the world and does not let one early blunder dominate the whole
run, which is exactly the failure mode ATE has on long drives.

SCOPE, AND A HARD EDGE. The standard sub-sequence lengths are 100-800 m. Our SIMULATED
trajectories are 30-60 m, so standard drift is mathematically undefined on them: there is
not one 100 m window to measure. `drift()` therefore reports NaN with n_segments=0 rather
than fabricating a value. The standard lengths are used on the real-radar credibility anchor
(Oxford / Boreas), where they are directly comparable to the published CFEAR and DRO numbers;
the simulated cells report ATE/RPE plus the four map metrics, as in paper 2. A reduced-length
drift figure may be computed by passing `lengths` explicitly, but it must be labelled as such
and never tabulated beside a published KITTI/Oxford number.
"""
from __future__ import annotations
import numpy as np


def path_lengths(traj: np.ndarray) -> np.ndarray:
    """Cumulative arc length (m) along a trajectory (n, >=2). Starts at 0."""
    xy = np.asarray(traj, dtype=float)[:, :2]
    seg = np.linalg.norm(np.diff(xy, axis=0), axis=1)
    return np.concatenate([[0.0], np.cumsum(seg)])


def _se2(pose) -> np.ndarray:
    """(x, y, yaw) -> 3x3 homogeneous SE(2) matrix."""
    x, y, th = float(pose[0]), float(pose[1]), float(pose[2])
    c, s = np.cos(th), np.sin(th)
    return np.array([[c, -s, x], [s, c, y], [0.0, 0.0, 1.0]])


def _se2_inv(T: np.ndarray) -> np.ndarray:
    R, t = T[:2, :2], T[:2, 2]
    out = np.eye(3)
    out[:2, :2] = R.T
    out[:2, 2] = -R.T @ t
    return out


def _last_frame_from(cum: np.ndarray, first: int, length: float) -> int:
    """First index j > first with cum[j] - cum[first] >= length, or -1 if none."""
    reach = np.nonzero(cum[first:] - cum[first] >= length)[0]
    return int(first + reach[0]) if reach.size else -1


def drift(est_traj: np.ndarray, gt: np.ndarray,
          lengths=(100, 200, 300, 400, 500, 600, 700, 800),
          step: int = 10) -> dict:
    """KITTI-protocol drift of `est_traj` against ground truth `gt`, both (n, 3) (x, y, yaw).

    Argument order is (est, gt), matching eval.metrics.

    For every start frame (each `step`-th) and every sub-sequence `length`, compare the
    relative motion the estimate claims against the relative motion the ground truth made:

        E = inv(gt_delta) @ est_delta
        translational error = ||E_t||  / length     -> reported as a percentage
        rotational error    = |E_yaw|  / length     -> reported as deg / 100 m

    and average over every segment. Returns
        {"trans_pct", "rot_deg_per_100m", "n_segments", "per_length": {L: (t%, r)}}
    with NaN for the two averages when no segment of any requested length fits inside the
    trajectory. That is a real case for us, not a corner one -- see the module docstring.
    """
    est_traj = np.asarray(est_traj, dtype=float)
    gt = np.asarray(gt, dtype=float)
    if est_traj.shape != gt.shape or gt.shape[1] < 3:
        raise ValueError(
            f"need matching (n,3) trajectories, got {est_traj.shape}, {gt.shape}")

    cum = path_lengths(gt)
    per_length: dict[int, tuple[float, float]] = {}
    all_t: list[float] = []
    all_r: list[float] = []

    for L in lengths:
        t_errs: list[float] = []
        r_errs: list[float] = []
        for first in range(0, len(gt), step):
            last = _last_frame_from(cum, first, float(L))
            if last < 0:
                continue
            gt_d = _se2_inv(_se2(gt[first])) @ _se2(gt[last])
            est_d = _se2_inv(_se2(est_traj[first])) @ _se2(est_traj[last])
            E = _se2_inv(gt_d) @ est_d
            t_errs.append(float(np.linalg.norm(E[:2, 2])) / L)
            r_errs.append(abs(float(np.arctan2(E[1, 0], E[0, 0]))) / L)
        if t_errs:
            per_length[int(L)] = (100.0 * float(np.mean(t_errs)),
                                  100.0 * float(np.rad2deg(np.mean(r_errs))))
            all_t.extend(t_errs)
            all_r.extend(r_errs)

    if not all_t:                       # trajectory too short for any requested length
        return {"trans_pct": float("nan"), "rot_deg_per_100m": float("nan"),
                "n_segments": 0, "per_length": {}}

    return {
        "trans_pct": 100.0 * float(np.mean(all_t)),                    # % of distance driven
        "rot_deg_per_100m": 100.0 * float(np.rad2deg(np.mean(all_r))),
        "n_segments": len(all_t),
        "per_length": per_length,
    }
