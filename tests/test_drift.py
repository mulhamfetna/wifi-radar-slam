import numpy as np
import pytest
from wifi_radar_slam.eval.drift import path_lengths, drift


def straight(n=200, dx=5.0):
    """A ~1 km straight-line trajectory: n frames, dx metres apart, heading +x."""
    t = np.zeros((n, 3))
    t[:, 0] = np.arange(n) * dx
    return t


def test_path_lengths_starts_at_zero_and_accumulates():
    cum = path_lengths(straight(n=5, dx=5.0))
    assert np.allclose(cum, [0.0, 5.0, 10.0, 15.0, 20.0])


def test_perfect_estimate_has_zero_drift():
    gt = straight()
    d = drift(gt, gt)
    assert d["n_segments"] > 0
    assert d["trans_pct"] == pytest.approx(0.0, abs=1e-9)
    assert d["rot_deg_per_100m"] == pytest.approx(0.0, abs=1e-9)


def test_a_one_percent_scale_error_gives_one_percent_drift():
    # An estimate that travels 1 % too far accrues exactly 1 % translational drift, at
    # EVERY sub-sequence length. This is the calibration test for the whole metric.
    gt = straight()
    est = gt.copy()
    est[:, 0] *= 1.01
    d = drift(est, gt)
    assert d["trans_pct"] == pytest.approx(1.0, abs=1e-6)


def test_drift_is_invariant_to_a_global_rigid_transform():
    # Drift is a metric on RELATIVE motion, which is exactly why radar odometry uses it:
    # unlike ATE, it does not care where the trajectory sits in the world.
    gt = straight()
    th = np.deg2rad(37.0)
    R = np.array([[np.cos(th), -np.sin(th)], [np.sin(th), np.cos(th)]])
    moved = gt.copy()
    moved[:, :2] = gt[:, :2] @ R.T + np.array([100.0, -50.0])
    moved[:, 2] = gt[:, 2] + th
    assert drift(moved, moved)["trans_pct"] == pytest.approx(
        drift(gt, gt)["trans_pct"], abs=1e-9)


def test_a_yaw_bias_shows_up_as_rotational_drift():
    gt = straight()
    est = gt.copy()
    est[:, 2] += np.deg2rad(1.0)              # constant 1 deg heading error
    assert drift(est, gt)["rot_deg_per_100m"] > 0.0


def test_short_trajectory_returns_nan_not_a_fabricated_number():
    # OUR SIMULATED TRAJECTORIES ARE 30-60 m. KITTI's shortest sub-sequence is 100 m, so
    # standard drift is UNDEFINED on them. It must say so, loudly, rather than quietly
    # averaging over zero segments or silently shrinking the window.
    gt = straight(n=10, dx=5.0)               # 45 m total
    d = drift(gt, gt)
    assert d["n_segments"] == 0
    assert np.isnan(d["trans_pct"])
    assert np.isnan(d["rot_deg_per_100m"])
    assert d["per_length"] == {}


def test_reduced_lengths_work_on_short_trajectories_when_asked_for_explicitly():
    # Sub-project 3 may report drift at reduced lengths on the simulated cells -- but only
    # because it passed `lengths` explicitly and labelled the result as such. The DEFAULT
    # never silently degrades to whatever happens to fit.
    gt = straight(n=10, dx=5.0)               # 45 m total
    d = drift(gt, gt, lengths=(10, 20, 30))
    assert d["n_segments"] > 0
    assert d["trans_pct"] == pytest.approx(0.0, abs=1e-9)


def test_per_length_breakdown_is_reported():
    gt = straight()
    d = drift(gt, gt)
    assert set(d["per_length"]) <= {100, 200, 300, 400, 500, 600, 700, 800}
    assert all(len(v) == 2 for v in d["per_length"].values())


def test_mismatched_trajectories_are_rejected():
    with pytest.raises(ValueError):
        drift(straight(n=10), straight(n=11))


def test_argument_order_matches_eval_metrics():
    # eval.metrics uses (est, gt). drift must too -- a silently swapped pair produces a
    # plausible-looking wrong number, which is the worst kind.
    gt = straight()
    est = gt.copy()
    est[:, 0] *= 1.10                          # estimate runs 10 % long
    assert drift(est, gt)["trans_pct"] == pytest.approx(10.0, abs=1e-6)
