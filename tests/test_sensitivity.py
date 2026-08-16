"""Unit tests for the reviewer-experiment harness (eval/sensitivity.py).

Deterministic; runs on the cached WiFiSLAM-Sim dataset, no Sionna.
"""
import numpy as np
import pytest

from wifi_radar_slam.eval import sensitivity as S


@pytest.fixture(scope="module")
def ds():
    return S.load_dataset()


@pytest.fixture(scope="module")
def dets(ds):
    return S.oracle_detections(ds)


def test_oracle_detections_shape_and_content(ds, dets):
    assert len(dets) == ds.poses.shape[0]
    allrows = np.vstack([d for d in dets if len(d)])
    assert allrows.shape[1] == 3
    assert (allrows[:, 0] > 0).all()                     # path lengths positive
    assert set(np.unique(allrows[:, 2]).astype(int)) <= {0, 1, 2}   # valid AP indices
    assert np.abs(allrows[:, 1]).max() <= np.pi + 1e-6   # AoA in radians


def test_perturb_ap_positions(ds):
    rng = np.random.default_rng(0)
    ap0 = ds.ap_positions
    np.testing.assert_array_equal(S.perturb_ap_positions(ap0, 0.0, rng), ap0)   # sigma 0 = identity
    big = S.perturb_ap_positions(ap0, 2.0, np.random.default_rng(1))
    shift = np.linalg.norm(big[:, :2] - ap0[:, :2], axis=1)
    assert (shift > 0).all() and shift.mean() < 8.0      # perturbed, but sane magnitude
    assert np.array_equal(big[:, 2], ap0[:, 2])          # z untouched


def test_blackout_empties_only_the_window(dets):
    b = S.blackout(dets, start=100, length=10)
    assert all(len(b[f]) == 0 for f in range(100, 110))
    assert len(b[50]) == len(dets[50]) and len(b[200]) == len(dets[200])


def test_baseline_localization_tracks(ds, dets):
    est, emap = S.localize(dets, ds.ap_positions, ds, np.random.default_rng(0))
    m = S.evaluate(est, emap, ds)
    assert m["ate_m"] < 0.5          # grounded: baseline ~0.15 m


def test_pf_dead_reckons_through_a_blackout(ds, dets):
    """With vehicle odometry, a 30-frame total measurement blackout does NOT blow up the
    trajectory — the PF dead-reckons through it and stays bounded. (The sensitivity the
    reviewer worried about shows up in the MAP, not the trajectory — see the experiment.)"""
    est_bo, _ = S.localize(S.blackout(dets, 100, 30), ds.ap_positions, ds,
                           np.random.default_rng(0))
    assert S.window_ate(est_bo, ds, 100, 30) < 0.5      # stays bounded through the blackout
