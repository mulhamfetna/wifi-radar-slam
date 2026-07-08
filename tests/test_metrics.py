import numpy as np
from wifi_radar_slam.eval.metrics import (
    ate, rpe, chamfer, occupancy_iou, map_accuracy, map_completeness,
)


def test_directional_map_metrics():
    # est points sit exactly on GT (perfect accuracy) but cover only 1 of 3 GT
    # surfaces (poor completeness) -> accuracy 0, completeness > 0, and the
    # symmetric chamfer is the average of the two directions.
    gt = np.array([[0.0, 0.0], [10.0, 0.0], [20.0, 0.0]])
    est = np.array([[0.0, 0.0]])
    assert np.isclose(map_accuracy(est, gt), 0.0)
    assert map_completeness(est, gt) > 0.0
    assert np.isclose(chamfer(est, gt),
                      0.5 * (map_accuracy(est, gt) + map_completeness(est, gt)))


def test_ate_zero_for_identical():
    t = np.random.default_rng(0).normal(size=(10, 3))
    assert ate(t, t) == 0.0


def test_ate_constant_offset():
    t = np.zeros((10, 3))
    s = t.copy()
    s[:, 0] += 2.0
    assert np.isclose(ate(s, t), 2.0)


def test_rpe_zero_for_constant_offset():
    t = np.cumsum(np.ones((10, 3)), axis=0)
    s = t.copy()
    s[:, 0] += 5.0                      # constant offset -> zero relative error
    assert np.isclose(rpe(s, t), 0.0)


def test_chamfer_zero_for_identical():
    m = np.array([[0.0, 0.0], [1.0, 1.0]])
    assert np.isclose(chamfer(m, m), 0.0)


def test_iou_identical_grid():
    m = np.array([[0.0, 0.0], [2.0, 2.0]])
    val = occupancy_iou(m, m, cell=1.0, bounds=(-1, 3, -1, 3))
    assert np.isclose(val, 1.0)
