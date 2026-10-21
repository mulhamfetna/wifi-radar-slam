import json
import numpy as np
from wifi_radar_slam.eval.scenes import (SceneCache, noisy_csi, oracle_detections,
                                         horizontal, fold_forward)


def _fake(tmp_path):
    paths = np.array([[0, 0, 1e-7, 0.3, 0, 1, 1, 5, 0],     # facade single bounce
                      [0, 0, 5e-8, 0.1, 0, 0, 0, -1, 0],    # LOS
                      [0, 1, 2e-7, 2.0, 0, 1, 1, 9, 1],     # floor bounce
                      [1, 1, 3e-7, -0.5, 0, 2, 1, 5, 0]])   # double bounce
    p = tmp_path / "c.npz"
    np.savez(p, poses=np.zeros((2, 3)), ap_positions=np.array([[0, 0, 5.5], [1, 1, 1.5]]),
             gt_map=np.zeros((1, 2)), paths=paths, cfr_40=np.ones((2, 2, 4, 8), np.complex64),
             meta=json.dumps({"timestep_s": 0.05, "carrier_hz": 5.2e9, "n_subcarriers": 8,
                              "n_rx_antennas": 4, "antenna_spacing_frac": 0.5}))
    return SceneCache.load(str(p))


def test_cache_roundtrip_and_oracle_filter(tmp_path):
    sc = _fake(tmp_path)
    assert sc.n_frames == 2 and 40 in sc.cfr and sc.rf(40).bandwidth_hz == 40e6
    d = oracle_detections(sc)
    assert d[0].shape == (1, 3) and np.isclose(d[0][0, 0], 1e-7 * 299792458.0)
    assert d[1].shape == (0, 3)


def test_noisy_csi_snr():
    cfr = np.ones((50, 3, 4, 64), complex)
    n = noisy_csi(cfr, 10.0, np.random.default_rng(0)) - cfr
    assert np.isclose(np.mean(np.abs(n) ** 2), 0.1, rtol=0.05)


def test_horizontal_projection_uses_height_difference():
    d = [np.array([[5.0, 0.0, 0.0], [3.0, 0.0, 1.0]])]
    h = horizontal(d, np.array([[0, 0, 5.5], [0, 0, 1.5]]), rx_height=1.5)[0]
    assert np.isclose(h[0, 0], 3.0) and np.isclose(h[1, 0], 3.0)


def test_fold_forward():
    assert np.isclose(fold_forward(np.pi - 0.3), 0.3)
    assert np.isclose(fold_forward(-np.pi + 0.2), -0.2)
    assert np.isclose(fold_forward(0.4), 0.4)


def test_excess_oracle_and_absolute_roundtrip(tmp_path):
    from wifi_radar_slam.eval.scenes import excess_oracle_detections, excess_to_absolute
    sc = _fake(tmp_path)
    ex = excess_oracle_detections(sc)
    assert np.isclose(ex[0][0, 0], (1e-7 - 5e-8) * 299792458.0)   # minus the LOS arrival
    ab = excess_to_absolute([np.array([[2.0, 0.0, 0.0]])], np.array([[3.0, 4.0, 0.0]]),
                            np.array([[0.0, 0.0, 1.5]]), rx_height=1.5)
    assert np.isclose(ab[0][0, 0], 7.0)
