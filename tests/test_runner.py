import numpy as np
from wifi_radar_slam.config import load_config
from wifi_radar_slam import runner


def test_phase_a_wiring(monkeypatch, tmp_path):
    from wifi_radar_slam import io_artifacts as io
    monkeypatch.setattr(io, "RESULTS_ROOT", tmp_path)
    cfg = load_config("configs/nominal.yaml")

    # fake scene + channel so no Sionna/GPU is needed
    class FakeBuilt:
        trajectory = np.column_stack([np.linspace(0, 10, 20), np.zeros(20), np.zeros(20)])
        ap_positions = [np.array([0.0, 20.0, 6.0])]
        ground_truth_map = np.array([[10.0, 3.0, 0.75]])

    def fake_build(_cfg):
        return FakeBuilt()

    def fake_csi(built, rf, snr, rng):
        n = built.trajectory.shape[0]
        return np.ones((n, 1, rf.n_rx_antennas, rf.n_subcarriers), dtype=complex)

    monkeypatch.setattr(runner, "build_scene", fake_build)
    monkeypatch.setattr(runner, "simulate_csi", fake_csi)

    metrics = runner.run_phase_a(cfg, np.random.default_rng(0))
    assert set(metrics) == {"ate", "rpe", "chamfer", "iou"}
    assert np.isfinite(metrics["ate"])
