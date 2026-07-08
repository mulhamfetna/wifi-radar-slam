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
    assert set(metrics) == {"ate", "rpe", "chamfer",
                            "map_accuracy", "map_completeness", "iou"}
    assert np.isfinite(metrics["ate"])


def test_phase_b_grid(monkeypatch, tmp_path):
    from wifi_radar_slam import io_artifacts as io
    monkeypatch.setattr(io, "RESULTS_ROOT", tmp_path)
    # stub run_phase_a to avoid re-running the pipeline
    monkeypatch.setattr(runner, "run_phase_a",
                        lambda cfg, rng, force=False: {"ate": 1.0, "rpe": 0.1,
                                                       "chamfer": 2.0, "iou": 0.5})
    base = load_config("configs/nominal.yaml")
    # "20.0e6" as a string mirrors how PyYAML parses scientific notation
    results = runner.run_phase_b(base, {"bandwidth_hz": ["20.0e6", 160e6]},
                                 np.random.default_rng(0))
    assert len(results) == 2
    assert results[0]["swept_param"] == "bandwidth_hz"
    assert results[0]["value"] == 20e6
