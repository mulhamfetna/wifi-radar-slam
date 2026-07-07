import numpy as np
import pytest
pytest.importorskip("sionna")
from wifi_radar_slam.config import load_config
from wifi_radar_slam.scene.builder import build_scene
from wifi_radar_slam.channel.simulator import simulate_csi


def test_simulate_csi_shape(monkeypatch):
    monkeypatch.setenv("WRS_NUM_SAMPLES", "10000")   # keep the CPU ray-trace test fast
    cfg = load_config("configs/nominal.yaml")
    built = build_scene(cfg)
    built.trajectory = built.trajectory[:2]          # 2 frames is enough for a shape check
    csi = simulate_csi(built, cfg.rf, cfg.snr_db, np.random.default_rng(0))
    assert csi.shape == (2, len(built.ap_positions), cfg.rf.n_rx_antennas, cfg.rf.n_subcarriers)
    assert np.iscomplexobj(csi)
    assert np.all(np.isfinite(csi))
