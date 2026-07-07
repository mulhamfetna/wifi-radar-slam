import numpy as np
import pytest
pytest.importorskip("sionna")
from wifi_radar_slam.config import load_config
from wifi_radar_slam.scene.builder import build_scene
from wifi_radar_slam.channel.simulator import simulate_csi


def test_simulate_csi_shape():
    cfg = load_config("configs/nominal.yaml")
    built = build_scene(cfg)
    # shorten trajectory for the smoke test
    built.trajectory = built.trajectory[:4]
    csi = simulate_csi(built, cfg.rf, cfg.snr_db, np.random.default_rng(0))
    assert csi.shape == (4, 3, cfg.rf.n_rx_antennas, cfg.rf.n_subcarriers)
    assert np.iscomplexobj(csi)
    assert np.all(np.isfinite(csi))
