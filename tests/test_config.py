from wifi_radar_slam.config import load_config, RunConfig


def test_load_nominal(tmp_path):
    cfg = load_config("configs/nominal.yaml")
    assert isinstance(cfg, RunConfig)
    assert cfg.rf.carrier_hz == 5.2e9
    assert cfg.rf.bandwidth_hz == 40e6
    assert cfg.rf.n_rx_antennas == 4
    assert cfg.trajectory.speed_mps == 5.0
    assert len(cfg.scene.ap_positions) == 3
    assert cfg.seed == 42


def test_derived_frame_count():
    cfg = load_config("configs/nominal.yaml")
    # 60 m at 5 m/s = 12 s; /0.05 s = 240 frames
    assert cfg.trajectory.n_frames == 240
