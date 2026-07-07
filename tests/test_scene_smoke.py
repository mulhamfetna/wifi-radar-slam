import pytest
pytest.importorskip("sionna")
from wifi_radar_slam.config import load_config
from wifi_radar_slam.scene.builder import build_scene


def test_build_scene_smoke():
    cfg = load_config("configs/nominal.yaml")
    built = build_scene(cfg)
    assert built.trajectory.shape == (cfg.trajectory.n_frames, 3)
    assert len(built.ap_positions) == 3
    assert built.ground_truth_map.shape[1] == 3
    assert built.ground_truth_map.shape[0] > 0
