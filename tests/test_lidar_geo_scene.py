import numpy as np
import pytest
pytest.importorskip("sionna")
from wifi_radar_slam.config import load_config
from wifi_radar_slam.scene.builder import build_scene
from wifi_radar_slam.lidar.config import OUSTER_OS1
from wifi_radar_slam.lidar.sensor_geo import geo_sensor
from wifi_radar_slam.lidar.runner import run_lidar


def test_model_a_runs_on_controlled_scene():
    cfg = load_config("configs/controlled_oracle.yaml")
    built = build_scene(cfg)
    built.trajectory = built.trajectory[:3]            # 3 frames -> fast smoke
    m = run_lidar(built, OUSTER_OS1, geo_sensor, np.random.default_rng(0),
                  cfg.trajectory.timestep_s)
    assert set(m) == {"ate", "rpe", "chamfer", "map_accuracy", "map_completeness", "iou"}
    assert np.isfinite(m["ate"])
