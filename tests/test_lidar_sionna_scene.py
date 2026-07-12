import numpy as np
import pytest
pytest.importorskip("sionna")
from wifi_radar_slam.config import load_config
from wifi_radar_slam.scene.builder import build_scene
from wifi_radar_slam.lidar.config import OUSTER_OS1
from wifi_radar_slam.lidar.sensor_sionna import sionna_lidar_sensor, SionnaLidarSensor
from wifi_radar_slam.lidar.runner import run_lidar


def test_model_b_returns_points_and_runs(monkeypatch):
    monkeypatch.setenv("WRS_NUM_SAMPLES", "100000")     # keep the ray-trace test fast
    cfg = load_config("configs/controlled_oracle.yaml")
    built = build_scene(cfg)
    built.trajectory = built.trajectory[:3]
    # sensor yields a non-empty scan (diffuse backscatter works)
    sensor = SionnaLidarSensor(built, OUSTER_OS1, np.random.default_rng(0))
    assert len(sensor(built.trajectory[0])) > 0
    # end-to-end metrics via the shared runner
    m = run_lidar(built, OUSTER_OS1, sionna_lidar_sensor, np.random.default_rng(0),
                  cfg.trajectory.timestep_s)
    assert set(m) == {"ate", "rpe", "chamfer", "map_accuracy", "map_completeness", "iou"}
    assert np.isfinite(m["ate"])
