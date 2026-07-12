import numpy as np
from wifi_radar_slam.lidar.config import LidarConfig
from wifi_radar_slam.lidar.sensor_ref import reference_sensor
from wifi_radar_slam.lidar.runner import run_lidar


class _Built:
    def __init__(self):
        # box of wall points around a straight +x drive
        walls = []
        for y in (-4.0, 4.0):
            walls += [[x, y, 0.0] for x in np.linspace(0, 10, 40)]
        self.ground_truth_map = np.array(walls)
        n, dt, speed = 12, 0.1, 3.0
        self.trajectory = np.array([[speed * dt * f, 0.0, 0.0] for f in range(n)])


def test_run_lidar_emits_six_metrics_and_tracks():
    cfg = LidarConfig(angular_res_deg=2.0, fov_deg=360.0, max_range_m=100.0,
                      min_range_m=0.5, range_sigma_m=0.02)
    built = _Built()
    m = run_lidar(built, cfg, reference_sensor, np.random.default_rng(0), timestep_s=0.1)
    assert set(m) == {"ate", "rpe", "chamfer", "map_accuracy", "map_completeness", "iou"}
    assert np.isfinite(m["ate"]) and m["ate"] < 1.0
    assert np.isfinite(m["chamfer"])
