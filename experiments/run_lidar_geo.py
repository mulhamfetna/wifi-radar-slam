"""Model A (geometric bbox-segment LiDAR) baseline on the simulated scenes.

Run on a host with sionna-rt installed (e.g. the amd server), throttled:
    nice -n 19 ionice -c3 python experiments/run_lidar_geo.py
Builds each scene, ray-casts the bbox-segment LiDAR, runs ICP SLAM, and reports
the six WiFi-comparable metrics (same metric shape as the WiFi runner).
"""
import json
import numpy as np
from wifi_radar_slam.config import load_config
from wifi_radar_slam.scene.builder import build_scene
from wifi_radar_slam.lidar.config import OUSTER_OS1
from wifi_radar_slam.lidar.sensor_geo import geo_sensor
from wifi_radar_slam.lidar.runner import run_lidar

SCENES = {
    "controlled_wall": "configs/controlled_oracle.yaml",
    "street_canyon": "configs/street_metal_oracle.yaml",
}


def main() -> None:
    rng = np.random.default_rng(0)
    results = {}
    for label, cfgpath in SCENES.items():
        cfg = load_config(cfgpath)
        built = build_scene(cfg)
        m = run_lidar(built, OUSTER_OS1, geo_sensor, rng, cfg.trajectory.timestep_s)
        results[label] = m
        print(f"[model A] {label}: " + json.dumps(m))
    with open("data/lidar_geo_results.json", "w") as f:
        json.dump(results, f, indent=2)
    print("saved -> data/lidar_geo_results.json")


if __name__ == "__main__":
    main()
