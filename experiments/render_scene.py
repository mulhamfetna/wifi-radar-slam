"""Render the Sionna RT scene with the WiFi ray paths overlaid.

Post-hoc visualization (reads a config, computes paths for one vehicle position,
renders to an image). Requires `sionna-rt`. Example:

    python experiments/render_scene.py configs/nominal.yaml docs/assets/scene_paths.png
"""
import sys
import numpy as np
import sionna.rt as rt
import mitsuba as mi
from wifi_radar_slam.config import load_config
from wifi_radar_slam.scene.builder import build_scene


def main(config_path: str, out_path: str,
         vehicle_xy=(0.0, 0.0), camera=(45.0, 70.0, 55.0), samples=200000):
    cfg = load_config(config_path)
    built = build_scene(cfg)
    scene = built.scene
    scene.receivers["veh"].position = mi.Point3f(vehicle_xy[0], vehicle_xy[1], 1.5)
    paths = rt.PathSolver()(scene, max_depth=3, samples_per_src=samples)
    cam = rt.Camera(position=list(camera), look_at=[0.0, 0.0, 0.0])
    scene.render_to_file(camera=cam, filename=out_path, paths=paths,
                         resolution=(960, 700), num_samples=256)
    print(f"wrote {out_path}")


if __name__ == "__main__":
    cfg = sys.argv[1] if len(sys.argv) > 1 else "configs/nominal.yaml"
    out = sys.argv[2] if len(sys.argv) > 2 else "scene_paths.png"
    main(cfg, out)
