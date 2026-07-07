import numpy as np
from wifi_radar_slam.config import load_config
from wifi_radar_slam.runner import run_phase_a

if __name__ == "__main__":
    cfg = load_config("configs/nominal.yaml")
    metrics = run_phase_a(cfg, np.random.default_rng(cfg.seed))
    print(metrics)
