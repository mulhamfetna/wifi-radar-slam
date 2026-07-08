import sys
import numpy as np
from wifi_radar_slam.config import load_config
from wifi_radar_slam.runner import run_phase_a

if __name__ == "__main__":
    config_path = sys.argv[1] if len(sys.argv) > 1 else "configs/nominal.yaml"
    cfg = load_config(config_path)
    metrics = run_phase_a(cfg, np.random.default_rng(cfg.seed))
    print(metrics)
