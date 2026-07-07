import numpy as np
import yaml
from wifi_radar_slam.config import load_config
from wifi_radar_slam.runner import run_phase_b

if __name__ == "__main__":
    spec = yaml.safe_load(open("configs/sweep.yaml"))
    base = load_config(spec["base"])
    results = run_phase_b(base, spec["sweeps"], np.random.default_rng(base.seed))
    print(f"{len(results)} sweep points written to results/sweep/eval/summary.json")
