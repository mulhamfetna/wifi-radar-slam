"""Regenerate paper-2's OWN WiFi numbers, and audit paper-1's reported values.

Runs each WiFi config over several seeds and reports mean +/- std, so the paper
quotes a reproducible statistic rather than a single lucky run.
"""
import dataclasses, json, numpy as np
from wifi_radar_slam.config import load_config
from wifi_radar_slam.runner import run_phase_a

CONFIGS = {
    ("controlled_wall", "oracle"):    "configs/controlled_oracle.yaml",
    ("controlled_wall", "realistic"): "configs/controlled_music_joint.yaml",
    ("street_canyon",  "oracle"):     "configs/street_metal_oracle.yaml",
    ("street_canyon",  "realistic"):  "configs/street_metal_music.yaml",
}
# what paper 1 reported (docs/results-v1.md), for audit
PAPER1 = {
    ("controlled_wall", "oracle"):    {"ate": 0.045, "map_accuracy": 0.25, "iou": 0.79},
    ("controlled_wall", "realistic"): {"ate": 0.027},
    ("street_canyon",  "oracle"):     {"ate": 0.116, "map_accuracy": 0.30, "iou": 0.077},
}
SEEDS = (42, 1, 2, 3, 4)
KEYS = ["ate", "rpe", "chamfer", "map_accuracy", "map_completeness", "iou"]
out = {}
for (scene, mode), path in CONFIGS.items():
    cfg0 = load_config(path)
    runs = []
    for s in SEEDS:
        cfg = dataclasses.replace(cfg0, seed=s, run_name=f"regen_{scene}_{mode}_{s}")
        runs.append(run_phase_a(cfg, np.random.default_rng(s), force=True))
    stats = {}
    for k in KEYS:
        v = np.array([r[k] for r in runs], dtype=float)
        v = v[np.isfinite(v)]
        stats[k] = {"mean": float(v.mean()), "std": float(v.std())} if v.size else None
    out.setdefault(scene, {})[mode] = stats
    line = "  ".join(f"{k}={stats[k]['mean']:.3f}+-{stats[k]['std']:.3f}"
                     for k in ("ate", "chamfer", "map_accuracy", "iou") if stats[k])
    print(f"[{scene}/{mode}] {line}")
    p1 = PAPER1.get((scene, mode))
    if p1:
        for k, rep in p1.items():
            if stats.get(k):
                m, sd = stats[k]["mean"], stats[k]["std"]
                ok = abs(rep - m) <= max(2 * sd, 0.02 * max(abs(rep), 1e-9), 0.006)
                print(f"     audit {k}: paper-1 reports {rep} | ours {m:.3f}+-{sd:.3f} "
                      f"-> {'consistent' if ok else '*** DOES NOT REPRODUCE ***'}")
out["_note"] = (f"Paper-2's own WiFi results. mean+-std over seeds {SEEDS}. "
                "Regenerated so paper 2 does not depend on the unpublished paper 1.")
out["_seeds"] = list(SEEDS)
json.dump(out, open("data/wifi_results_paper2.json", "w"), indent=2)
print("\nsaved -> data/wifi_results_paper2.json")
