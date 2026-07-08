from __future__ import annotations
import dataclasses
from .config import RunConfig
from .geometry import velocity_from_poses
from .sensing.frontend import extract_detections
from .sensing.oracle import extract_oracle_detections
from .slam.particle_filter import run_slam
from .eval.metrics import ate, rpe, chamfer, occupancy_iou, map_accuracy, map_completeness
from .eval.figures import plot_map
from . import io_artifacts as io
from .scene.builder import build_scene          # patched in tests
from .channel.simulator import simulate_csi      # patched in tests


def run_phase_a(cfg: RunConfig, rng, force: bool = False) -> dict:
    run = cfg.run_name
    built = build_scene(cfg)

    if cfg.sensing_mode == "oracle":
        # oracle map: single-specular-bounce detections from Sionna's true paths
        detections = extract_oracle_detections(built, cfg.rf, rng)
    else:
        if force or not io.exists(run, "channel", "csi"):
            csi = simulate_csi(built, cfg.rf, cfg.snr_db, rng)
            io.save_array(run, "channel", "csi", csi)
        else:
            csi = io.load_array(run, "channel", "csi")
        detections = extract_detections(csi, cfg.rf, n_paths=3, world_aoa=cfg.world_aoa)
    velocity = velocity_from_poses(built.trajectory, cfg.trajectory.timestep_s)
    est_traj, est_map = run_slam(detections, built.ap_positions, velocity,
                                 cfg.trajectory.timestep_s, rng,
                                 init_pose=built.trajectory[0],
                                 map_min_support=cfg.map_min_support)

    gt_traj = built.trajectory
    gt_xy = built.ground_truth_map[:, :2]
    metrics = {
        "ate": ate(est_traj, gt_traj),
        "rpe": rpe(est_traj, gt_traj),
        "chamfer": chamfer(est_map, gt_xy),
        "map_accuracy": map_accuracy(est_map, gt_xy),
        "map_completeness": map_completeness(est_map, gt_xy),
        "iou": occupancy_iou(est_map, gt_xy, cell=1.0),
    }
    io.save_json(run, "eval", "metrics", metrics)
    plot_map(est_map, gt_xy, est_traj, gt_traj,
             str(io.run_dir(run) / "eval" / "map.png"))
    return metrics


def run_phase_b(base_cfg: RunConfig, sweep: dict, rng) -> list[dict]:
    """Run Phase A over a sweep grid; one swept parameter at a time."""
    results = []
    for param, values in sweep.items():
        for v in values:
            fv = float(v)          # YAML may parse "20.0e6" as a string; coerce
            if param == "bandwidth_hz":
                rf = dataclasses.replace(base_cfg.rf, bandwidth_hz=fv)
                cfg = dataclasses.replace(base_cfg, rf=rf,
                                          run_name=f"sweep_{param}_{fv:.0f}")
            elif param == "snr_db":
                cfg = dataclasses.replace(base_cfg, snr_db=fv,
                                          run_name=f"sweep_{param}_{fv:.0f}")
            elif param == "speed_mps":
                traj = dataclasses.replace(base_cfg.trajectory, speed_mps=fv)
                cfg = dataclasses.replace(base_cfg, trajectory=traj,
                                          run_name=f"sweep_{param}_{fv:.0f}")
            elif param == "n_aps":
                sc = dataclasses.replace(
                    base_cfg.scene,
                    ap_positions=base_cfg.scene.ap_positions[: int(fv)])
                cfg = dataclasses.replace(base_cfg, scene=sc,
                                          run_name=f"sweep_{param}_{int(fv)}")
            else:
                raise ValueError(f"unknown sweep parameter: {param}")
            m = run_phase_a(cfg, rng)
            results.append({"swept_param": param, "value": fv, **m})
    io.save_json("sweep", "eval", "summary", {"results": results})
    return results
