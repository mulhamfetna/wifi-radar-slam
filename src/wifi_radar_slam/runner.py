from __future__ import annotations
from .config import RunConfig
from .geometry import velocity_from_poses
from .sensing.frontend import extract_detections
from .slam.particle_filter import run_slam
from .eval.metrics import ate, rpe, chamfer, occupancy_iou
from .eval.figures import plot_map
from . import io_artifacts as io
from .scene.builder import build_scene          # patched in tests
from .channel.simulator import simulate_csi      # patched in tests


def run_phase_a(cfg: RunConfig, rng, force: bool = False) -> dict:
    run = cfg.run_name
    built = build_scene(cfg)

    if force or not io.exists(run, "channel", "csi"):
        csi = simulate_csi(built, cfg.rf, cfg.snr_db, rng)
        io.save_array(run, "channel", "csi", csi)
    else:
        csi = io.load_array(run, "channel", "csi")

    detections = extract_detections(csi, cfg.rf, n_paths=3)
    velocity = velocity_from_poses(built.trajectory, cfg.trajectory.timestep_s)
    est_traj, est_map = run_slam(detections, built.ap_positions, velocity,
                                 cfg.trajectory.timestep_s, rng)

    gt_traj = built.trajectory
    gt_xy = built.ground_truth_map[:, :2]
    metrics = {
        "ate": ate(est_traj, gt_traj),
        "rpe": rpe(est_traj, gt_traj),
        "chamfer": chamfer(est_map, gt_xy),
        "iou": occupancy_iou(est_map, gt_xy, cell=1.0),
    }
    io.save_json(run, "eval", "metrics", metrics)
    plot_map(est_map, gt_xy, est_traj, gt_traj,
             str(io.run_dir(run) / "eval" / "map.png"))
    return metrics
