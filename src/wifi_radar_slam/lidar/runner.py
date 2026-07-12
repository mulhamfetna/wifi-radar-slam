from __future__ import annotations
from ..geometry import velocity_from_poses
from ..eval.metrics import (ate, rpe, chamfer, occupancy_iou,
                            map_accuracy, map_completeness)
from .slam_icp import run_lidar_slam


def run_lidar(built, cfg, make_sensor, rng, timestep_s: float) -> dict:
    """Run the LiDAR baseline on a BuiltScene and return the six comparison metrics.

    `make_sensor(built, cfg, rng) -> (pose -> Scan)` is the seam that selects the
    LiDAR model (reference / A / B). Metrics match `runner.run_phase_a` exactly so
    WiFi and LiDAR rows share one comparison table.
    """
    traj = built.trajectory
    sensor = make_sensor(built, cfg, rng)
    scans = [sensor(traj[f]) for f in range(len(traj))]
    velocity = velocity_from_poses(traj, timestep_s)
    est_traj, est_map = run_lidar_slam(scans, velocity, timestep_s, rng,
                                       init_pose=traj[0])
    gt_xy = built.ground_truth_map[:, :2]
    return {
        "ate": ate(est_traj, traj),
        "rpe": rpe(est_traj, traj),
        "chamfer": chamfer(est_map, gt_xy),
        "map_accuracy": map_accuracy(est_map, gt_xy),
        "map_completeness": map_completeness(est_map, gt_xy),
        "iou": occupancy_iou(est_map, gt_xy, cell=1.0),
    }
