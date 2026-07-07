from __future__ import annotations
import matplotlib
matplotlib.use("Agg")
import matplotlib.pyplot as plt


def plot_map(est_map, gt_map_xy, est_traj, gt_traj, path: str) -> None:
    fig, ax = plt.subplots(figsize=(8, 6))
    if gt_map_xy.size:
        ax.scatter(gt_map_xy[:, 0], gt_map_xy[:, 1], s=4, c="0.6", label="ground-truth map")
    if est_map.size:
        ax.scatter(est_map[:, 0], est_map[:, 1], s=12, c="C1", marker="x", label="estimated map")
    ax.plot(gt_traj[:, 0], gt_traj[:, 1], "k--", label="ground-truth path")
    ax.plot(est_traj[:, 0], est_traj[:, 1], "C0-", label="estimated path")
    ax.set_aspect("equal")
    ax.legend()
    ax.set_xlabel("x [m]")
    ax.set_ylabel("y [m]")
    fig.savefig(path, dpi=150, bbox_inches="tight")
    plt.close(fig)
