from __future__ import annotations
from dataclasses import dataclass
import numpy as np
from ..config import RunConfig
from ..geometry import straight_trajectory, targets_to_pointmap

import sionna.rt as rt   # heavy import isolated to this module (GPU stage)


@dataclass
class BuiltScene:
    scene: "rt.Scene"
    trajectory: np.ndarray
    ap_positions: list
    ground_truth_map: np.ndarray


def build_scene(cfg: RunConfig) -> BuiltScene:
    """Build a Sionna RT scene for the parking-lot config and its ground truth.

    NOTE (Sionna API): constructor/method names below target Sionna RT 0.19.x.
    If the pinned Sionna differs, adapt ONLY within this file and keep the
    BuiltScene fields identical so downstream stages are unaffected.
    Verify with: python -c "import sionna.rt as rt; help(rt)".
    """
    scene = rt.load_scene()                       # empty scene
    scene.frequency = cfg.rf.carrier_hz

    # ground plane
    scene.add(rt.Rectangle(
        name="ground", size=[200.0, 200.0], position=[30.0, 0.0, 0.0],
        material=rt.RadioMaterial("itu_concrete"),
    ))

    # box targets (cars / poles / walls) as cuboids
    for i, t in enumerate(cfg.scene.targets):
        material = "itu_metal" if t["kind"] == "car" else "itu_concrete"
        scene.add(rt.Box(
            name=f"target_{i}", size=list(t["size"]), position=list(t["center"]),
            material=rt.RadioMaterial(material),
        ))

    # transmit array = APs (single iso element); receive array = vehicle ULA
    scene.tx_array = rt.PlanarArray(
        num_rows=1, num_cols=1, vertical_spacing=0.5, horizontal_spacing=0.5,
        pattern="iso", polarization="V",
    )
    scene.rx_array = rt.PlanarArray(
        num_rows=1, num_cols=cfg.rf.n_rx_antennas, vertical_spacing=0.5,
        horizontal_spacing=cfg.rf.antenna_spacing_frac, pattern="iso", polarization="V",
    )

    ap_positions = [np.array(p, dtype=float) for p in cfg.scene.ap_positions]
    for i, ap in enumerate(ap_positions):
        scene.add(rt.Transmitter(name=f"ap_{i}", position=ap.tolist()))

    traj = straight_trajectory(
        cfg.trajectory.length_m, cfg.trajectory.speed_mps, cfg.trajectory.timestep_s)
    gt_map = targets_to_pointmap(cfg.scene.targets, spacing=0.5)
    return BuiltScene(scene=scene, trajectory=traj,
                      ap_positions=ap_positions, ground_truth_map=gt_map)
