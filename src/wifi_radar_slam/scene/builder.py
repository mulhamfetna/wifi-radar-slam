from __future__ import annotations
from dataclasses import dataclass
import numpy as np
from ..config import RunConfig
from ..geometry import straight_trajectory, footprint_points, RX_HEIGHT_M


@dataclass
class BuiltScene:
    scene: object          # sionna.rt.Scene (lazy import; kept untyped here)
    trajectory: np.ndarray
    ap_positions: list
    ground_truth_map: np.ndarray


# Map config scene names to built-in Sionna RT outdoor scenes. Sionna RT 2.0 has
# no programmatic box primitive, so we use a shipped street scene (more credible
# than hand-built boxes) and derive ground truth from its actual meshes.
_BUILTIN_SCENE = {
    "parking_lot": "simple_street_canyon_with_cars",
    "parking_lot_smoke": "simple_street_canyon_with_cars",
    "street_canyon": "simple_street_canyon_with_cars",
    "floor_wall": "floor_wall",
}


def build_scene(cfg: RunConfig) -> BuiltScene:
    """Build a Sionna RT 2.0 scene from a built-in outdoor environment.

    Targets `sionna-rt` 2.0.x: load_scene / PlanarArray / Transmitter / Receiver /
    PathSolver. Ground truth is the positions of the scene's static scatterers
    (cars + buildings, excluding the floor). AP positions and RF/trajectory come
    from the config.
    """
    import sionna.rt as rt   # lazy: heavy Mitsuba/Dr.Jit import, only when building

    key = _BUILTIN_SCENE.get(cfg.scene.name, "simple_street_canyon_with_cars")
    scene = rt.load_scene(getattr(rt.scene, key))
    scene.frequency = cfg.rf.carrier_hz

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
        scene.add(rt.Transmitter(name=f"ap_{i}",
                                 position=[float(ap[0]), float(ap[1]), float(ap[2])]))
    scene.add(rt.Receiver(name="veh", position=[0.0, 0.0, RX_HEIGHT_M]))

    # trajectory centered so the vehicle drives through the middle of the scene
    traj = straight_trajectory(cfg.trajectory.length_m, cfg.trajectory.speed_mps,
                               cfg.trajectory.timestep_s)
    traj[:, 0] -= cfg.trajectory.length_m / 2.0

    # ground-truth map: xy footprint (facade outline) of each static scatterer's
    # bounding box, excluding the floor. Reflections land on facades, not mesh
    # centroids, so the footprint is the correct reference for map Chamfer/IoU.
    gt = []
    for name, obj in scene.objects.items():
        if "floor" in name.lower():
            continue
        bb = obj.mi_mesh.bbox()
        fp = footprint_points(np.array(bb.min).ravel(), np.array(bb.max).ravel(),
                              spacing=1.0)
        if fp.size:
            gt.append(np.column_stack([fp, np.zeros(len(fp))]))   # z=0 -> keep (M,3)
    ground_truth_map = np.vstack(gt) if gt else np.zeros((0, 3))

    return BuiltScene(scene=scene, trajectory=traj,
                      ap_positions=ap_positions, ground_truth_map=ground_truth_map)
