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
    "street_canyon_metal": "simple_street_canyon_with_cars",
    "floor_wall": "floor_wall",
}

# scenes whose building/car materials are overridden to a perfect reflector, so
# single-bounce specular returns exist (the default penetrable materials refract
# and yield no localizable single-bounce reflections). Used with in-street APs.
_METAL_SCENES = {"street_canyon_metal"}


def _footprint_ground_truth(scene):
    """Facade footprints (xy-perimeter) of every non-floor scatterer, as (M, 3)."""
    import numpy as np
    gt = []
    for name, obj in scene.objects.items():
        if "floor" in name.lower():
            continue
        bb = obj.mi_mesh.bbox()
        fp = footprint_points(np.array(bb.min).ravel(), np.array(bb.max).ravel(),
                              spacing=1.0)
        if fp.size:
            gt.append(np.column_stack([fp, np.zeros(len(fp))]))
    return np.vstack(gt) if gt else np.zeros((0, 3))


def _build_controlled_wall(cfg: RunConfig, rt, mi) -> BuiltScene:
    """Controlled reflective scene: a large metal wall + floor where single-bounce
    specular geometry is guaranteed clean (unlike the penetrable street canyon).

    The built-in `floor_wall` (a 4 m toy: wall in the x=0 plane) is scaled to
    vehicle scale and given a metal radio material (perfect reflector -> no
    transmission, clean specular returns). The vehicle drives parallel to the wall
    (+y at fixed x); every AP yields a single-bounce wall reflection each frame, so
    the oracle map should trace the wall footprint quantitatively.
    """
    import numpy as np
    scene = rt.load_scene(rt.scene.floor_wall)
    wall, floor = scene.objects["wall"], scene.objects["floor"]
    wall.scaling = mi.Point3f(1.0, 12.0, 4.0)      # -> y in [-21, 21], z in [-2.1, 5.9]
    floor.scaling = mi.Point3f(15.0, 12.0, 1.0)    # -> x in [-30, 30], y in [-24, 24]
    # assign metal BEFORE setting the frequency: the default brick/concrete ITU
    # materials are undefined at 60 GHz and the frequency setter updates all
    # assigned materials, so the override must precede it.
    metal = rt.ITURadioMaterial("wr_metal", "metal", thickness=0.02)
    wall.radio_material = metal
    floor.radio_material = metal
    scene.frequency = cfg.rf.carrier_hz

    scene.tx_array = rt.PlanarArray(num_rows=1, num_cols=1, vertical_spacing=0.5,
                                    horizontal_spacing=0.5, pattern="iso", polarization="V")
    scene.rx_array = rt.PlanarArray(num_rows=1, num_cols=cfg.rf.n_rx_antennas,
                                    vertical_spacing=0.5,
                                    horizontal_spacing=cfg.rf.antenna_spacing_frac,
                                    pattern="iso", polarization="V")

    ap_positions = [np.array(p, dtype=float) for p in cfg.scene.ap_positions]
    for i, ap in enumerate(ap_positions):
        scene.add(rt.Transmitter(name=f"ap_{i}",
                                 position=[float(ap[0]), float(ap[1]), float(ap[2])]))
    scene.add(rt.Receiver(name="veh", position=[-8.0, 0.0, RX_HEIGHT_M]))

    # trajectory parallel to the wall: fixed x = -8, sweeping y over length_m
    n = cfg.trajectory.n_frames
    ys = np.linspace(-cfg.trajectory.length_m / 2, cfg.trajectory.length_m / 2, n)
    traj = np.zeros((n, 3))
    traj[:, 0] = -8.0
    traj[:, 1] = ys

    return BuiltScene(scene=scene, trajectory=traj, ap_positions=ap_positions,
                      ground_truth_map=_footprint_ground_truth(scene))


def build_scene(cfg: RunConfig) -> BuiltScene:
    """Build a Sionna RT 2.0 scene from a built-in outdoor environment.

    Targets `sionna-rt` 2.0.x: load_scene / PlanarArray / Transmitter / Receiver /
    PathSolver. Ground truth is the facade footprints of the scene's static
    scatterers (excluding the floor). AP positions and RF/trajectory come from the
    config. `scene.name == "controlled_wall"` selects the controlled reflective
    scene used for the mapping demonstration.
    """
    import sionna.rt as rt   # lazy: heavy Mitsuba/Dr.Jit import, only when building
    import mitsuba as mi

    if cfg.scene.name == "controlled_wall":
        return _build_controlled_wall(cfg, rt, mi)

    key = _BUILTIN_SCENE.get(cfg.scene.name, "simple_street_canyon_with_cars")
    scene = rt.load_scene(getattr(rt.scene, key))

    if cfg.scene.name in _METAL_SCENES:   # perfect-reflector buildings -> clean specular
        # override BEFORE setting frequency (default ITU materials may be undefined
        # at 60 GHz, and the frequency setter updates all assigned materials)
        metal = rt.ITURadioMaterial("sc_metal", "metal", thickness=0.3)
        for name, obj in scene.objects.items():
            if "floor" not in name.lower():
                obj.radio_material = metal
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
    return BuiltScene(scene=scene, trajectory=traj,
                      ap_positions=ap_positions,
                      ground_truth_map=_footprint_ground_truth(scene))
