from __future__ import annotations
import numpy as np


def _bbox_to_segments(bbmin, bbmax, z_height: float) -> np.ndarray:
    """The 4 xy rectangle edges of an axis-aligned bbox, if the horizontal scan
    plane z=z_height cuts it; else an empty (0,2,2) array. Pure geometry."""
    bbmin = np.asarray(bbmin, dtype=float).ravel()
    bbmax = np.asarray(bbmax, dtype=float).ravel()
    if z_height < bbmin[2] or z_height > bbmax[2]:
        return np.empty((0, 2, 2))
    x0, y0, x1, y1 = bbmin[0], bbmin[1], bbmax[0], bbmax[1]
    c = np.array([[x0, y0], [x1, y0], [x1, y1], [x0, y1]])
    return np.array([[c[0], c[1]], [c[1], c[2]], [c[2], c[3]], [c[3], c[0]]])


def scene_segments(built, z_height: float) -> np.ndarray:
    """2D wall segments (S,2,2) of every non-floor scene object at the scan plane.

    Reads object bounding boxes via `obj.mi_mesh.bbox()` (same API as the pipeline's
    footprint ground truth), so it requires a real Sionna-built scene at call time.
    """
    segs = []
    for name, obj in built.scene.objects.items():
        if "floor" in name.lower():
            continue
        bb = obj.mi_mesh.bbox()
        s = _bbox_to_segments(np.array(bb.min).ravel(), np.array(bb.max).ravel(), z_height)
        if s.shape[0]:
            segs.append(s)
    return np.vstack(segs) if segs else np.empty((0, 2, 2))
