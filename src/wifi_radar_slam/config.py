from __future__ import annotations
from dataclasses import dataclass
import yaml


@dataclass(frozen=True)
class RFConfig:
    carrier_hz: float
    bandwidth_hz: float
    n_subcarriers: int
    n_rx_antennas: int
    antenna_spacing_frac: float


@dataclass(frozen=True)
class TrajectoryConfig:
    length_m: float
    speed_mps: float
    timestep_s: float
    shape: str

    @property
    def duration_s(self) -> float:
        return self.length_m / self.speed_mps

    @property
    def n_frames(self) -> int:
        return int(round(self.duration_s / self.timestep_s))


@dataclass(frozen=True)
class SceneConfig:
    name: str
    ap_positions: list[tuple[float, float, float]]
    targets: list[dict]


@dataclass(frozen=True)
class RunConfig:
    run_name: str
    seed: int
    snr_db: float
    rf: RFConfig
    trajectory: TrajectoryConfig
    scene: SceneConfig
    sensing_mode: str = "music"   # "music" (CSI->MUSIC) or "oracle" (Sionna true paths)
    world_aoa: bool = False       # map MUSIC electrical angle -> world azimuth (single-ULA;
                                  # off by default: the transform's front/back ambiguity
                                  # regresses localization in multi-sided scenes)
    map_min_support: int = 1      # drop mapped clusters with fewer than this many detections
                                  # (consensus filter; rejects delay-AoA mis-pairing phantoms)
    joint_estimation: bool = False  # joint 2-D (delay-angle) MUSIC vs separate 1-D + sorted pairing


def load_config(path: str) -> RunConfig:
    with open(path, "r") as f:
        raw = yaml.safe_load(f)
    # PyYAML (YAML 1.1) parses unsigned-exponent scientific notation like
    # "5.2e9" as a string, so coerce numeric fields explicitly.
    r = raw["rf"]
    rf = RFConfig(
        carrier_hz=float(r["carrier_hz"]),
        bandwidth_hz=float(r["bandwidth_hz"]),
        n_subcarriers=int(r["n_subcarriers"]),
        n_rx_antennas=int(r["n_rx_antennas"]),
        antenna_spacing_frac=float(r["antenna_spacing_frac"]),
    )
    t = raw["trajectory"]
    traj = TrajectoryConfig(
        length_m=float(t["length_m"]),
        speed_mps=float(t["speed_mps"]),
        timestep_s=float(t["timestep_s"]),
        shape=str(t["shape"]),
    )
    scene = SceneConfig(
        name=raw["scene"]["name"],
        ap_positions=[tuple(float(c) for c in p) for p in raw["scene"]["ap_positions"]],
        targets=list(raw["scene"]["targets"]),
    )
    return RunConfig(
        run_name=raw["run_name"], seed=int(raw["seed"]),
        snr_db=float(raw["snr_db"]), rf=rf, trajectory=traj, scene=scene,
        sensing_mode=str(raw.get("sensing_mode", "music")),
        world_aoa=bool(raw.get("world_aoa", False)),
        map_min_support=int(raw.get("map_min_support", 1)),
        joint_estimation=bool(raw.get("joint_estimation", False)),
    )
