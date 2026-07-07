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


def load_config(path: str) -> RunConfig:
    with open(path, "r") as f:
        raw = yaml.safe_load(f)
    rf = RFConfig(**raw["rf"])
    traj = TrajectoryConfig(**raw["trajectory"])
    scene = SceneConfig(
        name=raw["scene"]["name"],
        ap_positions=[tuple(p) for p in raw["scene"]["ap_positions"]],
        targets=list(raw["scene"]["targets"]),
    )
    return RunConfig(
        run_name=raw["run_name"], seed=int(raw["seed"]),
        snr_db=float(raw["snr_db"]), rf=rf, trajectory=traj, scene=scene,
    )
