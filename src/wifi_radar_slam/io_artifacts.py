from __future__ import annotations
import json
import pathlib
import numpy as np

RESULTS_ROOT = pathlib.Path("results")


def run_dir(run_name: str) -> pathlib.Path:
    d = RESULTS_ROOT / run_name
    d.mkdir(parents=True, exist_ok=True)
    return d


def _path(run_name: str, stage: str, name: str, ext: str) -> pathlib.Path:
    d = run_dir(run_name) / stage
    d.mkdir(parents=True, exist_ok=True)
    return d / f"{name}.{ext}"


def save_array(run_name: str, stage: str, name: str, array: np.ndarray) -> None:
    np.savez_compressed(_path(run_name, stage, name, "npz"), data=array)


def load_array(run_name: str, stage: str, name: str) -> np.ndarray:
    with np.load(_path(run_name, stage, name, "npz")) as z:
        return z["data"]


def save_json(run_name: str, stage: str, name: str, obj: dict) -> None:
    _path(run_name, stage, name, "json").write_text(json.dumps(obj, indent=2))


def load_json(run_name: str, stage: str, name: str) -> dict:
    return json.loads(_path(run_name, stage, name, "json").read_text())


def exists(run_name: str, stage: str, name: str) -> bool:
    return (_path(run_name, stage, name, "npz").exists()
            or _path(run_name, stage, name, "json").exists())
