"""Cost model (paper 2, RQ5): WiFi sensing package vs the automotive LiDAR envelope.

Pure functions over a sourced price file (`data/cost_data.yaml`) and the RQ3 metrics.
Every price is a (low, high) range with a text citation and date; outputs propagate
ranges rather than inventing point estimates.
"""
from __future__ import annotations
import yaml


def load_cost_data(path: str) -> dict:
    """Load the sourced price data (same yaml.safe_load idiom as config.load_config)."""
    with open(path, "r") as f:
        return yaml.safe_load(f)


def _range(item) -> tuple[float, float]:
    return float(item["low"]), float(item["high"])


def wifi_package_cost(data: dict, rx: str = "pi_nexmon", n_aps: int = 0) -> tuple[float, float]:
    """Vehicle-side WiFi sensing package: one CSI receiver + antennas.

    Headline (ambient-free): APs are pre-existing infrastructure -> not counted.
    `n_aps > 0` adds the deployed-AP sensitivity. Vehicle compute is assumed to be
    the existing SoC ($0 marginal).
    """
    receivers = data["wifi"]["receivers"]
    if rx not in receivers:
        raise KeyError(f"unknown receiver {rx!r}; have {sorted(receivers)}")
    lo, hi = _range(receivers[rx])
    a_lo, a_hi = _range(data["wifi"]["antennas"])
    lo, hi = lo + a_lo, hi + a_hi
    if n_aps:
        p_lo, p_hi = _range(data["wifi"]["ap_unit"])
        lo, hi = lo + n_aps * p_lo, hi + n_aps * p_hi
    return lo, hi


def lidar_envelope(data: dict) -> list[dict]:
    """The LiDAR price tiers, with numeric low/high coerced to float."""
    return [{**t, "low": float(t["low"]), "high": float(t["high"])}
            for t in data["lidar_tiers"]]


def cost_ratio(wifi_range, lidar_range) -> tuple[float, float]:
    """How many x cheaper the WiFi package is than a LiDAR, as a (low, high) range.

    Conservative low  = cheapest LiDAR / priciest  WiFi
    Optimistic   high = priciest  LiDAR / cheapest WiFi
    """
    w_lo, w_hi = float(wifi_range[0]), float(wifi_range[1])
    l_lo, l_hi = float(lidar_range[0]), float(lidar_range[1])
    return l_lo / w_hi, l_hi / w_lo
