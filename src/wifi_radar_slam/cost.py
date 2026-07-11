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
