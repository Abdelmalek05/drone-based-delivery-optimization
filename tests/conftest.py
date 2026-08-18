import json
from pathlib import Path

import pytest

from dronevrp import load_instances

ROOT = Path(__file__).resolve().parent.parent


@pytest.fixture(scope="session")
def instances():
    insts, _ = load_instances(ROOT / "data" / "data_spread.json")
    return insts


@pytest.fixture(scope="session")
def published():
    """The results shipped in results/cached-results.json — used as a golden file."""
    with open(ROOT / "results" / "cached-results.json", encoding="utf-8") as f:
        return json.load(f)["per_instance"]


@pytest.fixture(scope="session")
def small(instances):
    """INST_01: 8 customers, 3 drones — the instance B&B can solve to proven optimality."""
    return instances[0]
