"""Loading benchmark instances from the dataset file."""

import json
from pathlib import Path
from typing import List, Tuple

from .instance import Instance

DEFAULT_DATA_PATH = Path("data/data_spread.json")


def load_dataset(path: Path = DEFAULT_DATA_PATH) -> dict:
    """Read the raw dataset JSON."""
    with open(path, encoding="utf-8") as f:
        return json.load(f)


def load_instances(path: Path = DEFAULT_DATA_PATH) -> Tuple[List[Instance], dict]:
    """Return (instances, raw_dataset) for the 10 benchmark instances."""
    data = load_dataset(path)
    instances = [Instance.from_dict(d, data["metadata"]) for d in data["instances"]]
    return instances, data
