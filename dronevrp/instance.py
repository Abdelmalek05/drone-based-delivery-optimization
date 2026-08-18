"""Problem instance: distances, demands, drone parameters, blocked arcs."""

from dataclasses import dataclass

import numpy as np


@dataclass
class Instance:
    """All the data of one problem instance."""
    instance_id: str
    category: str
    n: int                              # number of customers
    K: int                              # number of drones
    Q: float                            # payload capacity (kg)
    B: float                            # battery capacity (kWh)
    alpha: float                        # base kWh/km
    beta: float                         # load factor kWh/km/kg
    dist: np.ndarray                    # (n+1, n+1) distance matrix in km
    demand: np.ndarray                  # (n+1,) demand in kg (demand[0] = 0)
    blocked: set                        # frozenset({i,j}) NFZ-blocked undirected edges
    customers: list                     # raw customer dicts (for plotting)
    depot: dict                         # raw depot dict (for plotting)

    @classmethod
    def from_dict(cls, inst_dict: dict, meta: dict) -> "Instance":
        n = inst_dict["num_customers"]
        K = inst_dict["num_drones"]
        spec = inst_dict["fleet"][0]
        # wh -> kWh
        alpha = spec["base_consumption_wh_per_km"] / 1000.0
        beta  = spec["load_factor_wh_per_km_per_kg"] / 1000.0
        Q = spec["max_payload_kg"]
        B = spec["battery_capacity_kwh"]

        dist = np.array(inst_dict["distance_matrix_km"], dtype=float)
        demand = np.zeros(n + 1)
        for c in inst_dict["customers"]:
            demand[c["customer_id"]] = c["demand_kg"]

        blocked = set()
        for key in inst_dict["nfz_blocked_edges"].get("edges", {}).keys():
            i, j = map(int, key.split("-"))
            blocked.add(frozenset({i, j}))

        return cls(
            instance_id=inst_dict["instance_id"],
            category=inst_dict["category"],
            n=n, K=K, Q=Q, B=B, alpha=alpha, beta=beta,
            dist=dist, demand=demand, blocked=blocked,
            customers=inst_dict["customers"],
            depot={"latitude": 36.7833, "longitude": 3.0583, "name": "Algiers_Port"},
        )

    def is_arc_blocked(self, i: int, j: int) -> bool:
        return frozenset({i, j}) in self.blocked
