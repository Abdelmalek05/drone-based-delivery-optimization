"""Route and Solution: the objects every solver manipulates."""

from typing import List, Optional, Tuple

from .instance import Instance


class Route:
    """One drone's path. Stored as a list of customer ids (without the depot)."""

    def __init__(self, customers: Optional[List[int]] = None):
        self.customers: List[int] = list(customers) if customers else []

    def __len__(self):
        return len(self.customers)

    def __iter__(self):
        return iter(self.customers)

    def copy(self) -> "Route":
        return Route(self.customers[:])

    def is_empty(self) -> bool:
        return len(self.customers) == 0

    def load(self, inst: Instance) -> float:
        if not self.customers:
            return 0.0
        return float(inst.demand[self.customers].sum())

    def energy(self, inst: Instance) -> float:
        """Total kWh of the route, including the return leg to the depot."""
        if not self.customers:
            return 0.0
        payload = self.load(inst)
        e = 0.0
        prev = 0  # depot
        for c in self.customers:
            d = inst.dist[prev, c]
            e += d * (inst.alpha + inst.beta * payload)
            payload -= inst.demand[c]
            prev = c
        # return leg, empty drone
        e += inst.dist[prev, 0] * inst.alpha
        return e

    def has_nfz(self, inst: Instance) -> bool:
        """True if any arc on the route is blocked."""
        if not self.customers:
            return False
        seq = [0] + self.customers + [0]
        return any(inst.is_arc_blocked(a, b) for a, b in zip(seq, seq[1:]))

    def is_feasible(self, inst: Instance) -> bool:
        if self.load(inst) > inst.Q + 1e-9:
            return False
        if self.has_nfz(inst):
            return False
        if self.energy(inst) > inst.B + 1e-9:
            return False
        return True


class Solution:
    """A list of routes, one per drone."""

    def __init__(self, routes: Optional[List[Route]] = None):
        self.routes: List[Route] = list(routes) if routes else []

    def copy(self) -> "Solution":
        return Solution([r.copy() for r in self.routes])

    def total_energy(self, inst: Instance) -> float:
        return sum(r.energy(inst) for r in self.routes)

    def active_drones(self) -> int:
        return sum(1 for r in self.routes if not r.is_empty())

    def visited_customers(self) -> List[int]:
        return [c for r in self.routes for c in r.customers]

    def check_feasibility(self, inst: Instance, verbose: bool = False) -> Tuple[bool, List[str]]:
        """Return (feasible, list_of_violation_strings)."""
        violations = []

        # every customer visited exactly once
        all_visits = self.visited_customers()
        expected = set(range(1, inst.n + 1))
        actual = set(all_visits)
        if len(all_visits) != len(actual):
            violations.append(f"duplicate visits: {len(all_visits) - len(actual)} duplicates")
        missing = expected - actual
        extra = actual - expected
        if missing:
            violations.append(f"missing customers: {sorted(missing)}")
        if extra:
            violations.append(f"extra/invalid customers: {sorted(extra)}")

        # drone count
        if self.active_drones() > inst.K:
            violations.append(f"too many active drones: {self.active_drones()} > {inst.K}")

        # per-route checks
        for k, r in enumerate(self.routes):
            if r.load(inst) > inst.Q + 1e-9:
                violations.append(f"route {k}: payload {r.load(inst):.3f} > Q={inst.Q}")
            if r.energy(inst) > inst.B + 1e-9:
                violations.append(f"route {k}: energy {r.energy(inst):.4f} > B={inst.B}")
            if r.has_nfz(inst):
                violations.append(f"route {k}: contains NFZ-blocked arc")

        feasible = (len(violations) == 0)
        if verbose and not feasible:
            for v in violations:
                print("  VIOLATION:", v)
        return feasible, violations

    def pretty(self) -> str:
        lines = []
        for k, r in enumerate(self.routes):
            if r.is_empty():
                continue
            path = " -> ".join(str(x) for x in r.customers)
            lines.append(f"  drone {k+1}: depot -> {path} -> depot")
        return "\n".join(lines) if lines else "  (empty solution)"
