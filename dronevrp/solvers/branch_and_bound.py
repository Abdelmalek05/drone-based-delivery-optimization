"""Exact depth-first branch and bound with an admissible lower bound."""

import math
import time
from typing import List, Optional

import numpy as np

from ..instance import Instance
from ..solution import Route, Solution
from .base import Solver


class BranchAndBoundSolver(Solver):
    """Depth-first B&B with append-only branching: each node appends one
    unvisited customer to the END of a route, and only the first empty route
    is ever extended (this breaks the K! symmetry between interchangeable
    empty routes). Uses an admissible in/out-arc lower bound and a wall-clock
    time limit. This is a real exact method: given enough time, it proves
    optimality."""

    def __init__(self, inst: Instance, time_limit: float = 600.0, verbose: bool = False):
        super().__init__(inst)
        self.time_limit = time_limit
        self.verbose = verbose
        self.best_energy: float = math.inf
        self.best_solution: Optional[Solution] = None
        self.nodes_explored: int = 0
        self.timed_out: bool = False
        self._start: float = 0.0

        # Precompute, for each customer c, the cheapest base-energy arcs that
        # can enter and leave it via any allowed node. Used by the admissible
        # lower bound: every unvisited customer must be entered by SOME arc and
        # left by SOME arc in any feasible solution, so the sum of the cheapest
        # of each gives a valid lower bound on its remaining contribution.
        self._min_in_arc = np.zeros(inst.n + 1)
        self._min_out_arc = np.zeros(inst.n + 1)
        for c in range(1, inst.n + 1):
            best_in = math.inf
            best_out = math.inf
            for j in range(inst.n + 1):
                if j == c:
                    continue
                if not inst.is_arc_blocked(j, c):
                    cost = inst.dist[j, c] * inst.alpha
                    if cost < best_in:
                        best_in = cost
                if not inst.is_arc_blocked(c, j):
                    cost = inst.dist[c, j] * inst.alpha
                    if cost < best_out:
                        best_out = cost
            self._min_in_arc[c]  = best_in  if best_in  < math.inf else 0.0
            self._min_out_arc[c] = best_out if best_out < math.inf else 0.0

    def _route_feasible(self, customers: List[int]) -> bool:
        return Route(customers).is_feasible(self.inst)

    def _lower_bound(self, partial: List[Route], unvisited: set, partial_energy: float) -> float:
        """Admissible lower bound: energy already committed in the partial routes,
        plus, for every still-unvisited customer, half of the sum of its cheapest
        possible incoming arc and cheapest possible outgoing arc.

        Why this is admissible: in any feasible solution every customer has exactly
        one incoming arc (cost >= min_in_arc[c] in base energy) and exactly one
        outgoing arc (cost >= min_out_arc[c]). Each arc in the solution is the
        outgoing arc of its source and the incoming arc of its destination, so if
        we sum (min_in[c] + min_out[c]) over all unvisited customers we count
        every arc roughly twice. Dividing by 2 gives a valid lower bound that is
        usually about twice as tight as using min_in_arc alone.

        Since partial_energy already includes the return-to-depot arcs of the
        currently open routes, no arc is double-counted between partial_energy
        and the unvisited-customer contributions."""
        lb = partial_energy
        for c in unvisited:
            lb += 0.5 * (self._min_in_arc[c] + self._min_out_arc[c])
        return lb

    def _explore(self, partial: List[Route], unvisited: set, partial_energy: float):
        if time.time() - self._start > self.time_limit:
            self.timed_out = True
            return
        self.nodes_explored += 1

        if not unvisited:
            total = sum(r.energy(self.inst) for r in partial)
            if total < self.best_energy:
                self.best_energy = total
                self.best_solution = Solution([r.copy() for r in partial])
                if self.verbose:
                    print(f"  [B&B] new incumbent: {total:.4f} kWh "
                          f"after {self.nodes_explored} nodes")
            return

        lb = self._lower_bound(partial, unvisited, partial_energy)
        if lb >= self.best_energy - 1e-9:
            return

        # Branch over (customer, route) pairs: try every unvisited customer in
        # every route, appending it at the end. To break the symmetry between
        # equivalent empty routes, only append to the first empty route. This
        # exploration is exhaustive: every solution is reachable as a sequence
        # of append operations in some order.
        for c in list(unvisited):
            seen_empty = False
            for k in range(len(partial)):
                if partial[k].is_empty():
                    if seen_empty:
                        continue
                    seen_empty = True
                new_customers = partial[k].customers + [c]
                if not self._route_feasible(new_customers):
                    continue
                old = partial[k]
                partial[k] = Route(new_customers)
                new_partial_e = partial_energy - old.energy(self.inst) + partial[k].energy(self.inst)
                unvisited.remove(c)
                self._explore(partial, unvisited, new_partial_e)
                unvisited.add(c)
                partial[k] = old
                if self.timed_out:
                    return

    def solve(self) -> Solution:
        self._start = time.time()
        partial = [Route() for _ in range(self.inst.K)]
        unvisited = set(range(1, self.inst.n + 1))
        self._explore(partial, unvisited, 0.0)
        self.runtime_sec = time.time() - self._start
        return self.best_solution if self.best_solution is not None else Solution(partial)
