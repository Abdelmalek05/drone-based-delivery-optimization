"""Nearest-neighbour construction heuristic."""

import math
import time
from typing import List

from ..solution import Route, Solution
from .base import Solver


class GreedySolver(Solver):
    """Nearest-neighbour: each drone picks the closest reachable customer until it cannot move."""

    def solve(self) -> Solution:
        t0 = time.time()
        inst = self.inst
        unvisited = set(range(1, inst.n + 1))
        routes: List[Route] = []

        for _ in range(inst.K):
            if not unvisited:
                break
            r = Route()
            current = 0
            while True:
                best_c = None
                best_d = math.inf
                for c in unvisited:
                    if inst.is_arc_blocked(current, c):
                        continue
                    trial = Route(r.customers + [c])
                    if not trial.is_feasible(inst):
                        continue
                    if inst.dist[current, c] < best_d:
                        best_d = inst.dist[current, c]
                        best_c = c
                if best_c is None:
                    break
                r.customers.append(best_c)
                unvisited.remove(best_c)
                current = best_c
            routes.append(r)

        while len(routes) < inst.K:
            routes.append(Route())

        self.runtime_sec = time.time() - t0
        return Solution(routes)
