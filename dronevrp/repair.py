"""Repairer: turn any candidate solution back into a feasible one."""

import math
import random
from typing import List, Optional, Tuple

from .instance import Instance
from .solution import Route, Solution


class Repairer:
    """Fix a (possibly broken) solution: dedup, trim infeasible routes, reinsert missing customers."""

    def __init__(self, inst: Instance):
        self.inst = inst

    def _try_insert(self, route: Route, cust: int) -> Tuple[Optional[Route], float]:
        """Try inserting cust at every position of route. Return (best_route, delta_energy)."""
        base_e = route.energy(self.inst)
        if route.load(self.inst) + self.inst.demand[cust] > self.inst.Q + 1e-9:
            return None, math.inf

        best_route = None
        best_delta = math.inf
        for pos in range(len(route) + 1):
            new_customers = route.customers[:pos] + [cust] + route.customers[pos:]
            cand = Route(new_customers)
            if not cand.is_feasible(self.inst):
                continue
            delta = cand.energy(self.inst) - base_e
            if delta < best_delta:
                best_delta = delta
                best_route = cand
        return best_route, best_delta

    def repair(self, sol: Solution) -> Solution:
        inst = self.inst
        sol = sol.copy()
        pool: List[int] = []

        # 1. de-duplicate
        seen = set()
        for r in sol.routes:
            cleaned = []
            for c in r.customers:
                if c in seen or c < 1 or c > inst.n:
                    continue
                cleaned.append(c)
                seen.add(c)
            r.customers = cleaned

        # missing customers go to the pool
        for c in range(1, inst.n + 1):
            if c not in seen:
                pool.append(c)

        # 2. trim infeasible routes
        for r in sol.routes:
            while r.customers and not r.is_feasible(inst):
                if r.has_nfz(inst):
                    seq = [0] + r.customers + [0]
                    drop_idx = None
                    for k, (a, b) in enumerate(zip(seq, seq[1:])):
                        if inst.is_arc_blocked(a, b):
                            drop_idx = min(k, len(r.customers) - 1)
                            break
                    if drop_idx is None:
                        break
                    pool.append(r.customers.pop(drop_idx))
                else:
                    pool.append(r.customers.pop())

        # 3. reinsert pool, cheapest insertion across existing routes
        pool = list(dict.fromkeys(pool))
        random.shuffle(pool)
        for c in pool:
            best = None  # (delta, route_idx, new_route)
            for k, r in enumerate(sol.routes):
                new_r, delta = self._try_insert(r, c)
                if new_r is not None and (best is None or delta < best[0]):
                    best = (delta, k, new_r)
            if best is not None:
                sol.routes[best[1]] = best[2]
            else:
                # try a fresh drone if one is available
                if sol.active_drones() < inst.K:
                    fresh = Route([c])
                    if fresh.is_feasible(inst):
                        placed = False
                        for k, r in enumerate(sol.routes):
                            if r.is_empty():
                                sol.routes[k] = fresh
                                placed = True
                                break
                        if not placed:
                            sol.routes.append(fresh)
                # if still nothing works the customer is left out

        while len(sol.routes) < inst.K:
            sol.routes.append(Route())

        return sol
