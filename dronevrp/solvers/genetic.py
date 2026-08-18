"""Memetic genetic algorithm over permutation chromosomes."""

import math
import random
import time
from typing import List, Optional, Tuple

import numpy as np

from ..instance import Instance
from ..operators import MixedMutation, OrderCrossover, TwoOptNeighbor
from ..repair import Repairer
from ..solution import Route, Solution
from .base import Solver
from .greedy import GreedySolver


class GeneticAlgorithmSolver(Solver):
    """Permutation-encoded GA with OX, mixed mutation, tournament selection,
    elitism, 2-opt local search after crossover (memetic), and greedy seeding.
    Supports an optional wall-clock time_limit (used by the same-time study)."""

    def __init__(self, inst: Instance,
                 pop_size: int = 60,
                 n_gen: int = 250,
                 p_cx: float = 0.9,
                 p_mut: float = 0.2,
                 elite_size: int = 2,
                 tournament_k: int = 3,
                 p_local: float = 0.5,
                 time_limit: Optional[float] = None,
                 seed: Optional[int] = None,
                 verbose: bool = False):
        super().__init__(inst)
        self.pop_size = pop_size
        self.n_gen = n_gen
        self.p_cx = p_cx
        self.p_mut = p_mut
        self.elite_size = elite_size
        self.tournament_k = tournament_k
        self.p_local = p_local
        self.time_limit = time_limit
        self.seed = seed
        self.verbose = verbose

        self.crossover = OrderCrossover()
        self.mutation = MixedMutation()
        self.repairer = Repairer(inst)
        self.two_opt = TwoOptNeighbor()

        self.history: List[float] = []
        self.best_energy: float = math.inf
        self.best_solution: Optional[Solution] = None

    def _split_into_routes(self, perm: List[int]) -> Solution:
        """Decode a permutation into AT MOST K routes.

        We pack customers left-to-right into the current route while it stays
        feasible. When the next customer no longer fits, we open a NEW route
        for it -- but only while fewer than K drones are in use. Once all K
        drones are committed, any customer that does not fit the current route
        is left out of the decode here; the Repairer then cheapest-inserts
        those leftover customers into the existing routes.

        This guarantees the decoded solution never uses more than K routes,
        which fixes the previous behaviour: an over-K decode was always
        rejected as infeasible (fitness = +inf), so a large fraction of the
        population was silently wasted on the smaller / tighter instances.
        """
        inst = self.inst
        routes: List[Route] = []
        current = Route()
        for c in perm:
            # route-slots already committed (current counts only if non-empty)
            slots_used = len(routes) + (0 if current.is_empty() else 1)
            trial = Route(current.customers + [c])
            if trial.is_feasible(inst):
                current = trial
            elif slots_used < inst.K and Route([c]).is_feasible(inst):
                # a free drone is still available: close current, open a new route
                if not current.is_empty():
                    routes.append(current)
                current = Route([c])
            else:
                # no free drone (or c is infeasible on its own): skip c here,
                # the Repairer will insert it into the cheapest feasible route
                pass
        if not current.is_empty():
            routes.append(current)
        while len(routes) < inst.K:
            routes.append(Route())
        return Solution(routes)

    def _evaluate(self, perm: List[int]) -> Tuple[float, Solution]:
        sol = self.repairer.repair(self._split_into_routes(perm))
        feas, _ = sol.check_feasibility(self.inst)
        if not feas:
            return math.inf, sol
        return sol.total_energy(self.inst), sol

    def _tournament(self, scored):
        contenders = random.sample(scored, min(self.tournament_k, len(scored)))
        return min(contenders, key=lambda x: x[0])[1]

    def _local_search(self, sol: Solution) -> Solution:
        best = sol
        best_e = sol.total_energy(self.inst)
        for _ in range(5):
            cand = self.repairer.repair(self.two_opt.apply(best))
            feas, _ = cand.check_feasibility(self.inst)
            if not feas:
                continue
            e = cand.total_energy(self.inst)
            if e < best_e:
                best = cand
                best_e = e
        return best

    def _greedy_permutation(self) -> List[int]:
        """Get the customer order produced by the greedy nearest-neighbour solver."""
        g_sol = GreedySolver(self.inst).solve()
        perm: List[int] = []
        for r in g_sol.routes:
            perm.extend(r.customers)
        # if greedy left some customers out, append them at the end in random order
        missing = [c for c in range(1, self.inst.n + 1) if c not in perm]
        random.shuffle(missing)
        perm.extend(missing)
        return perm

    def solve(self) -> Solution:
        if self.seed is not None:
            random.seed(self.seed)
            np.random.seed(self.seed)
        t0 = time.time()
        n = self.inst.n

        # initial population: one greedy chromosome + random ones
        population: List[List[int]] = [self._greedy_permutation()]
        for _ in range(self.pop_size - 1):
            p = list(range(1, n + 1))
            random.shuffle(p)
            population.append(p)

        scored: List[Tuple[float, List[int], Solution]] = []
        for p in population:
            e, s = self._evaluate(p)
            scored.append((e, p, s))

        scored.sort(key=lambda x: x[0])
        self.best_energy = scored[0][0]
        self.best_solution = scored[0][2]
        self.history.append(self.best_energy)

        for gen in range(self.n_gen):
            # stop early if a wall-clock budget was given and has been exceeded
            if self.time_limit is not None and time.time() - t0 > self.time_limit:
                break
            new_pop: List[List[int]] = []

            # elitism
            for i in range(self.elite_size):
                new_pop.append(scored[i][1][:])

            tour_pool = [(e, p) for (e, p, _) in scored]

            while len(new_pop) < self.pop_size:
                p1 = self._tournament(tour_pool)
                p2 = self._tournament(tour_pool)
                if random.random() < self.p_cx:
                    child = self.crossover.apply(p1, p2)
                else:
                    child = p1[:]
                if random.random() < self.p_mut:
                    child = self.mutation.apply(child)
                new_pop.append(child)

            scored = []
            for i, p in enumerate(new_pop):
                e, s = self._evaluate(p)
                if i >= self.elite_size and random.random() < self.p_local and e < math.inf:
                    s2 = self._local_search(s)
                    e2 = s2.total_energy(self.inst)
                    if e2 < e:
                        e, s = e2, s2
                scored.append((e, p, s))
            scored.sort(key=lambda x: x[0])

            if scored[0][0] < self.best_energy:
                self.best_energy = scored[0][0]
                self.best_solution = scored[0][2]
            self.history.append(self.best_energy)

            if self.verbose and gen % 20 == 0:
                print(f"  gen {gen:3d}  best={self.best_energy:.4f} kWh")

        self.runtime_sec = time.time() - t0
        return self.best_solution
