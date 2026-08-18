"""Simulated annealing over the relocate / 2-opt / swap neighbourhood."""

import math
import random
import time
from typing import List, Optional

import numpy as np

from ..instance import Instance
from ..operators import (NeighborhoodOperator, RelocateNeighbor, SwapBetweenNeighbor,
                         TwoOptNeighbor)
from ..repair import Repairer
from ..solution import Route, Solution
from .base import Solver
from .greedy import GreedySolver


class SimulatedAnnealingSolver(Solver):
    """Geometric-cooling SA over the three-move neighbourhood.
    Supports an optional wall-clock time_limit (used by the same-time study)."""

    def __init__(self, inst: Instance,
                 n_iter: Optional[int] = None,
                 T0: Optional[float] = None,
                 gamma: float = 0.995,
                 time_limit: Optional[float] = None,
                 seed: Optional[int] = None,
                 verbose: bool = False):
        super().__init__(inst)
        # scale iterations with n if not given
        self.n_iter = n_iter if n_iter is not None else max(5000, 200 * inst.n)
        self.T0 = T0
        self.gamma = gamma
        self.time_limit = time_limit
        self.seed = seed
        self.verbose = verbose

        self.neighbors: List[NeighborhoodOperator] = [
            RelocateNeighbor(),
            TwoOptNeighbor(),
            SwapBetweenNeighbor(),
        ]
        self.repairer = Repairer(inst)

        self.history: List[float] = []
        self.best_energy: float = math.inf
        self.best_solution: Optional[Solution] = None

    def _neighbor(self, sol: Solution) -> Solution:
        op = random.choice(self.neighbors)
        return self.repairer.repair(op.apply(sol))

    def solve(self) -> Solution:
        if self.seed is not None:
            random.seed(self.seed)
            np.random.seed(self.seed)
        t0 = time.time()
        inst = self.inst

        current = self.repairer.repair(GreedySolver(inst).solve())
        feas, _ = current.check_feasibility(inst)
        if not feas:
            current = Solution([Route([c]) for c in range(1, min(inst.n, inst.K) + 1)])
            while len(current.routes) < inst.K:
                current.routes.append(Route())
            current = self.repairer.repair(current)

        current_e = current.total_energy(inst)
        self.best_solution = current.copy()
        self.best_energy = current_e
        self.history.append(self.best_energy)

        if self.T0 is None:
            T = max(0.05 * current_e / math.log(1 / 0.37), 1e-4)
        else:
            T = self.T0

        for it in range(self.n_iter):
            # stop early if a wall-clock budget was given and has been exceeded
            if self.time_limit is not None and time.time() - t0 > self.time_limit:
                break
            cand = self._neighbor(current)
            feas, _ = cand.check_feasibility(inst)
            if not feas:
                T *= self.gamma
                self.history.append(self.best_energy)
                continue
            cand_e = cand.total_energy(inst)
            delta = cand_e - current_e
            if delta < 0 or random.random() < math.exp(-delta / max(T, 1e-12)):
                current = cand
                current_e = cand_e
                if current_e < self.best_energy:
                    self.best_energy = current_e
                    self.best_solution = current.copy()
            T *= self.gamma
            self.history.append(self.best_energy)

            if self.verbose and it % 1000 == 0:
                print(f"  it {it:5d}  T={T:.4f}  best={self.best_energy:.4f} kWh")

        self.runtime_sec = time.time() - t0
        return self.best_solution
