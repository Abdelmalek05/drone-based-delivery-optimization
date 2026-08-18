"""Solver-level tests, including an exhaustive check that branch and bound is exact."""

import itertools

import numpy as np
import pytest

from dronevrp import (BranchAndBoundSolver, GeneticAlgorithmSolver, GreedySolver,
                      Instance, Repairer, Route, SimulatedAnnealingSolver, Solution)


@pytest.fixture
def tiny():
    """5 customers, 2 drones — small enough to enumerate every possible solution."""
    rng = np.random.default_rng(7)
    n = 5
    pts = rng.uniform(0, 10, size=(n + 1, 2))
    dist = np.linalg.norm(pts[:, None, :] - pts[None, :, :], axis=-1)
    return Instance(
        instance_id="TINY", category="unit", n=n, K=2, Q=6.0, B=50.0,
        alpha=0.044, beta=0.008, dist=dist,
        demand=np.array([0.0, 1.0, 2.0, 1.0, 2.0, 1.0]),
        blocked=set(), customers=[], depot={},
    )


def brute_force_optimum(inst):
    """Enumerate every assignment of customers to drones and every visit order."""
    customers = list(range(1, inst.n + 1))
    best = float("inf")
    for assignment in itertools.product(range(inst.K), repeat=inst.n):
        groups = [[c for c, k in zip(customers, assignment) if k == d]
                  for d in range(inst.K)]
        for orders in itertools.product(*(itertools.permutations(g) for g in groups)):
            routes = [Route(list(o)) for o in orders]
            if not all(r.is_feasible(inst) for r in routes):
                continue
            best = min(best, Solution(routes).total_energy(inst))
    return best


def test_branch_and_bound_finds_the_true_optimum(tiny):
    """The exact solver must match exhaustive enumeration, not merely look plausible."""
    solver = BranchAndBoundSolver(tiny, time_limit=60.0)
    solver.solve()
    assert not solver.timed_out
    assert solver.best_energy == pytest.approx(brute_force_optimum(tiny), rel=1e-9)


def test_no_heuristic_beats_the_optimum(tiny):
    optimum = brute_force_optimum(tiny)
    for solver in (GeneticAlgorithmSolver(tiny, pop_size=20, n_gen=30, seed=0),
                   SimulatedAnnealingSolver(tiny, n_iter=500, seed=0)):
        solver.solve()
        assert solver.best_energy >= optimum - 1e-9


@pytest.mark.parametrize("solver_cls,kwargs", [
    (GeneticAlgorithmSolver, dict(pop_size=20, n_gen=20)),
    (SimulatedAnnealingSolver, dict(n_iter=300)),
])
def test_same_seed_gives_identical_results(tiny, solver_cls, kwargs):
    """Reproducibility: the experimental protocol depends on seeds being honoured."""
    first = solver_cls(tiny, seed=123, **kwargs)
    first.solve()
    second = solver_cls(tiny, seed=123, **kwargs)
    second.solve()
    assert first.best_energy == second.best_energy


@pytest.mark.parametrize("solver_cls,kwargs", [
    (GreedySolver, {}),
    (GeneticAlgorithmSolver, dict(pop_size=20, n_gen=20, seed=0)),
    (SimulatedAnnealingSolver, dict(n_iter=300, seed=0)),
])
def test_solvers_return_feasible_solutions(tiny, solver_cls, kwargs):
    solution = Repairer(tiny).repair(solver_cls(tiny, **kwargs).solve())
    feasible, violations = solution.check_feasibility(tiny)
    assert feasible, violations


def test_metaheuristics_improve_on_greedy(instances):
    """On a medium instance, SA must beat the greedy baseline it starts from."""
    inst = instances[4]                                    # INST_05, 25 customers
    greedy = Repairer(inst).repair(GreedySolver(inst).solve()).total_energy(inst)
    sa = SimulatedAnnealingSolver(inst, seed=0)
    sa.solve()
    assert sa.best_energy < greedy
