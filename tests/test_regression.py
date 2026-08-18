"""Equivalence tests: the package must reproduce the published experimental results.

The numbers in results/cached-results.json were produced by the original notebook.
Greedy is deterministic and B&B is exact, so both must match exactly — this is what
guarantees the extraction from the notebook did not change any behaviour.
"""

import pytest

from dronevrp import BranchAndBoundSolver, GreedySolver, Repairer


def test_greedy_reproduces_published_energy(instances, published):
    """Greedy is deterministic: every instance must match the published value."""
    for inst in instances:
        solution = Repairer(inst).repair(GreedySolver(inst).solve())
        assert solution.total_energy(inst) == pytest.approx(
            published[inst.instance_id]["greedy"]["energy"], rel=1e-6
        ), f"greedy energy changed on {inst.instance_id}"


def test_greedy_solutions_are_feasible(instances):
    for inst in instances:
        solution = Repairer(inst).repair(GreedySolver(inst).solve())
        feasible, violations = solution.check_feasibility(inst)
        assert feasible, f"{inst.instance_id}: {violations}"


def test_bnb_reproduces_proven_optimum(small, published):
    """INST_01 is solved to proven optimality; the optimum must not drift."""
    solver = BranchAndBoundSolver(small, time_limit=120.0)
    solver.solve()
    assert not solver.timed_out
    assert solver.best_energy == pytest.approx(
        published["INST_01"]["bnb"]["energy"], rel=1e-6
    )


def test_bnb_optimum_beats_every_heuristic(small, published):
    """No heuristic may report a solution better than the proven optimum."""
    block = published["INST_01"]
    optimum = block["bnb"]["energy"]
    for method in ("greedy", "ga", "sa"):
        value = block[method]["energy"] if method == "greedy" else block[method]["best"]
        assert value >= optimum - 1e-9, f"{method} claims to beat the proven optimum"
