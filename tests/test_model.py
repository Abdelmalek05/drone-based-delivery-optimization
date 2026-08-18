"""Unit tests for the energy model and feasibility rules, on a hand-built instance."""

import numpy as np
import pytest

from dronevrp import Instance, Repairer, Route, Solution

ALPHA, BETA = 0.1, 0.01


def make_instance(n=2, K=2, Q=5.0, B=100.0, blocked=frozenset()):
    """Depot + n customers on a line, with distances chosen for hand computation."""
    dist = np.array([[0.0, 1.0, 3.0],
                     [1.0, 0.0, 2.0],
                     [3.0, 2.0, 0.0]])
    return Instance(
        instance_id="TEST", category="unit", n=n, K=K, Q=Q, B=B,
        alpha=ALPHA, beta=BETA, dist=dist, demand=np.array([0.0, 1.0, 2.0]),
        blocked=set(blocked), customers=[], depot={},
    )


def test_energy_matches_hand_computation():
    """Payload drops at each drop, and the drone flies home empty."""
    inst = make_instance()
    # leg 0->1 carrying 3 kg : 1 * (0.1 + 0.01*3) = 0.13
    # leg 1->2 carrying 2 kg : 2 * (0.1 + 0.01*2) = 0.24
    # leg 2->0 empty         : 3 * 0.1            = 0.30
    assert Route([1, 2]).energy(inst) == pytest.approx(0.67)


def test_energy_depends_on_visit_order():
    """The whole point of the load-dependent model: order changes cost."""
    inst = make_instance()
    assert Route([1, 2]).energy(inst) != pytest.approx(Route([2, 1]).energy(inst))


def test_empty_route_costs_nothing():
    assert Route([]).energy(make_instance()) == 0.0


def test_payload_capacity_is_enforced():
    inst = make_instance(Q=2.5)          # route [1, 2] needs 3 kg
    assert not Route([1, 2]).is_feasible(inst)
    assert Route([2]).is_feasible(inst)


def test_battery_capacity_is_enforced():
    inst = make_instance(B=0.5)          # route [1, 2] needs 0.67 kWh
    assert not Route([1, 2]).is_feasible(inst)


def test_blocked_arc_makes_route_infeasible():
    inst = make_instance(blocked=[frozenset({1, 2})])
    assert not Route([1, 2]).is_feasible(inst)
    assert Route([1]).is_feasible(inst)   # depot legs are still clear


def test_blocked_arc_is_undirected():
    inst = make_instance(blocked=[frozenset({1, 2})])
    assert inst.is_arc_blocked(1, 2) and inst.is_arc_blocked(2, 1)


class TestRepairer:
    def test_removes_duplicate_visits(self):
        inst = make_instance()
        repaired = Repairer(inst).repair(Solution([Route([1, 1, 2]), Route()]))
        assert sorted(repaired.visited_customers()) == [1, 2]

    def test_reinserts_missing_customers(self):
        inst = make_instance()
        repaired = Repairer(inst).repair(Solution([Route([1]), Route()]))
        assert sorted(repaired.visited_customers()) == [1, 2]

    def test_splits_an_overloaded_route(self):
        inst = make_instance(Q=2.5)
        repaired = Repairer(inst).repair(Solution([Route([1, 2]), Route()]))
        assert sorted(repaired.visited_customers()) == [1, 2]
        feasible, violations = repaired.check_feasibility(inst)
        assert feasible, violations

    def test_output_always_has_K_routes(self):
        inst = make_instance(K=4)
        assert len(Repairer(inst).repair(Solution([Route([1, 2])])).routes) == 4
