"""Energy-optimal drone delivery routing over Algiers.

A battery-, payload- and no-fly-zone-constrained multi-drop vehicle routing problem,
solved with a greedy heuristic, exact branch and bound, a genetic algorithm, and
simulated annealing.
"""

from .data import load_dataset, load_instances
from .instance import Instance
from .repair import Repairer
from .solution import Route, Solution
from .solvers import (BranchAndBoundSolver, GeneticAlgorithmSolver, GreedySolver,
                      SimulatedAnnealingSolver, Solver)

__version__ = "1.0.0"

__all__ = [
    "Instance", "Route", "Solution", "Repairer", "Solver",
    "GreedySolver", "BranchAndBoundSolver", "GeneticAlgorithmSolver",
    "SimulatedAnnealingSolver", "load_instances", "load_dataset",
]
