"""The four solvers, all sharing the :class:`Solver` interface."""

from .annealing import SimulatedAnnealingSolver
from .base import Solver
from .branch_and_bound import BranchAndBoundSolver
from .genetic import GeneticAlgorithmSolver
from .greedy import GreedySolver

__all__ = ["Solver", "GreedySolver", "BranchAndBoundSolver",
           "GeneticAlgorithmSolver", "SimulatedAnnealingSolver"]
