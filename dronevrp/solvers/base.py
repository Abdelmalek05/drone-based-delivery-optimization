"""Abstract solver interface."""

from abc import ABC, abstractmethod

from ..instance import Instance
from ..solution import Solution


class Solver(ABC):
    """Abstract base for all solvers."""

    def __init__(self, inst: Instance):
        self.inst = inst
        self.runtime_sec: float = 0.0

    @abstractmethod
    def solve(self) -> Solution:
        ...
