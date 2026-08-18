"""Crossover, mutation, and neighbourhood operators used by the GA and SA."""

import random
from abc import ABC, abstractmethod
from typing import List

from .solution import Solution


class Operator(ABC):
    """Abstract base for GA / SA operators."""
    pass


class CrossoverOperator(Operator):
    @abstractmethod
    def apply(self, p1: List[int], p2: List[int]) -> List[int]:
        ...


class MutationOperator(Operator):
    @abstractmethod
    def apply(self, perm: List[int]) -> List[int]:
        ...


class NeighborhoodOperator(Operator):
    """Operates on a Solution (used by SA)."""
    @abstractmethod
    def apply(self, sol: Solution) -> Solution:
        ...


class OrderCrossover(CrossoverOperator):
    """Classic OX for permutations."""

    def apply(self, p1: List[int], p2: List[int]) -> List[int]:
        n = len(p1)
        a, b = sorted(random.sample(range(n), 2))
        child = [None] * n
        child[a:b+1] = p1[a:b+1]
        in_child = set(p1[a:b+1])
        fill = (g for g in p2 if g not in in_child)
        for i in range(n):
            if child[i] is None:
                child[i] = next(fill)
        return child


class SwapMutation(MutationOperator):
    def apply(self, perm: List[int]) -> List[int]:
        p = perm[:]
        if len(p) < 2:
            return p
        i, j = random.sample(range(len(p)), 2)
        p[i], p[j] = p[j], p[i]
        return p


class RelocateMutation(MutationOperator):
    def apply(self, perm: List[int]) -> List[int]:
        p = perm[:]
        if len(p) < 2:
            return p
        i, j = random.sample(range(len(p)), 2)
        gene = p.pop(i)
        p.insert(j, gene)
        return p


class MixedMutation(MutationOperator):
    """50/50 swap or relocate."""
    def __init__(self):
        self.swap = SwapMutation()
        self.relocate = RelocateMutation()

    def apply(self, perm: List[int]) -> List[int]:
        if random.random() < 0.5:
            return self.swap.apply(perm)
        return self.relocate.apply(perm)


# Neighborhood operators for SA
class RelocateNeighbor(NeighborhoodOperator):
    """Move a single customer to a (possibly different) route."""

    def apply(self, sol: Solution) -> Solution:
        new = sol.copy()
        non_empty = [i for i, r in enumerate(new.routes) if not r.is_empty()]
        if not non_empty:
            return new
        src = random.choice(non_empty)
        pos_src = random.randrange(len(new.routes[src]))
        cust = new.routes[src].customers.pop(pos_src)
        dst = random.randrange(len(new.routes))
        target = new.routes[dst]
        pos_dst = random.randrange(len(target) + 1) if not target.is_empty() else 0
        target.customers.insert(pos_dst, cust)
        return new


class TwoOptNeighbor(NeighborhoodOperator):
    """Reverse a sub-sequence inside one route (intra-route 2-opt)."""

    def apply(self, sol: Solution) -> Solution:
        new = sol.copy()
        non_empty = [i for i, r in enumerate(new.routes) if len(r) >= 2]
        if not non_empty:
            return new
        idx = random.choice(non_empty)
        r = new.routes[idx]
        i, j = sorted(random.sample(range(len(r)), 2))
        r.customers[i:j+1] = list(reversed(r.customers[i:j+1]))
        return new


class SwapBetweenNeighbor(NeighborhoodOperator):
    """Swap one customer from route A with one from route B."""

    def apply(self, sol: Solution) -> Solution:
        new = sol.copy()
        non_empty = [i for i, r in enumerate(new.routes) if not r.is_empty()]
        if len(non_empty) < 2:
            return new
        a, b = random.sample(non_empty, 2)
        ia = random.randrange(len(new.routes[a]))
        ib = random.randrange(len(new.routes[b]))
        new.routes[a].customers[ia], new.routes[b].customers[ib] = (
            new.routes[b].customers[ib], new.routes[a].customers[ia]
        )
        return new
