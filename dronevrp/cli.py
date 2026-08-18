"""Command line interface: solve one instance without opening the notebook."""

import argparse
from pathlib import Path

from .data import DEFAULT_DATA_PATH, load_instances
from .repair import Repairer
from .solvers import (BranchAndBoundSolver, GeneticAlgorithmSolver, GreedySolver,
                      SimulatedAnnealingSolver)

SOLVERS = {
    "greedy": GreedySolver,
    "bnb": BranchAndBoundSolver,
    "ga": GeneticAlgorithmSolver,
    "sa": SimulatedAnnealingSolver,
}


def main(argv=None) -> int:
    parser = argparse.ArgumentParser(prog="dronevrp", description=__doc__)
    parser.add_argument("--instance", default="INST_01",
                        help="instance id, e.g. INST_05 (default: INST_01)")
    parser.add_argument("--solver", default="sa", choices=sorted(SOLVERS),
                        help="which solver to run (default: sa)")
    parser.add_argument("--seed", type=int, default=0, help="random seed")
    parser.add_argument("--time-limit", type=float, default=None,
                        help="wall-clock budget in seconds (bnb, ga, sa)")
    parser.add_argument("--data", type=Path, default=DEFAULT_DATA_PATH,
                        help=f"dataset path (default: {DEFAULT_DATA_PATH})")
    parser.add_argument("--list", action="store_true",
                        help="list the available instances and exit")
    args = parser.parse_args(argv)

    instances, _ = load_instances(args.data)
    by_id = {inst.instance_id: inst for inst in instances}

    if args.list:
        for inst in instances:
            print(f"{inst.instance_id}  {inst.category:6s} n={inst.n:3d}  K={inst.K:3d}")
        return 0

    if args.instance not in by_id:
        parser.error(f"unknown instance {args.instance!r}; try --list")
    inst = by_id[args.instance]

    kwargs = {}
    if args.solver != "greedy":
        kwargs["seed" if args.solver != "bnb" else "time_limit"] = (
            args.seed if args.solver != "bnb" else (args.time_limit or 600.0))
    if args.time_limit is not None and args.solver in ("ga", "sa"):
        kwargs["time_limit"] = args.time_limit

    solver = SOLVERS[args.solver](inst, **kwargs)
    solution = Repairer(inst).repair(solver.solve())
    feasible, violations = solution.check_feasibility(inst)

    print(f"{inst.instance_id}  n={inst.n}  K={inst.K}  solver={args.solver}")
    print(f"  energy   : {solution.total_energy(inst):.4f} kWh")
    print(f"  drones   : {solution.active_drones()} of {inst.K}")
    print(f"  runtime  : {solver.runtime_sec:.2f} s")
    print(f"  feasible : {feasible}" + ("" if feasible else f" -- {violations}"))
    for k, route in enumerate(solution.routes, 1):
        if not route.is_empty():
            print(f"  drone {k}: depot -> " + " -> ".join(map(str, route.customers)) + " -> depot")
    return 0 if feasible else 1


if __name__ == "__main__":
    raise SystemExit(main())
