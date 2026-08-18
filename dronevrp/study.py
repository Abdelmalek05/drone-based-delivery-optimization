"""The experimental study: run all solvers on all instances, or load cached results.

The parameters below are the published experimental budget. Changing them invalidates
`results/cached-results.json`, which the notebook and README both read.
"""

import json
import math
from pathlib import Path
from typing import List

import numpy as np
import pandas as pd
from scipy import stats

from .instance import Instance
from .repair import Repairer
from .solvers import (BranchAndBoundSolver, GeneticAlgorithmSolver, GreedySolver,
                      SimulatedAnnealingSolver)

DEFAULT_RESULTS_PATH = Path("results/cached-results.json")

N_RUNS = 15
BNB_TIME_LIMIT = 1200.0      # 20 minutes per instance
BNB_MAX_N = 15               # tighter bound makes n=15 feasible within the budget
GA_PARAMS = dict(pop_size=60, n_gen=250, p_cx=0.9, p_mut=0.2,
                 elite_size=2, p_local=0.5)
SA_GAMMA = 0.995             # SA n_iter is auto-scaled with n


def run_full_study(instances: List[Instance],
                   results_path: Path = DEFAULT_RESULTS_PATH,
                   force_rerun: bool = False) -> dict:
    if results_path.exists() and not force_rerun:
        print(f"Loading cached results from {results_path}")
        with open(results_path) as f:
            return json.load(f)

    results = {"per_instance": {}}
    for inst in instances:
        print(f"\n=== {inst.instance_id} ({inst.category}, n={inst.n}) ===")
        block = {"category": inst.category, "n": inst.n, "K": inst.K}

        # Greedy
        g = GreedySolver(inst)
        g_sol = Repairer(inst).repair(g.solve())
        g_feas, _ = g_sol.check_feasibility(inst)
        block["greedy"] = {
            "energy": g_sol.total_energy(inst) if g_feas else None,
            "runtime_sec": g.runtime_sec,
            "feasible": g_feas,
        }
        print(f"  greedy : E={block['greedy']['energy']}  feasible={g_feas}")

        # B&B (only on small enough instances)
        if inst.n <= BNB_MAX_N:
            bnb = BranchAndBoundSolver(inst, time_limit=BNB_TIME_LIMIT)
            bnb.solve()
            block["bnb"] = {
                "energy": bnb.best_energy if bnb.best_energy < math.inf else None,
                "runtime_sec": bnb.runtime_sec,
                "nodes_explored": bnb.nodes_explored,
                "timed_out": bnb.timed_out,
            }
            status = "TIMED OUT" if bnb.timed_out else "PROVEN OPTIMAL"
            print(f"  B&B    : E={bnb.best_energy:.4f}  nodes={bnb.nodes_explored:,}  "
                  f"t={bnb.runtime_sec:.1f}s  {status}")
        else:
            block["bnb"] = {"energy": None, "runtime_sec": None,
                            "nodes_explored": None, "timed_out": True, "skipped": True}
            print(f"  B&B    : skipped (n > {BNB_MAX_N})")

        # GA, N_RUNS times
        ga_e, ga_t, ga_h = [], [], []
        for seed in range(N_RUNS):
            ga = GeneticAlgorithmSolver(inst, seed=seed, **GA_PARAMS)
            ga.solve()
            ga_e.append(ga.best_energy)
            ga_t.append(ga.runtime_sec)
            ga_h.append(ga.history)
        block["ga"] = {
            "energies": ga_e,
            "best": float(min(ga_e)),
            "mean": float(np.mean(ga_e)),
            "std": float(np.std(ga_e)),
            "mean_runtime_sec": float(np.mean(ga_t)),
            "histories": [list(h) for h in ga_h],
        }
        print(f"  GA     : best={min(ga_e):.4f}  mean={np.mean(ga_e):.4f}  std={np.std(ga_e):.4f}")

        # SA, N_RUNS times
        sa_e, sa_t, sa_h = [], [], []
        for seed in range(N_RUNS):
            sa = SimulatedAnnealingSolver(inst, gamma=SA_GAMMA, seed=seed)
            sa.solve()
            sa_e.append(sa.best_energy)
            sa_t.append(sa.runtime_sec)
            sa_h.append(sa.history)
        block["sa"] = {
            "energies": sa_e,
            "best": float(min(sa_e)),
            "mean": float(np.mean(sa_e)),
            "std": float(np.std(sa_e)),
            "mean_runtime_sec": float(np.mean(sa_t)),
            "histories": [list(h) for h in sa_h],
        }
        print(f"  SA     : best={min(sa_e):.4f}  mean={np.mean(sa_e):.4f}  std={np.std(sa_e):.4f}")

        results["per_instance"][inst.instance_id] = block

    with open(results_path, "w") as f:
        json.dump(results, f, indent=2)
    return results


def build_summary_table(results: dict) -> pd.DataFrame:
    rows = []
    for inst_id, block in results["per_instance"].items():
        n_runs = len(block["ga"]["energies"])
        if block["ga"]["std"] > 0:
            ga_ci = stats.t.interval(0.95, n_runs - 1,
                                     loc=block["ga"]["mean"],
                                     scale=block["ga"]["std"] / math.sqrt(n_runs))
        else:
            ga_ci = (block["ga"]["mean"], block["ga"]["mean"])
        if block["sa"]["std"] > 0:
            sa_ci = stats.t.interval(0.95, n_runs - 1,
                                     loc=block["sa"]["mean"],
                                     scale=block["sa"]["std"] / math.sqrt(n_runs))
        else:
            sa_ci = (block["sa"]["mean"], block["sa"]["mean"])

        rows.append({
            "instance": inst_id,
            "n": block["n"],
            "K": block["K"],
            "category": block["category"],
            "greedy": block["greedy"]["energy"],
            "bnb": block["bnb"]["energy"],
            "bnb_to": block["bnb"].get("timed_out"),
            "ga_best": block["ga"]["best"],
            "ga_mean": block["ga"]["mean"],
            "ga_ci_low": ga_ci[0],
            "ga_ci_high": ga_ci[1],
            "sa_best": block["sa"]["best"],
            "sa_mean": block["sa"]["mean"],
            "sa_ci_low": sa_ci[0],
            "sa_ci_high": sa_ci[1],
        })
    return pd.DataFrame(rows)
