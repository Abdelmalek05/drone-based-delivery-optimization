> **Archival note.** This is an early report describing a first implementation (`metaheuristics/greedy.ipynb`, `genetic.ipynb`, `solver_base.py`) that has since been replaced by the single `notebook.ipynb` at the repository root. Its numbers and file references do not match the current code; it is kept as a record of how the project evolved.

# Report v1 — Metaheuristic Route Optimization

## 1. Overview

This report documents the structure, implementation, and measured behavior of the notebook-based metaheuristic workflow in `drone-delivery-optimization/metaheuristics/`:

- `greedy.ipynb`
- `genetic.ipynb`

The notebooks are built on top of the shared problem model in `solver_base.py` and are used to solve a drone delivery route optimization problem for the Algiers dataset. The goal is to maximize delivery quality while minimizing total energy, constraint violations, unserved customers, and runtime.

The system evaluates solutions using a fitness-based objective where lower is better. In the reported results, the fitness value is dominated by a large penalty term, so the most meaningful comparisons come from the secondary metrics:

- total energy consumption
- number of violations
- number of unserved customers
- number of routes used
- runtime

---

## 2. Shared Design Concepts

Both notebooks share a similar execution pattern:

1. Import the shared optimization engine.
2. Build or load a `Problem` instance.
3. Produce a route solution.
4. Evaluate that solution.
5. Compute summary metrics.
6. Print a human-readable report.
7. Return or display the result for later interpretation.

The notebooks are intentionally annotated cell by cell so the code and the explanation stay together.

### Key shared ideas

- `Problem` comes from `solver_base.py`
- solutions are represented as `List[Tuple[int, List[int]]]`
  - the first value is a drone or route identifier
  - the second value is a list of customer indices in the order they are visited
- evaluation is centralized in `problem.evaluate(solution)`
- reporting is done through helper functions rather than inline logic
- execution timing uses `perf_counter()`

This is a good modular design because it separates:
- search strategy
- problem evaluation
- reporting
- explanation/documentation

---

## 3. `greedy.ipynb` — Structure and Implementation

### 3.1 Purpose

`greedy.ipynb` demonstrates the greedy heuristic for drone delivery route optimization. It serves as the baseline notebook for comparison with the genetic notebook.

### 3.2 Notebook structure

The notebook now contains:

1. title and overview markdown
2. imports and module setup
3. explanation markdown for imports
4. `solution_summary`
5. explanation markdown for solution summary
6. `run_greedy`
7. explanation markdown for greedy execution
8. `print_report`
9. explanation markdown for the report
10. `main` and script guard
11. explanation markdown for the entry point

This means the notebook is not just a result dump; it is a guided walkthrough of the exact greedy implementation.

### 3.3 Imports and module setup

The import cell contains the exact setup needed to run the greedy solver:

- `random`
- `sys`
- `Path`
- `perf_counter`
- typing utilities
- `Problem`
- `greedy_solve`

The `sys.path` handling is included so the code remains runnable from notebook and script contexts.

### 3.4 `solution_summary`

This helper evaluates the solution and extracts comparable metrics:

- fitness
- energy in kWh
- constraint violations
- unserved customers
- served customers
- number of routes used

It works by calling:

```python
fitness, energy, violations, unserved = problem.evaluate(solution)
```

Then it derives:
- `served = problem.n - len(unserved)`
- `routes = sum(1 for _, route in solution if route)`

This function is important because it standardizes metrics for later comparison.

### 3.5 `run_greedy`

This is the main solver entry point.

#### Steps performed

1. seed the random number generator with `random.seed(seed)`
2. create the optimization problem with `Problem()`
3. measure start time using `perf_counter()`
4. solve using `greedy_solve(problem)`
5. measure elapsed time
6. compute summary metrics
7. add runtime to the metrics dictionary
8. return a structured result

The returned dictionary contains:

- `problem`
- `solution`
- `metrics`

### 3.6 `print_report`

This function prints a formatted console report. It shows:

- title header
- pretty-printed solution from `problem.format(solution)`
- energy
- violations
- unserved customers
- routes used
- runtime

If any customers remain unserved, it also prints their names by looking them up in `problem.customers`.

### 3.7 `main`

`main()` simply calls `run_greedy()`, extracts the returned pieces, prints the report, and returns the result dictionary.

### 3.8 Implementation characteristics

The greedy notebook is intentionally lightweight:
- no population
- no crossover
- no mutation
- no iterative search loop inside the notebook itself

It delegates the actual greedy construction to `greedy_solve()` from `solver_base.py`. That means the notebook is mainly an orchestration and documentation layer around the core greedy heuristic.

### 3.9 Strengths

- very fast
- simple to understand
- easy to reproduce
- good baseline for comparison

### 3.10 Limitations

- no global optimization beyond the greedy decision process
- can get stuck in locally reasonable but globally suboptimal layouts
- no mechanism for improving a route after the initial construction

---

## 4. `genetic.ipynb` — Structure and Implementation

### 4.1 Purpose

`genetic.ipynb` demonstrates the genetic algorithm for route optimization. Compared with the greedy notebook, it explores a much larger search space by evolving permutations of customer visits over many generations.

### 4.2 Notebook structure

The notebook contains:

1. title and overview markdown
2. imports and module setup
3. explanation markdown for imports
4. `GeneticSolver`
5. explanation markdown for the solver
6. `summarize_solution`
7. explanation markdown for solution summary
8. `run_genetic`
9. explanation markdown for genetic execution
10. `print_report`, `main`, and script guard
11. explanation markdown for the final entry point

This notebook is the annotated version of the full genetic search workflow.

### 4.3 Imports and module setup

The import cell includes the exact setup needed for the genetic solver:

- `random`
- `sys`
- `Path`
- `perf_counter`
- typing utilities
- `Problem`
- `greedy_solve`

The greedy solver is used as a seed source for the initial population.

### 4.4 `GeneticSolver.__init__`

The constructor stores all genetic algorithm parameters and enforces safe bounds:

- `pop_size = max(2, pop_size)`
- `generations = max(1, generations)`
- `tournament_size = max(2, min(tournament_size, self.pop_size))`
- `elitism = max(0, min(elitism, self.pop_size - 1))`

This prevents invalid configurations such as:
- population size smaller than 2
- tournament size larger than the population
- elitism equal to the entire population

The default parameters are:

- population size: `80`
- generations: `300`
- tournament size: `3`
- crossover rate: `0.85`
- mutation rate: `0.15`
- elitism: `2`

These values indicate a moderately exploratory GA with some preservation of the best individuals.

### 4.5 `fitness(perm)`

This method evaluates one chromosome.

#### Input
- `perm`: a permutation of customer indices

#### Process
1. decode the permutation into a route solution using `self.problem.decode(perm)`
2. evaluate the decoded solution with `self.problem.evaluate(solution)`
3. apply an additional penalty for unserved customers:
   ```python
   fit += 1000 * len(unserved)
   ```

#### Output
A tuple containing:
- fitness
- energy
- violations
- solution
- unserved set

This penalty is critical: it discourages solutions that omit customers.

### 4.6 `create_individual`

Generates a random chromosome by shuffling the list of all customer indices.

This ensures every candidate is a valid permutation before decoding.

### 4.7 `seed_individual`

This is one of the most important design choices in the GA.

Instead of starting the population entirely from random permutations, the algorithm injects a greedy-based seed:

1. run the greedy solver
2. collect the route order from the greedy solution
3. append any missing customers that were not already served
4. return that permutation

If the greedy seed cannot be constructed, the method falls back to a random individual.

#### Why this matters

This hybrid design combines:
- exploitation from the greedy heuristic
- exploration from genetic evolution

That often improves convergence speed and solution quality.

### 4.8 `tournament_select`

This function performs tournament selection.

#### How it works
- sample `tournament_size` candidates from the population
- compare their fitness values
- return the best candidate

Tournament selection is useful because it balances:
- selection pressure
- diversity
- implementation simplicity

The function copies the chosen individual to avoid modifying the original population entry later.

### 4.9 `ox_crossover`

This implements Order Crossover (OX), which is suitable for permutation problems.

#### Process
1. choose two cut points `a` and `b`
2. copy the slice `parent1[a:b]` into the child
3. preserve gene order from `parent2`
4. fill the remaining positions with genes not already used

#### Why OX is appropriate

Since the problem is a permutation-based routing problem, OX preserves:
- relative ordering
- permutation validity
- gene uniqueness

That is better than using a generic crossover designed for binary or fixed-length numeric vectors.

### 4.10 `mutate`

The mutation operator uses three possible actions:

1. swap two genes
2. move one gene to another position
3. reverse a subsequence

This is a strong choice because it introduces different local search effects:

- swap: small perturbation
- insertion: reorders a route more substantially
- reversal: can improve route structure by inverting a segment

These mutation styles are well suited to routing problems.

### 4.11 `build_initial_population`

This method creates the initial population.

#### Strategy
- generate random individuals
- replace the first one with the greedy seed
- optionally set the second one to the reverse of the seed

This gives the algorithm:
- one strong starting point
- one contrasted starting point
- a surrounding random search space

The reversed seed is a simple but useful way to introduce diversity early.

### 4.12 `solve`

This is the core GA loop.

#### Main steps

For each generation:

1. evaluate the entire population
2. identify the best individual of the generation
3. record history statistics
4. update the global best if needed
5. create the next population using:
   - elitism
   - tournament selection
   - crossover
   - mutation
   - fallback repair if chromosome length is invalid
6. print periodic progress every 50 generations and on the final generation

#### History tracking

The solver stores per-generation metrics:
- generation number
- fitness
- energy
- violations
- unserved customers

This allows later analysis of convergence behavior.

#### Elitism

The best individuals are copied directly to the next generation. This prevents losing good solutions through crossover or mutation.

#### Safety check

If a child chromosome ends up with the wrong length, it is replaced by a fresh random individual. This is a practical repair step to avoid invalid solutions.

### 4.13 `summarize_solution`

Like the greedy version, this helper computes standardized metrics from a final solution:
- fitness
- energy
- violations
- unserved
- served
- routes

This ensures the final result dictionary is directly comparable to the greedy one.

### 4.14 `run_genetic`

This wrapper:
1. seeds randomness
2. creates `Problem()`
3. instantiates `GeneticSolver`
4. runs the algorithm
5. measures runtime
6. summarizes the final solution
7. returns a structured result dictionary

Returned fields include:
- `problem`
- `solution`
- `metrics`
- `history`
- `fitness`
- `permutation`

This makes the genetic output richer than the greedy output because it includes evolutionary history.

### 4.15 `print_report` and `main`

These are equivalent in role to the greedy notebook:
- print a formatted report
- call the runner
- expose a command-line entry point

### 4.16 Strengths

- searches a much larger solution space
- can improve upon greedy initialization
- supports route permutation optimization
- tracks convergence history
- more capable of escaping local minima

### 4.17 Limitations

- significantly slower than greedy
- depends on parameter tuning
- results may vary across runs
- still not guaranteed to find the global optimum

---

## 5. Results Analysis

The generated comparison results are stored in `comparative_results.json`. The reported numbers are:

### 5.1 Greedy results

- Fitness: `30011.483217219014`
- Energy: `11.483217219013351 kWh`
- Constraint violations: `3`
- Unserved customers: `0`
- Served customers: `20`
- Routes used: `13`
- Runtime: `0.0054 s` in notebook execution

### 5.2 Genetic best results

- Fitness: `30010.631158035718`
- Energy: `10.631158035716485 kWh`
- Constraint violations: `3`
- Unserved customers: `0`
- Served customers: `20`
- Routes used: `13`
- Runtime: `6.5564 s` in notebook execution

### 5.3 Genetic average results across 3 trials

- Fitness: `30010.631158035718`
- Energy: `10.631158035716485 kWh`
- Constraint violations: `3`
- Unserved customers: `0`
- Runtime: `5.5863215333083645 s`

### 5.4 Improvement vs greedy

- Fitness improvement: `0.0028391105402207958%`
- Energy improvement: `7.420038888457742%`
- Runtime change: `-196736.52140910365%`

---

## 6. Interpretation of the Results

### 6.1 Fitness

The fitness improvement is extremely small:
- only about `0.00284%`

This indicates that the genetic algorithm improved the objective only marginally over greedy in this experiment.

That small difference likely means:
- the greedy method already produced a near-competitive solution
- or the penalty structure in fitness makes the objective dominated by a large constant term

Because fitness is around `30010`, the actual energy differences are masked by the bigger fitness scale.

### 6.2 Energy

Energy is where the genetic algorithm shows a clearer benefit:

- Greedy: `11.4832 kWh`
- Genetic best: `10.6312 kWh`

That is a reduction of about `7.42%`.

This is the most meaningful improvement in the comparison. It means the GA found a route arrangement that consumes less energy while maintaining the same number of violations, served customers, and routes.

### 6.3 Constraint violations

Both algorithms produced:
- `3` violations

So the genetic algorithm did not improve feasibility in this run. It optimized energy, not constraint satisfaction.

### 6.4 Unserved customers

Both methods served all customers:
- `0` unserved
- `20` served

This is important because it shows the penalty logic in the GA did its job: it did not sacrifice coverage.

### 6.5 Routes used

Both methods used:
- `13` routes

So the genetic algorithm improved route quality without changing the route count.

### 6.6 Runtime

This is the biggest tradeoff.

- Greedy runtime: about `0.0054 s` in notebook execution
- Genetic runtime: about `6.5564 s` in notebook execution

The genetic algorithm is dramatically slower, which is expected because it evaluates many candidate solutions over many generations.

---

## 7. Overall Assessment

### Greedy notebook
Best for:
- speed
- baseline solutions
- quick feasibility checks
- low computational cost

### Genetic notebook
Best for:
- better route refinement
- reducing energy
- exploring more complex search spaces
- generating stronger solutions when runtime is acceptable

### Notebook value

The notebooks are useful because they turn the code into a guided explanation:
- they show how the solver is constructed
- they reveal intermediate evaluation points
- they help the reader interpret the final result, not just run it

### Comparative conclusion

The results show a classic metaheuristic tradeoff:

- **Greedy** is much faster.
- **Genetic** produces a slightly better solution in terms of fitness and a noticeably better one in terms of energy.
- **Genetic** costs far more runtime.

In practical terms, the genetic algorithm is useful when route quality matters more than execution time. The greedy solver remains valuable as a fast baseline and as a seed generator for the genetic population.

---

## 8. Implementation Quality Notes

### Positive aspects

- clear separation of concerns
- reusable summary functions
- consistent metrics across methods
- deterministic seeding in notebook runs
- useful convergence history in the genetic notebook
- notebook documentation that explains each execution step

### Potential improvements

- add a richer objective breakdown to show how fitness is composed
- reduce the dominance of the fitness penalty constant so improvements are easier to interpret
- run the genetic notebook more than three times for more stable averages
- include standard deviation in the notebook summary
- save the full genetic history to file for post-analysis
- add more optimization strategies, such as local search after crossover or mutation
- add compact summary tables in the notebook outputs

---

## 9. Final Conclusion

The metaheuristic codebase now forms a notebook-based optimization and analysis pipeline:

- `greedy.ipynb` provides a fast baseline heuristic with explanatory cells.
- `genetic.ipynb` provides a stronger but slower evolutionary optimizer with explanatory cells.
- `solver_base.py` remains the shared problem engine.
- `comparative_results.json` stores the measured comparison summary.

The measured experiment shows that the genetic algorithm slightly improves solution quality, especially in energy consumption, while significantly increasing runtime. The greedy approach remains the best choice for speed, but the genetic approach is preferable when a small quality gain is worth the additional compute time.

Overall, the notebook-based workflow is well structured, modular, and suitable for comparative experimentation in drone delivery route optimization.
