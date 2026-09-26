# Implementation Plan: Energy-Aware Wind-Adaptive 3D Coverage Path Planning for Agricultural UAVs

Build a complete, modular, mathematically explicit, reproducible research framework for 3D agricultural UAV coverage path planning using Hybrid ACO-GA optimization and Mamdani Fuzzy environmental cost adaptation under controlled wind and terrain conditions.

## User Review Required

> [!IMPORTANT]
> The project will be built with high computational efficiency using NumPy vectorization so that `--quick` finishes in ~10-15 seconds (for smoke tests and quick verification) and `--full` executes reliably on standard laptop hardware within a few minutes while generating high-quality research outputs.

> [!NOTE]
> All code will reside in `SCOA_UAV/` (or root as required by workspace structure, following the specified directory layout with modular scripts and tests). We will place the modules directly in the project workspace directory:
> `c:\Users\Piyush Daspute\OneDrive\Desktop\Projects\SCOA\` matching the requested layout:
> `config.py`, `environment.py`, `coverage.py`, `energy.py`, `fitness.py`, `fuzzy_system.py`, `baseline.py`, `ga.py`, `aco.py`, `hybrid_aco_ga.py`, `experiments.py`, `statistics_analysis.py`, `visualization.py`, `main.py`, `tests/`, `results/`, `figures/`, and the 5 comprehensive markdown documentation files.

## Proposed System Architecture & Modules

1. **`config.py`**:
   - Central configuration dataclasses: `TerrainConfig`, `GridConfig`, `ObstacleConfig`, `WindConfig`, `UAVConfig`, `EnergyConfig`, `FitnessConfig`, `GAConfig`, `ACOConfig`, `ExperimentConfig`.
   - Preset configurations for `--quick` (10x10 grid, 5 GA generations, 5 ACO iterations, 5 ants, 1 trial) and `--full` (20x20 grid or 15x15 default full with configurable 20x20, 50-100 gens/iters, 20-30 ants, repeated trials).
   - Global seed configuration (`SEED = 42`).

2. **`environment.py`**:
   - Terrain generation: T1 (Flat), T2 (Rolling - sinusoidal + Gaussian), T3 (Complex - multi-frequency sinusoidal + dual Gaussians).
   - Deterministic elevation grid generation.
   - Circular obstacles: $(x, y, r, h, d_{safe})$.
   - Obstacle intersection / clearance verification for 3D path segments.

3. **`coverage.py`**:
   - Cell representation: `cell_id`, `(x, y)`, `terrain_z`, `uav_z = terrain_z + h_clearance`, `valid`, `obstacle`.
   - Grid decomposition (500m x 500m field).
   - Cell filtering: separates target valid cells from invalid obstacle cells.
   - Path coverage calculation: $CR = (N_{covered} / N_{target}) \times 100\%$.
   - Segment geometry & 3D Euclidean distance calculation: $d_i = \sqrt{\Delta x^2 + \Delta y^2 + \Delta z^2}$.
   - Turn angle calculation: $\theta = \arccos\left(\frac{\mathbf{v}_1 \cdot \mathbf{v}_2}{\|\mathbf{v}_1\| \|\mathbf{v}_2\|}\right)$, count turns where $\theta > \theta_{threshold}$.

4. **`fuzzy_system.py`**:
   - Pure NumPy implementation of Mamdani Fuzzy Inference System (no external heavyweight dependencies needed).
   - Inputs:
     - Wind speed ($0-12$ m/s): Low $[0, 0, 4]$, Moderate $[2, 6, 10]$, High $[8, 12, 12]$.
     - Relative wind direction ($0^\circ - 180^\circ$): Tailwind $[0, 0, 60]$, Crosswind $[30, 90, 150]$, Headwind $[120, 180, 180]$.
   - Output:
     - Environmental penalty factor $\mu_{wind} \in [1.0, 2.0]$: Minimal $[1.0, 1.0, 1.3]$, Moderate $[1.2, 1.5, 1.8]$, Severe $[1.6, 2.0, 2.0]$.
   - 7 Mamdani rules evaluated via min-implication, max-aggregation, and numerical centroid defuzzification over discrete universe.
   - Vectorized evaluation for fast fitness batch calls.

5. **`energy.py`**:
   - Dimensionally correct energy model:
     - Base / horizontal flight: $E_{horizontal, i} = P_{base} \cdot \mu_{wind, i} \cdot \frac{d_i}{v}$
     - Climb energy: $E_{climb, i} = P_{climb} \cdot \frac{\max(0, \Delta z_i)}{v_{climb}}$
     - Turn energy: $E_{turn} = P_{turn} \cdot N_{turns}$
     - Total energy: $E(P) = \sum E_{horizontal, i} + \sum E_{climb, i} + E_{turn}$
   - Relative wind angle calculation with proper wrap-around handling:
     $\theta_{flight} = \operatorname{atan2}(\Delta y, \Delta x)$, $\Delta\theta = ((\theta_{flight} - \theta_{wind} + 180) \bmod 360) - 180$, $\theta_{rel} = |\Delta\theta|$.

6. **`fitness.py`**:
   - Multi-objective normalized fitness function:
     $J(P) = w_E \hat{E} + w_D \hat{D} + w_T \hat{T} + w_C L_C + w_R R$
   - Constraint / obstacle penalty $R = M \cdot N_{violations}$.
   - Pre-computed distance matrix and vectorized evaluation for maximum speed.

7. **`baseline.py`**:
   - Deterministic Boustrophedon (Lawnmower) coverage path generator.
   - Snakes back and forth across valid cells, avoiding obstacles.
   - Serves as the primary non-metaheuristic baseline benchmark.

8. **`ga.py`**:
   - Permutation-based Genetic Algorithm:
     - Population initialization (random permutations of valid cell IDs).
     - Tournament selection.
     - Order Crossover (OX) ensuring valid permutations without duplicates or missing cells.
     - Permutation-safe mutation (inversion / 2-opt and swap).
     - Elitism (preserving top $k$ solutions).
     - Convergence tracking across generations.

9. **`aco.py`**:
   - Ant Colony Optimization for TSP/CPP permutation ordering:
     - Pheromone matrix $\tau_{ij}$ and distance/heuristic matrix $\eta_{ij} = \frac{1}{d_{ij} + \epsilon}$.
     - Probabilistic ant path construction using wheel/roulette selection.
     - Pheromone evaporation: $\tau_{ij} \leftarrow (1 - \rho)\tau_{ij}$.
     - Pheromone deposition: $\Delta\tau_{ij} = \frac{Q}{J(P_k)}$ on edges of elite solutions.
     - Convergence logging per iteration.

10. **`hybrid_aco_ga.py`**:
    - True hybridization:
      - Phase 1: ACO runs for $N_{aco}$ iterations to explore global topology and generate promising candidate routes.
      - Phase 2: Top $K$ unique ACO solutions seed the initial GA population alongside diverse permutations.
      - Phase 3: GA executes crossover, mutation, and selection to refine edge orderings.
    - Two variants supported:
      - `HybridACOGA` (standard distance/base energy guided)
      - `HybridACOGAFIS` (integrating the Mamdani FIS directly inside the fitness evaluation during both ACO and GA phases).

11. **`experiments.py`**:
    - Complete experimental harness:
      - Experiment 1 (Main Comparison): Lawnmower, GA, ACO, ACO-GA, ACO-GA-FIS under Rolling terrain, 6 m/s wind, 90°.
      - Experiment 2 (Wind Speed sensitivity): 0, 6, 12 m/s.
      - Experiment 3 (Wind Direction sensitivity): 0°, 90°, 180°.
      - Experiment 4 (Terrain Complexity): Flat, Rolling, Complex.
      - Experiment 5 (Ablation): ACO-GA vs ACO-GA-FIS across wind regimes.
    - Repeated trials management with deterministic random seeds ($42 + i$).
    - Automated CSV exports into `results/`.

12. **`statistics_analysis.py`**:
    - Descriptive statistics (Mean, SD, Min, Max).
    - Inferential statistics: Friedman test for multi-algorithm comparisons, Wilcoxon signed-rank test for paired comparisons (ACO-GA vs ACO-GA-FIS).
    - Summary tables export.

13. **`visualization.py`**:
    - High-quality matplotlib 3D and 2D figures:
      - `terrain.png`: 3D surface of the terrain.
      - `coverage_grid.png`: 2D/3D plot showing grid cells, obstacle zones, and safety buffers.
      - `lawnmower_path.png`: 3D visualization of the Lawnmower route.
      - `best_path_3d.png`: 3D visualization of the optimal ACO-GA-FIS path with elevation profile.
      - Comparison bar charts: `energy_comparison.png`, `coverage_comparison.png`, `distance_comparison.png`, `turns_comparison.png`, `runtime_comparison.png`.
      - Parametric sensitivity curves: `energy_vs_wind_speed.png`, `energy_vs_wind_direction.png`.
      - Convergence plot: `convergence_comparison.png`.

14. **`main.py`**:
    - CLI entrypoint supporting:
      - `python main.py` / `python main.py --quick` / `python main.py --full`
    - Automated validation checks (coverage $\ge 99\%$, energy $> 0$, distance $> 0$, no NaN/inf, no collisions, FIS $\in [1, 2]$).
    - Formatted terminal report matching the exact requested prompt format.

15. **Comprehensive Tests (`tests/`)**:
    - `test_environment.py`: Grid dimensions, terrain equations, obstacle safety buffer.
    - `test_coverage.py`: Valid cell filtering, coverage percentage, distance correctness.
    - `test_energy.py`: Dimensional correctness, climbing energy, positive energy values, wind heading wrap-around.
    - `test_fuzzy.py`: FIS bounds $[1.0, 2.0]$, rule firing, monotonicity and sensitivity for tailwind/crosswind/headwind.
    - `test_ga.py`: Permutation preservation in crossover and mutation (no duplicates, no missing cells).
    - `test_aco.py`: Pheromone dimensions, valid tour generation, positive probabilities.
    - `test_hybrid.py`: Hybrid seeding, end-to-end fitness evaluation with FIS.

16. **Documentation Files**:
    - `README.md`: Overview, quick start, directory structure, reproduction instructions.
    - `README_STEPS.md`: 25 comprehensive development steps with detailed mathematical formulations.
    - `README_BUGS.md`: Troubleshooting guide with symptom, cause, detection, and fixes across 8 bug categories.
    - `README_RESEARCH_PAPER.md`: Complete research paper draft structure (Title, Abstract, IEEE references, 9-slide presentation plan, 24 viva questions & answers).
    - `README_FLOW.md`: ASCII flowcharts and module dependency diagrams.

## Verification Plan

### Automated Tests
- Run `pytest -v tests/` to verify all 7 unit test suites pass 100%.
- Run `python main.py --quick` to verify end-to-end execution, CSV generation, figure generation, and validation checks.
- Run `python main.py --full` (or configured full mode) to verify full experiments and outputs.

### Manual Verification
- Verify generated CSV files in `results/` are properly populated and contain valid numerical data.
- Verify generated images in `figures/` are clean, readable, properly labeled with titles and legends.
- Verify terminal summary outputs proper formatted metrics.
