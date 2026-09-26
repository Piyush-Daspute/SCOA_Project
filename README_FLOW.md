# Project Flow and System Architecture

This document describes the execution flow, mathematical data pathways, and software module architecture of the **Energy-Aware Wind-Adaptive 3D Coverage Path Planning for Agricultural UAVs** framework.

---

## 1. High-Level Project Execution Flow

```text
                                  +-----------------------+
                                  |         START         |
                                  +-----------------------+
                                              |
                                              v
                                  +-----------------------+
                                  |   Configuration Setup |
                                  |      (config.py)      |
                                  +-----------------------+
                                              |
                                              v
                    +---------------------------------------------------+
                    |         Environment Initialization                |
                    |  - 3D Terrain Model (Flat / Rolling / Complex)    |
                    |  - 3D Circular Obstacles & Safety Buffers (d_safe)|
                    |  - Clearance Altitude: z_uav = z_terrain + h_clear|
                    +---------------------------------------------------+
                                              |
                                              v
                    +---------------------------------------------------+
                    |         Coverage Decomposition                    |
                    |  - 500m x 500m agricultural field grid            |
                    |  - Valid target cells vs obstacle-invalid cells   |
                    |  - Pairwise Euclidean distance matrix (3D)        |
                    |  - Obstacle collision matrix (segment checks)     |
                    +---------------------------------------------------+
                                              |
                                              v
                    +---------------------------------------------------+
                    |       Mamdani Fuzzy Inference System (FIS)        |
                    |  - Input 1: Wind speed (Low, Moderate, High)      |
                    |  - Input 2: Relative wind angle (Tail, Cross, Head|
                    |  - Min-implication, Max-aggregation, Centroid     |
                    |  - Output: Environmental energy multiplier [1, 2] |
                    +---------------------------------------------------+
                                              |
                                              v
                    +---------------------------------------------------+
                    |             Optimization Algorithms               |
                    |                                                   |
                    |  [1] Deterministic Lawnmower Baseline             |
                    |      - Cellular decomposition into row segments   |
                    |      - Obstacle-free inter-segment linking        |
                    |                                                   |
                    |  [2] Standalone Genetic Algorithm (GA)            |
                    |      - Permutation representation                 |
                    |      - Tournament selection & Order Crossover(OX) |
                    |      - Inversion/Swap mutation & Stage 6 Repair   |
                    |                                                   |
                    |  [3] Standalone Ant Colony Optimization (ACO)     |
                    |      - Pheromone matrix & heuristic visibility    |
                    |      - Obstacle-suppressed ant path construction  |
                    |      - Evaporation & elitist pheromone deposit    |
                    |                                                   |
                    |  [4] Hybrid ACO-GA                                |
                    |      - Phase 1: ACO global exploration            |
                    |      - Phase 2: Seed extraction                   |
                    |      - Phase 3: GA refinement (without FIS)       |
                    |                                                   |
                    |  [5] Proposed ACO-GA-FIS                          |
                    |      - Same hybrid structure                      |
                    |      - FIS actively modulates segment energy      |
                    |        inside fitness function during search      |
                    +---------------------------------------------------+
                                              |
                                              v
                    +---------------------------------------------------+
                    |         Automated Experimental Suite              |
                    |  - Exp 1: Main comparative evaluation             |
                    |  - Exp 2: Wind speed sensitivity (0, 6, 12 m/s)   |
                    |  - Exp 3: Wind direction sweep (0, 90, 180 deg)   |
                    |  - Exp 4: Terrain complexity (T1, T2, T3)         |
                    |  - Exp 5: Repeated trials & statistical tests     |
                    +---------------------------------------------------+
                                              |
                                              v
                    +---------------------------------------------------+
                    |        Automatic Result Sanity Validation         |
                    |  - Coverage >= 99%                                |
                    |  - Energy > 0, Distance > 0, Runtime >= 0         |
                    |  - No NaN / Inf values                            |
                    |  - Zero obstacle collisions                       |
                    |  - FIS factor strictly within [1.0, 2.0]          |
                    +---------------------------------------------------+
                                              |
                                              v
                    +---------------------------------------------------+
                    |          Visualization & Result Export            |
                    |  - CSV tables exported to results/                |
                    |  - 2D/3D figures exported to figures/             |
                    |  - Descriptive stats (Mean +/- SD)                |
                    |  - Non-parametric tests (Friedman / Wilcoxon)     |
                    |  - Terminal research summary report               |
                    +---------------------------------------------------+
                                              |
                                              v
                                  +-----------------------+
                                  |          END          |
                                  +-----------------------+
```

---

## 2. Module Dependency Architecture

```text
                            +-------------+
                            |  config.py  |
                            +------+------+
                                   |
                +------------------+------------------+
                |                                     |
                v                                     v
       +-----------------+                   +------------------+
       | environment.py  |                   | fuzzy_system.py  |
       +--------+--------+                   +--------+---------+
                |                                     |
                v                                     |
       +-----------------+                            |
       |   coverage.py   |                            |
       +--------+--------+                            |
                |                                     |
                +------------------+                  |
                |                  |                  |
                |                  v                  v
                |         +----------------------------------+
                |         |            energy.py             |
                |         +-----------------+----------------+
                |                           |
                +------------------+        |
                                   |        |
                                   v        v
                         +-----------------------------+
                         |         fitness.py          |
                         +--------------+--------------+
                                        |
        +-------------------------------+-------------------------------+
        |                               |                               |
        v                               v                               v
+---------------+               +---------------+               +---------------+
|  baseline.py  |               |     ga.py     |               |    aco.py     |
+---------------+               +-------+-------+               +-------+-------+
                                        |                               |
                                        +---------------+---------------+
                                                        |
                                                        v
                                            +-----------------------+
                                            |   hybrid_aco_ga.py    |
                                            +-----------+-----------+
                                                        |
                                                        v
+-----------------------+                   +-----------------------+
| visualization.py      |<------------------|    experiments.py     |
+-----------------------+                   +-----------+-----------+
                                                        |
+-----------------------+                               |
| statistics_analysis.py|<------------------------------+
+-----------------------+                               |
                                                        v
                                                +---------------+
                                                |    main.py    |
                                                +---------------+
```

---

## 3. Data Pathways and Transformations

1. **Cell Definition**:
   `FieldConfig` $\to$ `CoverageGrid._decompose()` $\to$ List of `Cell` objects containing `(x, y, terrain_z, uav_z, valid, obstacle)`.
2. **Precomputed Matrices**:
   - `distance_matrix`: Pairwise 3D Euclidean distances between valid cells.
   - `is_edge_blocked`: Precomputed boolean matrix flagging 3D segments that intersect circular obstacles.
3. **Fitness Evaluation Pipeline**:
   Permutation `path` $\to$ `CoverageGrid` (computes length, coverage rate, turn count, obstacle checks) $\to$ `EnergyModel` (computes horizontal flight energy with FIS multiplier, climbing energy, turning energy) $\to$ `FitnessEvaluator` (normalizes objectives and computes scalar cost $J(P)$).
4. **Genetic Algorithm Flow**:
   Initial valid population $\to$ Tournament selection $\to$ Order Crossover (OX) preserving permutations $\to$ Inversion/Swap mutation $\to$ Stage 6 2-opt collision repair $\to$ Elitism replacement.
5. **Ant Colony Optimization Flow**:
   Heuristic visibility matrix $\eta_{ij} = \frac{1}{d_{ij} \mu_{wind, ij} + \epsilon}$ (with obstacle edges suppressed to 0) $\to$ Probabilistic tour construction $\to$ Pheromone evaporation $\tau \leftarrow (1-\rho)\tau \to$ Elite deposit $\Delta\tau = \frac{Q}{J(P_k)} \to$ Unique elite paths extracted.
6. **Hybridization Bridge**:
   Unique top ACO paths $\to$ Injected into GA initial population $\to$ Genetic crossover and local 2-opt mutation further refine edge sequences.
