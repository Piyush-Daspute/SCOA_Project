# Step-by-Step Development & Mathematical Reference (README_STEPS.md)

This document provides a comprehensive, mathematically explicit record of the 25 development steps undertaken to construct, validate, and package the 3D Agricultural UAV Coverage Path Planning research framework.

---

## Step 1: Project Objective
The primary objective of this project is to develop a simulation-based optimization framework for energy-aware, wind-adaptive, and terrain-following 3D coverage path planning (CPP) for an agricultural unmanned aerial vehicle (UAV). The system evaluates whether integrating a Mamdani Fuzzy Inference System (FIS) for dynamic environmental cost adaptation into a hybrid Ant Colony Optimization - Genetic Algorithm (ACO-GA) framework reduces overall UAV energy consumption under controlled synthetic wind and elevation regimes.

---

## Step 2: Research Question
> **Can a hybrid ACO-GA optimizer augmented with a Mamdani Fuzzy Inference System reduce estimated UAV energy consumption under varying wind conditions while maintaining high 3D agricultural field coverage?**

---

## Step 3: Research Gap
While standalone metaheuristics (such as GA and ACO) and classic boustrophedon patterns have been extensively investigated for 2D coverage, agricultural applications require 3D terrain awareness, obstacle avoidance, and explicit consideration of environmental wind vectors. Existing literature frequently treats wind as a post-optimization evaluation metric rather than embedding fuzzy environmental cost adaptation directly inside the optimization loop. This study provides an experimental evaluation of the integrated effect of hybrid ACO-GA optimization and Mamdani fuzzy environmental adaptation under controlled conditions.

---

## Step 4: Environment Generation
The agricultural environment is modeled as a bounded 3D workspace defined over a $500\text{ m} \times 500\text{ m}$ field:
$$\mathcal{W} = \{(x, y, z) \mid 0 \le x \le 500,\, 0 \le y \le 500,\, z_{terrain}(x, y) \le z \le z_{max}\}$$
The environment is managed by `Environment` in `environment.py`.

---

## Step 5: Terrain Generation
Three distinct terrain scenarios are supported:
1. **Flat Terrain ($T_1$)**:
   $$Z_{T1}(x, y) = 0$$
2. **Rolling Terrain ($T_2$)**:
   $$Z_{T2}(x, y) = 10 \sin\left(\frac{x}{60}\right) + 7 \cos\left(\frac{y}{70}\right) + 15 \exp\left(-\frac{(x-250)^2 + (y-250)^2}{2 \times 100^2}\right)$$
3. **Complex Terrain ($T_3$)**:
   $$Z_{T3}(x, y) = 8 \sin\left(\frac{x}{40}\right) + 6 \cos\left(\frac{y}{50}\right) + 12 G_1(x, y) + 10 G_2(x, y)$$
   where $G_1(x, y) = \exp\left(-\frac{(x-150)^2 + (y-350)^2}{2 \times 70^2}\right)$ and $G_2(x, y) = \exp\left(-\frac{(x-350)^2 + (y-150)^2}{2 \times 60^2}\right)$.

The UAV maintains constant terrain-following clearance $h_{clearance} = 20.0\text{ m}$:
$$z_{UAV}(x, y) = Z(x, y) + h_{clearance}$$

---

## Step 6: Coverage Decomposition
The field is decomposed into a discrete 2D grid of $N_x \times N_y$ cells. For each cell $(i, j)$:
$$x_c = (i + 0.5) \frac{W}{N_x}, \quad y_c = (j + 0.5) \frac{L}{N_y}$$
Each cell contains: `cell_id`, coordinates $(x_c, y_c)$, terrain elevation $z_{terrain}$, UAV clearance altitude $z_{UAV}$, boolean validity flag `valid`, and obstacle status `obstacle`.

---

## Step 7: Obstacle Generation & Clearance
Circular obstacles are modeled as 3D cylinders with coordinates $(x_{obs}, y_{obs})$, radius $r_{obs}$, height $h_{obs}$, and safety buffer $d_{safe} = 3.0\text{ m}$.
A coverage cell is marked invalid if:
$$\sqrt{(x_c - x_{obs})^2 + (y_c - y_{obs})^2} \le r_{obs} + d_{safe}$$
Path segments connecting waypoints $(x_1, y_1, z_1)$ and $(x_2, y_2, z_2)$ are evaluated for cylinder intersection with precomputed bounding-box acceleration.

---

## Step 8: Wind Field Generation
Synthetic wind conditions are parameterized by speed $V_w \in \{0, 3, 6, 9, 12\}\text{ m/s}$ and meteorological direction $\theta_w \in [0^\circ, 360^\circ)$.

---

## Step 9: Relative Wind Angle Calculation
For each path segment between consecutive waypoints $\Delta x = x_{i+1} - x_i$ and $\Delta y = y_{i+1} - y_i$:
$$\theta_{flight} = (\operatorname{atan2}(\Delta y, \Delta x) \times \frac{180^\circ}{\pi}) \pmod{360^\circ}$$
$$\Delta\theta = ((\theta_{flight} - \theta_w + 180^\circ) \pmod{360^\circ}) - 180^\circ$$
$$\theta_{rel} = |\Delta\theta| \in [0^\circ, 180^\circ]$$
Convention: $\theta_{rel} = 0^\circ$ corresponds to a pure tailwind; $\theta_{rel} = 90^\circ$ corresponds to a crosswind; $\theta_{rel} = 180^\circ$ corresponds to a direct headwind.

---

## Step 10: Mamdani Fuzzy Inference System (FIS)
Implemented in `fuzzy_system.py`:
- **Input 1: Wind Speed ($0-12\text{ m/s}$)**:
  - Low: $[0, 0, 4]$ (left shoulder)
  - Moderate: $[2, 6, 10]$ (triangular)
  - High: $[8, 12, 12]$ (right shoulder)
- **Input 2: Relative Wind Direction ($0^\circ-180^\circ$)**:
  - Tailwind: $[0, 0, 60]$ (left shoulder)
  - Crosswind: $[30, 90, 150]$ (triangular)
  - Headwind: $[120, 180, 180]$ (right shoulder)
- **Output: Environmental Energy Multiplier $\mu_{wind} \in [1.0, 2.0]$**:
  - Minimal: $[1.0, 1.0, 1.3]$
  - Moderate: $[1.2, 1.5, 1.8]$
  - Severe: $[1.6, 2.0, 2.0]$

**Mamdani Rule Base**:
1. If Wind Speed is Low $\to$ Multiplier is Minimal
2. If Wind Speed is Moderate and Direction is Tailwind $\to$ Multiplier is Minimal
3. If Wind Speed is Moderate and Direction is Crosswind $\to$ Multiplier is Moderate
4. If Wind Speed is Moderate and Direction is Headwind $\to$ Multiplier is Moderate
5. If Wind Speed is High and Direction is Tailwind $\to$ Multiplier is Moderate
6. If Wind Speed is High and Direction is Crosswind $\to$ Multiplier is Severe
7. If Wind Speed is High and Direction is Headwind $\to$ Multiplier is Severe

**Defuzzification**: Centroid integration over discretized universe $z \in [1.0, 2.0]$:
$$z^* = \frac{\int z \mu_C(z)\,dz}{\int \mu_C(z)\,dz}$$

---

## Step 11: Dimensionally Consistent Energy Model
Energy expenditure is modeled as power multiplied by flight duration ($E = P \cdot t$):
1. **Horizontal Base Energy**:
   $$E_{horizontal, i} = P_{base} \cdot \mu_{wind, i} \cdot \frac{d_{3D, i}}{v_{cruise}}$$
2. **Climbing Energy**:
   $$E_{climb, i} = P_{climb} \cdot \frac{\max(0, \Delta z_i)}{v_{climb}}$$
3. **Turn Energy**:
   $$E_{turn} = P_{turn} \cdot N_{turns}$$
4. **Total Path Energy**:
   $$E(P) = \sum_{i=1}^{N-1} \left[ P_{base} \mu_{wind, i} \frac{d_{3D, i}}{v_{cruise}} + P_{climb} \frac{\max(0, \Delta z_i)}{v_{climb}} \right] + P_{turn} N_{turns}$$

Standard assumptions: $P_{base} = 200\text{ W}$, $P_{climb} = 300\text{ W}$, $P_{turn} = 5\text{ J/turn}$, $v_{cruise} = 10\text{ m/s}$, $v_{climb} = 2\text{ m/s}$, $\theta_{turn} \ge 30^\circ$.

---

## Step 12: Fitness Function
The multi-objective normalized fitness cost $J(P)$ is minimized:
$$J(P) = w_E \hat{E} + w_D \hat{D} + w_T \hat{T} + w_C L_C + w_R R$$
where:
$$\hat{E} = \frac{E - E_{min}}{E_{max} - E_{min} + \epsilon}, \quad \hat{D} = \frac{D - D_{min}}{D_{max} - D_{min} + \epsilon}, \quad \hat{T} = \frac{T - T_{min}}{T_{max} - T_{min} + \epsilon}$$
$$L_C = 1 - \frac{CR}{100}, \quad R = N_{obstacle\_violations}$$
Default weights: $w_E = 0.50$, $w_D = 0.25$, $w_T = 0.15$, $w_C = 0.10$, $w_R = 10^5$.

---

## Step 13: Deterministic Lawnmower Baseline
Implemented via Boustrophedon Cellular Decomposition in `baseline.py`. Decomposes the field into contiguous valid horizontal row strips and connects segments greedily along obstacle-free transitions, achieving 100% target cell coverage and 0 obstacle violations.

---

## Step 14: Genetic Algorithm (GA)
Implemented in `ga.py`:
- Chromosome: Permutation of valid cell IDs.
- Selection: Tournament selection ($k=3$).
- Crossover: Order Crossover (OX-1) preserving permutation validity.
- Mutation: 2-opt subsegment inversion and pair swap ($p_m = 0.15$).
- Repair: Stage 6 2-opt collision untangling.
- Elitism: Retains top 2 elite chromosomes.

---

## Step 15: Ant Colony Optimization (ACO)
Implemented in `aco.py`:
- Pheromone matrix $\tau_{ij}$ with bounds $[\tau_{min}, \tau_{max}] = [0.01, 50.0]$.
- Heuristic visibility: $\eta_{ij} = \frac{1}{d_{ij} + \epsilon}$.
- Transition probability:
  $$P_{ij}^k = \frac{\tau_{ij}^\alpha \eta_{ij}^\beta}{\sum_{l \in allowed_k} \tau_{il}^\alpha \eta_{il}^\beta}$$
- Evaporation: $\tau_{ij} \leftarrow (1 - \rho)\tau_{ij}$ ($\rho = 0.1$).
- Deposit: $\Delta\tau_{ij} = \frac{Q}{J(P_k)}$ for elite ant tours.

---

## Step 16: Hybrid ACO-GA Optimization
Implemented in `hybrid_aco_ga.py`:
1. Phase 1: ACO runs for $N_{aco}$ iterations to discover promising global topology.
2. Phase 2: Top unique ACO tours are extracted and injected as seeds into the initial GA population.
3. Phase 3: GA executes genetic operators (OX, mutation, repair, elitism) to locally refine and untangle the paths.

---

## Step 17: Proposed ACO-GA-FIS Method
In `HybridACOGAOptimizer` with `use_fis=True`:
- In ACO: heuristic visibility actively incorporates wind multiplier $\mu_{wind, ij}$:
  $$\eta_{ij}^{wind} = \frac{1}{d_{ij} \mu_{wind, ij} + \epsilon}$$
- In GA: fitness evaluation calculates segment-by-segment wind multipliers via FIS during crossover, mutation, and selection.

---

## Step 18: Automated Experimental Suite
Implemented in `experiments.py`:
- **Experiment 1**: Main comparative benchmark across all 5 algorithms.
- **Experiment 2**: Wind speed sensitivity sweep ($0, 6, 12\text{ m/s}$).
- **Experiment 3**: Wind direction sensitivity sweep ($0^\circ, 90^\circ, 180^\circ$).
- **Experiment 4**: Terrain complexity comparison ($T_1, T_2, T_3$).
- **Experiment 5**: Repeated randomized trials across deterministic seeds.

---

## Step 19: Statistical Analysis
Implemented in `statistics_analysis.py`:
- Descriptive metrics: Mean $\pm$ SD, Min, Max across trials.
- Inferential tests: Non-parametric Friedman test across algorithms, and Wilcoxon signed-rank test for paired comparison between ACO-GA and ACO-GA-FIS.

---

## Step 20: 2D/3D Visualization Engine
Implemented in `visualization.py`:
- `terrain.png`: 3D agricultural elevation mesh.
- `coverage_grid.png`: 2D field decomposition with obstacle cylinders and safety zones.
- `lawnmower_path.png`: 3D Lawnmower trajectory above terrain.
- `best_path_3d.png`: 3D ACO-GA-FIS trajectory with start/end markers.
- Comparative bar charts (Energy, Distance, Turns, Coverage, Runtime).
- Sensitivity curves and convergence progression graphs.

---

## Step 21: Automatic Result Validation
Before writing results to permanent storage, `validate_results_integrity()` verifies:
- Coverage rate $\ge 99.0\%$
- Energy $> 0$, Distance $> 0$, Runtime $\ge 0$
- No `NaN` or `Inf` values
- Obstacle violations $== 0$
- FIS output strictly in $[1.0, 2.0]$

---

## Step 22: How to Run the Project
```bash
# Fast smoke-test (10x10 grid, finishes in ~5 seconds)
python main.py --quick

# Full experimental study (20x20 grid, repeated trials)
python main.py --full

# Run all unit tests
python -m pytest -v
```

---

## Step 23: How to Reproduce Experiments
All experiments utilize a deterministic seed sequence:
$$seed_i = \text{SEED} + i \quad (\text{SEED} = 42)$$
Executing `python main.py --full` reproduces the exact experimental numbers stored in `results/` and figures in `figures/`.

---

## Step 24: How to Configure Field & Grid Size
In `config.py`, modify `FieldConfig`:
```python
grid_n_x = 20  # Change number of X cells
grid_n_y = 20  # Change number of Y cells
width = 500.0  # Change field width (meters)
length = 500.0 # Change field length (meters)
```

---

## Step 25: How to Configure Algorithm Parameters
In `config.py`:
- **GA**: modify `GAConfig(population_size=30, generations=30, crossover_prob=0.8, mutation_prob=0.15)`.
- **ACO**: modify `ACOConfig(num_ants=10, iterations=20, alpha=1.0, beta=2.0, evaporation_rate=0.1)`.
- **Wind**: modify `WindConfig(speed=6.0, direction=90.0)`.
- **Fitness Weights**: modify `FitnessConfig(w_energy=0.5, w_distance=0.25, w_turns=0.15, w_coverage=0.1)`.
