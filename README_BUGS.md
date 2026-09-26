# Engineering Bugs & Troubleshooting Guide (README_BUGS.md)

This document provides a systematic diagnostic and troubleshooting manual for common bugs, edge cases, and mathematical pitfalls encountered in the 3D UAV coverage path planning and fuzzy optimization pipeline.

---

## 1. Environment & Grid Discretization

### 1.1 Inconsistent Grid Dimensions and Cell Centers
* **Symptom**: Cells overlap or leave unmonitored gaps; cell centers do not align with the $[0, W] \times [0, L]$ field boundaries.
* **Cause**: Incorrect cell coordinate generation using integer cell indices directly without offset (e.g., $x = i \cdot \Delta x$ instead of $(i + 0.5) \cdot \Delta x$).
* **Detection**: Automated assertion in `tests/test_coverage.py`: verify that $x_{min} = 0.5 \Delta x$ and $x_{max} = W - 0.5 \Delta x$.
* **Fix**: Compute center coordinates explicitly as $x_c = (i + 0.5) \frac{W}{N_x}$ and $y_c = (j + 0.5) \frac{L}{N_y}$.

### 1.2 Obstacle Buffer Penetration
* **Symptom**: Path waypoints pass inside the safety envelope $r + d_{safe}$.
* **Cause**: Evaluating only the physical obstacle radius $r$ while ignoring the safety buffer $d_{safe}=3\text{ m}$.
* **Detection**: Collision test in `Environment.is_point_obstructed(x, y)`: verifies $\text{dist}((x, y), (x_{obs}, y_{obs})) \le r + d_{safe}$.
* **Fix**: Use `effective_radius = radius + safety_buffer` universally across all spatial predicates.

---

## 2. Geometric & Trigonometric Computations

### 2.1 Angular Wrap-Around Errors for Relative Wind Direction
* **Symptom**: Calculating $\theta_{rel} = |\theta_{flight} - \theta_{wind}|$ produces $340^\circ$ instead of $20^\circ$ when $\theta_{flight}=350^\circ$ and $\theta_{wind}=10^\circ$.
* **Cause**: Direct arithmetic difference without modulo wrapping across the $360^\circ / 0^\circ$ circular branch cut.
* **Detection**: Unit test in `tests/test_energy.py::test_relative_wind_angle_wrap_around`.
* **Fix**: Use modular normalization:
  $$\Delta\theta = ((\theta_{flight} - \theta_{wind} + 180^\circ) \pmod{360^\circ}) - 180^\circ$$
  $$\theta_{rel} = |\Delta\theta| \in [0^\circ, 180^\circ]$$

### 2.2 Numerical Instability in Vector Dot Products
* **Symptom**: `numpy.arccos` returns `NaN` for turn angle calculations.
* **Cause**: Floating-point rounding error causes $\frac{\mathbf{v}_1 \cdot \mathbf{v}_2}{\|\mathbf{v}_1\| \|\mathbf{v}_2\|}$ to become $1.0000000002$ or $-1.0000000002$.
* **Detection**: Automated assertion verifying no `NaN` values in `calculate_turn_count`.
* **Fix**: Always clamp the cosine argument: `cos_theta = np.clip(dot / (norm1 * norm2), -1.0, 1.0)`.

---

## 3. Genetic Algorithm (GA) Permutation Safety

### 3.1 Gene Duplication and Omission from Standard Crossover
* **Symptom**: Offspring chromosomes visit certain cells multiple times while omitting other target cells.
* **Cause**: Applying standard single-point or arithmetic crossover meant for continuous optimization to discrete permutations.
* **Detection**: Invariant check in `tests/test_ga.py::test_order_crossover`: `assert len(set(child)) == len(valid_cells)`.
* **Fix**: Employ Order Crossover (OX-1): preserve an internal slice from Parent 1, then fill remaining open gene loci sequentially from Parent 2, skipping already copied alleles.

### 3.2 Obstacle Collisions in Randomly Initialized Permutations
* **Symptom**: Raw random permutations connect spatially distant cells, generating segments that intersect circular obstacle cylinders.
* **Cause**: Random shuffling ignores geometric proximity and obstacles.
* **Detection**: `validate_results_integrity()` flags `obstacle_violations > 0`.
* **Fix**: Implement **Stage 6: Repair Operator** via 2-opt edge untangling. For any segment $(p_i, p_{i+1})$ intersecting an obstacle, reverse an appropriate subsegment $p[i+1 : j+1]$ that substitutes obstacle-free alternative edges.

---

## 4. Ant Colony Optimization (ACO)

### 4.1 Numerical Underflow and Stagnation in Transition Probabilities
* **Symptom**: All heuristic weights evaluate to zero, causing ant selection to fail or loop indefinitely.
* **Cause**: When pheromone values evaporate excessively without lower bounds, $\tau_{ij}^\alpha \eta_{ij}^\beta \to 0$.
* **Detection**: Unit test `tests/test_aco.py::test_aco_matrices_dimensions` verifies $\tau_{min} \le \tau_{ij} \le \tau_{max}$.
* **Fix**: Clamp pheromone matrix entries to $[\tau_{min}, \tau_{max}] = [0.01, 50.0]$ after every evaporation and deposition cycle.

### 4.2 Ants Constructing Obstacle-Violating Edges
* **Symptom**: Unconstrained probabilistic choice allows ants to traverse across obstacle cylinders.
* **Cause**: Heuristic visibility $\eta_{ij}$ only considered Euclidean distance without penalizing blocked edges.
* **Fix**: Precompute `is_edge_blocked[i, j]` and set $\eta_{ij} = 0.0$ for all blocked edges. During ant path construction, filter candidate destinations to unblocked nodes.

---

## 5. Mamdani Fuzzy Inference System (FIS)

### 5.1 Centroid Defuzzification Division by Zero
* **Symptom**: FIS evaluation returns `NaN` when input falls into an unactivated membership region.
* **Cause**: $\sum \mu_C(z) = 0$ in the denominator of the centroid equation.
* **Detection**: Boundary condition test across input combinations in `tests/test_fuzzy.py::test_fis_output_bounds`.
* **Fix**: Safeguard denominator: if $\sum \mu_C(z) < 10^{-9}$, default gracefully to minimal multiplier $1.0$.

### 5.2 Membership Function Shoulder Clipping
* **Symptom**: Inputs at $0\text{ m/s}$ or $12\text{ m/s}$ evaluate to zero membership.
* **Cause**: Using triangular MFs without horizontal shoulders at universe extremes.
* **Fix**: Implement shoulder detection in `trimf()`: if $a == b$, clamp $\mu(x) = 1.0$ for all $x \le a$; if $b == c$, clamp $\mu(x) = 1.0$ for all $x \ge c$.

---

## 6. Energy Model Formulation

### 6.1 Dimensional Inconsistency in Climbing Power
* **Symptom**: Treating power directly as energy ($P \cdot \Delta z$), resulting in incorrect units $[W \cdot m]$.
* **Cause**: Failing to convert vertical displacement into flight duration.
* **Detection**: Checked in `tests/test_energy.py::test_energy_dimensional_consistency`.
* **Fix**: Maintain strict dimensional consistency:
  $$t_{climb} = \frac{\max(0, \Delta z_i)}{v_{climb}} \quad [s]$$
  $$E_{climb} = P_{climb} \cdot t_{climb} \quad [W \cdot s = J]$$

### 6.2 Negative Climbing Energy During Descent
* **Symptom**: Aircraft descent subtracts energy from total battery expenditure.
* **Cause**: Omitting the non-negative threshold operator $\max(0, \Delta z)$.
* **Fix**: Apply $\max(0.0, \Delta z_i)$ so descent segments consume base propulsion power without unphysical energy regeneration.

---

## 7. Experimental Rigor and Reproducibility

### 7.1 Random Seed Pollution Across Repeated Trials
* **Symptom**: Repeated trials yield identical numbers or stochastic results cannot be reproduced.
* **Cause**: Global pseudo-random state modified by third-party libraries without localized random instances.
* **Fix**: Seed every run explicitly using an isolated random number generator sequence: $seed_i = \text{SEED} + i$.

### 7.2 Disparate Evaluation Ground Truths
* **Symptom**: Algorithms optimized without FIS are evaluated using different criteria than FIS-guided methods.
* **Fix**: All algorithms are evaluated in the **exact same ground-truth physics environment** (true wind speed and direction with FIS cost) to ensure fair comparison.

---

## 8. Visualization & Matplotlib Dimensions

### 8.1 Headless Server Display Hangs
* **Symptom**: Script freezes when trying to display interactive GUI windows.
* **Cause**: Matplotlib attempting to connect to an X-server / Windows GDI handle.
* **Fix**: Explicitly specify non-interactive backend at script start: `matplotlib.use("Agg")`.

### 8.2 Dimension Mismatch in 3D Wireframe / Surface
* **Symptom**: `ValueError: shape mismatch: objects cannot be broadcast to a single shape`.
* **Cause**: Passing 1D coordinate vectors to `Axes3D.plot_surface` instead of 2D meshgrids.
* **Fix**: Generate full 2D coordinate matrices via `np.meshgrid(xs, ys)` before evaluating elevation $Z(X, Y)$.
