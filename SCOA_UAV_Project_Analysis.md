# SCOA UAV Project — Analysis, Research Positioning, Inferences, and Presentation Guide

## 1. Executive Verdict

The project is good enough for the SCOA FA-1 and is a legitimate research-style simulation project. However, it should not be presented exactly as the current README presents it.

| Question | Assessment |
|---|---|
| Does the project actually work? | Yes, with one testing/setup issue |
| Is it a reasonable SCOA research project? | Yes |
| Is the exact method highly novel for a research publication? | No — not strongly enough yet |
| Can it honestly be called a novel experimental integration/study? | Yes, with careful wording |
| Is real UAV data required for this assignment? | No |
| Should anything be changed before presentation? | Yes — several targeted corrections |

The main issue is not the lack of real data. The larger issue is that some claims in the README are stronger than what the experiments actually demonstrate.

## 2. What the Project Actually Does

The project solves:

> Given an agricultural field represented as a 3D terrain grid, find an order in which a UAV should visit all valid cells such that estimated flight energy is minimized while maintaining complete coverage and avoiding obstacles.

This is an optimization problem, not a conventional machine-learning training problem.

```text
Agricultural Field
        |
        v
500 m × 500 m simulated field
        |
        v
20 × 20 coverage grid
        |
        +------------------+
        |                  |
     Terrain            Obstacles
        |                  |
        +--------+---------+
                 |
                 v
          Valid coverage cells
                 |
        +--------+--------+
        |        |        |
       GA       ACO    Lawnmower
        |        |
        +----+---+
             |
             v
           ACO-GA
             |
             v
      Mamdani Fuzzy System
             |
      Wind speed + heading
             |
             v
      Wind energy multiplier
             |
             v
         Energy model
             |
             v
        Fitness function
             |
             v
      Best UAV trajectory
```

## 3. What Is the Dataset?

This is a simulation-based optimization study rather than a supervised-learning problem. Therefore, instead of a conventional labelled dataset, controlled environmental instances are generated consisting of terrain elevation, obstacles, coverage cells, and wind conditions.

The simulation environment contains:

- field dimensions
- terrain elevation
- UAV altitude
- obstacles
- valid/invalid coverage cells
- wind speed
- wind direction

Thus, the experimental data is a synthetic simulation dataset/environment.

This is appropriate for optimization because it allows controlled experiments such as:

```text
Same terrain
Same obstacles
Same UAV
Same algorithms

Wind = 0 m/s
Wind = 6 m/s
Wind = 12 m/s
```

## 4. How the Project Works

### Step 1 — Create the Field

The main environment uses:

\[
500m 	imes 500m
\]

with:

\[
20	imes20=400
\]

coverage cells.

Some cells are blocked by obstacles. The main environment has approximately 374 valid target cells.

Each cell has:

\[
(x,y,z)
\]

where \(x,y\) are horizontal coordinates and \(z\) is the terrain-following UAV altitude.

The UAV altitude is:

\[
z_{UAV}=z_{terrain}+20
\]

so the UAV maintains a 20 m clearance above terrain.

### Why 3D Matters

If terrain changes elevation:

\[
\Delta z 
eq 0
\]

and the geometric distance becomes:

\[
d_i=
\sqrt{
\Delta x_i^2+
\Delta y_i^2+
\Delta z_i^2
}
\]

Thus the problem is a 3D coverage path-planning problem.

## 5. GA and ACO

A route is represented as a permutation:

\[
P=[p_1,p_2,\ldots,p_N]
\]

Different permutations cover the same cells but can have very different distance, turns, altitude changes, wind exposure, and energy.

### Genetic Algorithm

GA uses:

- selection
- permutation-compatible crossover
- mutation
- repair
- repeated evolution

```text
Random routes
      |
Selection
      |
Crossover
      |
Mutation
      |
Repair
      |
Better routes
      |
Repeat
```

### Ant Colony Optimization

ACO uses pheromone:

\[
	au_{ij}
\]

and transition probability:

\[
P_{ij}^{k}
=
rac{
	au_{ij}^{lpha}\eta_{ij}^{eta}
}{
\sum_{l\in allowed}
	au_{il}^{lpha}\eta_{il}^{eta}
}
\]

where:

\[
\eta_{ij}=rac{1}{d_{ij}+\epsilon}
\]

Pheromone evaporation is:

\[
	au_{ij}\leftarrow(1-ho)	au_{ij}
\]

Good routes reinforce promising transitions.

## 6. Why ACO-GA?

ACO is good at exploring promising transitions, while GA can recombine and refine candidate permutations.

```text
ACO
 |
 v
Good candidate paths
 |
 v
Seed GA
 |
 v
Crossover
 |
 v
Mutation
 |
 v
Repair
 |
 v
Refined path
```

The intention is to combine ACO exploration with GA refinement.

## 7. Mamdani Fuzzy Inference System

Inputs:

1. Wind speed: \(0-12\,m/s\)
2. Relative wind angle: \(0^\circ-180^\circ\)

Linguistic categories include:

- Low / Moderate / High wind
- Tailwind / Crosswind / Headwind

Output:

\[
\mu_{wind}\in[1,2]
\]

Example rule:

> High wind + crosswind → Severe penalty.

The FIS uses Mamdani inference and centroid defuzzification.

### Why Fuzzy Logic?

The FIS encodes qualitative environmental knowledge into a continuous penalty without pretending that the simulation has a fully calibrated aerodynamic model.

The feedback loop is:

```text
Candidate path
      |
Calculate flight heading
      |
Relative wind angle
      |
Mamdani fuzzy inference
      |
Wind multiplier
      |
Energy
      |
Fitness
      |
ACO/GA selection
      |
New candidate paths
```

## 8. Energy Model

Conceptually:

\[
E =
E_{horizontal}
+
E_{climb}
+
E_{turn}
\]

with:

\[
E_{horizontal}
=
\sum_i
P_{base}\mu_{wind,i}t_i
\]

\[
t_i=rac{d_i}{v_i}
\]

and:

\[
E_{climb}
=
\sum_i
P_{climb}
rac{\max(0,\Delta z_i)}
{v_{climb}}
\]

\[
E_{turn}
=
P_{turn}N_{turn}
\]

Therefore:

\[
E_{total}
=
\sum_i
\left[
P_{base}\mu_{wind,i}t_i+
P_{climb}t_{climb,i}
ight]
+
P_{turn}N_{turn}
\]

The simulation uses:

\[
E=P	imes t
\]

so the units are:

\[
W	imes s=J
\]

## 9. Fitness Function

The objective is:

\[
J(P)=
w_E\hat E+
w_D\hat D+
w_T\hat T+
w_C L_C+
w_RR
\]

where:

- \(E\) = energy
- \(D\) = distance
- \(T\) = turns
- \(L_C\) = coverage loss
- \(R\) = obstacle/constraint penalty

Current weights:

\[
w_E=0.50,\quad
w_D=0.25,\quad
w_T=0.15,\quad
w_C=0.10
\]

The optimizer minimizes \(J(P)\).

A concise explanation is:

> The UAV route is optimized by minimizing a weighted energy-distance-turn objective while maintaining complete coverage and obstacle safety, with the environmental energy component adaptively modified by the Mamdani fuzzy system.

## 10. Actual Results

The current `main_comparison.csv` contains approximately:

| Algorithm | Energy (kJ) | Distance (m) | Turns | Coverage |
|---|---:|---:|---:|---:|
| Lawnmower | 377.0 | 10,709 | 63 | 100% |
| GA | 2,275.8 | 73,932 | 340 | 100% |
| ACO | 587.5 | 18,122 | 306 | 100% |
| ACO-GA | 588.5 | 18,143 | 305 | 100% |
| ACO-GA-FIS | 577.7 | 17,820 | 278 | 100% |

These values should be regenerated once after final code corrections so all CSVs, plots, READMEs, and presentation figures use one final experiment run.

## 11. Inference 1 — Lawnmower

The regular contiguous rectangular field strongly favors a lawnmower/boustrophedon route:

```text
→ → → → → → →
              ↓
← ← ← ← ← ← ←
              ↓
→ → → → → → →
              ↓
← ← ← ← ← ← ←
```

A poor permutation can jump between distant cells and create huge distances.

Therefore the current result, approximately 377 kJ for lawnmower versus 578 kJ for ACO-GA-FIS, is understandable.

The correct inference is:

> For a simple contiguous rectangular field, a deterministic coverage heuristic can outperform general-purpose metaheuristics.

Do not claim the hybrid algorithm is universally superior.

## 12. Inference 2 — GA

With approximately 374 cells, the theoretical permutation space is:

\[
374!
\]

which is enormous.

The current full configuration uses a relatively small search budget:

- population = 20
- generations = 20

Therefore GA can remain trapped in inefficient routes.

This explains the approximately 73.9 km route distance and 2276 kJ energy.

Defensible wording:

> Under the present search budget and permutation representation, GA showed poor convergence on the high-dimensional ordering problem.

Do not generalize this into "GA is inferior."

## 13. Inference 3 — ACO

ACO directly learns promising transitions:

\[
iightarrow j
\]

through pheromone reinforcement.

The observed result is approximately:

\[
GApprox2276kJ
\]

versus:

\[
ACOpprox588kJ
\]

A defensible interpretation is:

> Under the present experimental configuration, ACO's pheromone-guided route construction was more effective than the current GA search budget for discovering efficient cell-transition sequences.

## 14. Inference 4 — ACO-GA

The results are approximately:

\[
ACO=587.55kJ
\]

\[
ACO-GA=588.50kJ
\]

This is almost identical.

This suggests:

> The ACO stage is dominating solution quality, while the current GA refinement stage contributes limited additional optimization under the tested parameterization.

This is a useful result because combining two algorithms does not automatically guarantee improvement.

## 15. Inference 5 — ACO-GA-FIS

The current values are approximately:

\[
ACO-GA=588.50kJ
\]

\[
ACO-GA-FIS=577.70kJ
\]

Reduction:

\[
rac{588.50-577.70}{588.50}	imes100
pprox1.83\%
\]

The FIS version also has lower distance and fewer turns while maintaining 100% coverage.

The appropriate conclusion is:

> The fuzzy environmental adaptation produced a measurable but modest reduction in estimated energy under the tested 6 m/s wind scenario.

Do not describe 1.83% as revolutionary.

## 16. Why FIS Produces the Improvement

If Route A has many crosswind/headwind segments and Route B has more favorable headings, FIS can increase the environmental multiplier for Route A:

```text
Unfavorable wind
      |
      v
Higher fuzzy multiplier
      |
      v
Higher estimated energy
      |
      v
Worse fitness
```

The optimizer can therefore favor paths with less unfavorable wind exposure.

## 17. Wind-Speed Results

| Wind speed | ACO-GA | ACO-GA-FIS | Relative change |
|---:|---:|---:|---:|
| 0 m/s | 441.87 | 441.87 | 0% |
| 6 m/s | 588.50 | 577.70 | ~1.83% |
| 12 m/s | 715.84 | 703.66 | ~1.70% |

At 0 m/s there is effectively no wind penalty, so:

\[
\mu_{wind}pprox1
\]

and FIS provides essentially no additional benefit.

At 6 and 12 m/s, wind becomes relevant and the fuzzy adaptation changes route evaluation.

Useful inference:

> The value of environmental adaptation emerges when environmental conditions become non-negligible.

## 18. Wind-Direction Results

| Wind direction | ACO-GA | ACO-GA-FIS | Change |
|---:|---:|---:|---:|
| 0° | 591.98 | 589.24 | ~0.46% lower |
| 90° | 588.50 | 577.70 | ~1.83% lower |
| 180° | 591.86 | 596.00 | ~0.70% higher |

The 180° result is important.

It shows:

> The fuzzy adaptation is not universally beneficial under the current rule base and optimizer configuration.

This should be reported rather than hidden.

## 19. Why Can FIS Make the 180° Case Worse?

FIS modifies the optimization landscape.

A route that is geometrically short may be penalized because of an unfavorable wind heading:

```text
Without FIS:

A ---------------------- B
```

The optimizer may instead select:

```text
With FIS:

A ------+
        |
        +---------------- B
```

The second route may have a better wind penalty but travel farther.

If the fuzzy rules and objective weights do not balance this correctly, total energy can increase:

\[
596>591.9
\]

This can be a limitation of the fuzzy rule base or its interaction with route geometry, rather than automatically being a coding error.

## 20. Technical Issue — Energy Decomposition

One important implementation issue should be corrected.

The implementation defines:

\[
d_i=
\sqrt{\Delta x^2+\Delta y^2+\Delta z^2}
\]

and uses that distance in horizontal/base energy while separately adding climb energy.

This can partially count vertical movement twice.

A cleaner formulation is:

\[
d_{xy,i}
=
\sqrt{\Delta x_i^2+\Delta y_i^2}
\]

for horizontal propulsion:

\[
E_{horizontal}
=
P_{base}\mu_{wind}
rac{d_{xy,i}}{v}
\]

and:

\[
E_{climb}
=
P_{climb}
rac{\max(0,\Delta z_i)}
{v_{climb}}
\]

The 3D distance can still be reported separately as a geometric metric.

## 21. Technical Issue — Wind-Direction Convention

The project currently treats:

- 0° = tailwind
- 90° = crosswind
- 180° = headwind

This is acceptable if wind direction is defined as the direction toward which the wind travels.

Explicitly define:

> Wind vector direction is defined as the direction toward which the wind travels.

For example:

```text
0°   → wind toward +X
90°  → wind toward +Y
180° → wind toward -X
```

This removes ambiguity.

## 22. Testing Issue

The project has **24 passing tests** when executed with the correct project path configuration.

However, running `pytest` from an incorrect import context can produce errors for modules such as:

```text
config
fuzzy_system
```

The project should be configured so that:

```bash
pytest
```

works directly from the project root.

Avoid claiming absolute "bug-free." A better statement is:

> The implementation has been tested through unit tests, smoke tests, and end-to-end execution, with observed issues corrected and remaining limitations documented.

## 23. Stale README Results

The README contains values from an earlier experiment that do not exactly match the current CSV results.

For example, the README reports values around 537.5 kJ for ACO-GA-FIS, while the current comparison CSV reports approximately 577.7 kJ.

This must be corrected.

The final research pipeline should be:

```text
Final code
    |
    v
One final experiment run
    |
    +--> CSV results
    +--> Figures
    +--> Statistical analysis
    +--> README
    +--> Research paper
    +--> Presentation
```

All reported values must come from the same final experiment run.

## 24. Is the Project Actually Novel?

The project should not claim that ACO, GA, fuzzy logic, wind-aware planning, or 3D UAV coverage are individually novel.

The safer position is:

### Integration + controlled experimental analysis

The study integrates:

1. synthetic 3D terrain
2. obstacle-constrained coverage
3. ACO
4. GA refinement
5. Mamdani environmental adaptation
6. wind-relative heading
7. energy-aware fitness
8. controlled wind experiments
9. terrain experiments
10. FIS ablation
11. repeated stochastic evaluation

The contribution can be described as:

> A controlled simulation study investigating whether fuzzy environmental adaptation improves a hybrid ACO-GA coverage planner under varying wind and 3D terrain conditions.

This is legitimate research positioning for a college SCOA article, but it should not be presented as a fundamentally new optimization algorithm.

## 25. Recommended Novelty Position

| Type of novelty | Assessment |
|---|---|
| Novel algorithmic theory | Low |
| Novel application formulation | Moderate |
| Novel experimental integration | Moderate |
| Suitable research novelty for a college SCOA project | Yes |
| Strong publication-level novelty | Not yet established |

## 26. Recommended Research Question

> **How does incorporating Mamdani fuzzy environmental adaptation into a hybrid ACO-GA coverage planner affect the estimated energy consumption of agricultural UAVs under varying wind conditions and 3D terrain?**

## 27. Recommended Hypotheses

### H1

> Incorporating wind-aware fuzzy adaptation will reduce estimated energy consumption relative to an otherwise identical ACO-GA planner under non-zero wind conditions.

### H2

> The benefit of fuzzy adaptation will vary with wind speed and relative wind direction.

### H3

> Terrain complexity will increase climb-related energy consumption.

These hypotheses are testable and do not assume that every experiment must produce a positive result.

## 28. Recommended Changes Before Submission

### Must Fix

1. Correct the energy decomposition.
2. Define the wind-direction convention explicitly.
3. Fix the pytest/import setup.
4. Regenerate all results.
5. Replace stale README values.

### Strongly Recommended

6. Increase stochastic trials from 3 to at least 10.
7. Prefer 20 trials if computationally feasible.
8. Report mean ± standard deviation.
9. Include the negative 180° wind-direction result.
10. Explicitly label the environment as synthetic.
11. Avoid overclaiming novelty.
12. Avoid statistical significance claims with only 3 trials.

## 29. Statistical Reporting

With only:

\[
n=3
\]

the statistical evidence is weak.

Do not say:

> The improvement is statistically significant.

Instead say:

> Across three stochastic trials, ACO-GA-FIS showed lower mean estimated energy than ACO-GA.

With 10–20 trials, statistical tests such as Wilcoxon signed-rank for paired ACO-GA versus ACO-GA-FIS comparisons become more useful. A Friedman test can be considered for repeated comparisons across multiple algorithms when the experimental design supports it.

# 30. Presentation Plan

## Slide 1 — Title

**Energy-Aware Wind-Adaptive 3D Coverage Path Planning for Agricultural UAVs**

Include:

- Name
- PCCOE
- CSE (AI&ML)
- SCOA
- Academic year

## Slide 2 — Problem

Show:

```text
Agricultural field

Terrain
Obstacles
Wind
Limited battery
100% coverage requirement
```

Say:

> A UAV has to cover the entire field, but simply minimizing geometric distance does not necessarily minimize energy because terrain and wind affect the flight.

## Slide 3 — Research Gap

| Existing approach | Limitation |
|---|---|
| Fixed lawnmower | Does not adapt to environmental cost |
| GA | Large permutation search space |
| ACO | May require additional refinement |
| Energy-aware CPP | May use simplified environmental assumptions |
| Proposed study | Tests ACO-GA with fuzzy environmental adaptation |

Do not say nobody has studied the individual techniques.

## Slide 4 — Architecture

```text
          TERRAIN
             |
             v
       3D GRID + OBSTACLES
             |
             v
       VALID COVERAGE CELLS
             |
     +-------+--------+
     |                |
    ACO              GA
     |                ^
     +---- ACO SEEDS--+
             |
             v
            PATH
             |
             v
     WIND SPEED + HEADING
             |
             v
        MAMDANI FIS
             |
             v
      ENERGY MULTIPLIER
             |
             v
      ENERGY + DISTANCE
      + TURNS + COVERAGE
             |
             v
          FITNESS
             |
             v
        OPTIMIZATION
```

## Slide 5 — Mathematics

### 3D distance

\[
d_i=\sqrt{\Delta x^2+\Delta y^2+\Delta z^2}
\]

### Relative wind angle

\[
	heta_{rel}
=
\left|
((	heta_f-	heta_w+180)mod360)-180
ight|
\]

### Energy

\[
E=
\sum
\left(
P_{base}\mu_{wind}t_i
+
P_{climb}t_{climb,i}
ight)
+
P_{turn}N_{turn}
\]

### Objective

\[
J=
0.5\hat E+
0.25\hat D+
0.15\hat T+
0.1L_C+
R
\]

## Slide 6 — FIS

```text
Wind Speed
LOW / MEDIUM / HIGH

        +

Relative Direction
TAIL / CROSS / HEAD

        |
        v

Mamdani Rules

        |
        v

MIN-MAX inference

        |
        v

Centroid defuzzification

        |
        v

μwind = 1.0 – 2.0
```

Example:

> High wind + crosswind → Severe penalty.

## Slide 7 — Experimental Setup

| Parameter | Value |
|---|---|
| Field | 500 × 500 m |
| Grid | 20 × 20 |
| Terrain | Flat/Rolling/Complex |
| Obstacles | 6 |
| UAV clearance | 20 m |
| Wind | 0/6/12 m/s |
| Wind directions | 0/90/180° |
| Algorithms | 5 |
| Trials | Preferably 10+ |

Explicitly state:

> Simulation-based synthetic environment.

## Slide 8 — Results

Key findings:

1. Lawnmower performs extremely well on a regular contiguous field.
2. ACO substantially outperforms the current GA search configuration.
3. ACO-GA provides little additional improvement over ACO under the current parameterization.
4. ACO-GA-FIS provides modest energy improvement under several wind scenarios.

## Slide 9 — Conclusion

A strong conclusion is:

> The experiments demonstrate that environmental adaptation can influence UAV coverage optimization, but its benefit is dependent on wind conditions and route structure. The ACO-GA-FIS framework achieved lower estimated energy than ACO-GA in several tested wind scenarios while maintaining complete coverage. However, the deterministic lawnmower baseline remained highly competitive on the regular synthetic field, demonstrating the importance of matching optimization complexity to environmental structure.

# 31. Viva Questions and Answers

### "Is this real UAV energy?"

> No. It is a simulation-level energy model. The purpose is to compare the relative behavior of planning strategies, not to predict the exact battery consumption of a physical UAV.

### "Why don't you have a real dataset?"

> This is an optimization and simulation problem rather than a supervised-learning problem. We therefore generated controlled synthetic terrain, obstacle, and wind scenarios. This allows individual environmental variables to be changed independently.

### "Then what is your dataset?"

> The experimental dataset consists of generated field instances containing terrain elevation, valid coverage cells, obstacle geometry, and wind conditions.

### "Is this actually novel?"

> We are not claiming that ACO, GA, fuzzy logic, wind-aware planning, or 3D UAV coverage are individually novel. The contribution of our study is the controlled integration and experimental evaluation of ACO-GA optimization with a Mamdani fuzzy environmental cost inside the energy objective for 3D agricultural coverage. We also perform ablation and wind-sensitivity experiments to study when that adaptation actually helps.

# 32. What Not to Say

Do not say:

> Our algorithm proved to be the best.

The data does not support that because the lawnmower baseline has lower energy in the current regular field.

Do not say:

> GA is always worse than ACO.

The experiment only demonstrates the result under the current configuration.

Do not say:

> FIS always improves energy.

The 180° wind-direction experiment produces a worse ACO-GA-FIS result.

Do not say:

> This is the first ACO-GA-FIS UAV planner.

That requires a much more exhaustive literature review.

# 33. Strongest Overall Research Interpretation

The strongest interpretation is:

> **Optimization performance depends strongly on environmental structure. A simple deterministic lawnmower planner is highly competitive on a regular contiguous field, ACO substantially outperforms the current GA search configuration, ACO-GA adds little under the present search budget, and fuzzy environmental adaptation provides modest but condition-dependent gains under wind.**

This is a much more credible research story.

# 34. Optional Major Improvement

If time permits, the most valuable improvement would be a more realistic environment rather than another optimization algorithm.

For example:

```text
Current:
Synthetic terrain
Synthetic wind
Synthetic obstacles

        ↓

Improved:
Real DEM
+
Real field boundary
+
Publicly available wind scenario
```

Even one publicly available DEM could make the experimental setup more convincing.

However, this is optional for the SCOA submission.

# 35. Final Recommendation

Do not abandon the project.

Do not replace the topic.

### Fix

1. Energy decomposition.
2. Wind convention.
3. Pytest setup.
4. Stale README results.

### Strengthen

5. Increase trials to 10–20.
6. Rerun all experiments.
7. Report mean ± SD.
8. Include the negative 180° result.
9. Clearly label the environment as synthetic.
10. Tone down novelty claims.

### Final project positioning

> **A simulation-based experimental study of wind-aware energy optimization for 3D agricultural UAV coverage using hybrid ACO-GA with Mamdani fuzzy adaptation.**

This is much more defensible than calling it a revolutionary new algorithm.

# 36. Final Bottom Line

The project is not invalid because the data is synthetic.

It contains:

- a real optimization problem
- mathematical formulation
- multiple optimization algorithms
- fuzzy inference
- energy modelling
- controlled experiments
- ablation
- performance metrics
- reproducible simulation
- research documentation

The three most important corrections before submission are:

1. **Fix the energy-model decomposition.**
2. **Make all README/results consistent with one final experiment run.**
3. **Tone down novelty and statistical claims.**

The most important research finding from the current results is:

> **The effectiveness of an optimization method depends strongly on environmental structure: a simple deterministic lawnmower planner is excellent on a regular field, ACO substantially improves over the current GA search configuration, ACO-GA adds little under the present budget, and fuzzy environmental adaptation provides modest but condition-dependent gains under wind.**

The fact that the 180° case becomes slightly worse with FIS should be reported rather than hidden. It provides evidence that the fuzzy adaptation is condition-dependent and gives you a concrete limitation and future-research direction.
