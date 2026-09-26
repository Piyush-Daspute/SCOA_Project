# Energy-Aware Wind-Adaptive 3D Coverage Path Planning for Agricultural UAVs

[![Python 3.11](https://img.shields.io/badge/python-3.11-blue.svg)](https://www.python.org/downloads/)
[![Tests](https://img.shields.io/badge/tests-passing-brightgreen.svg)](https://docs.pytest.org/)
[![Optimization](https://img.shields.io/badge/method-Hybrid%20ACO--GA%20%2B%20Mamdani%20FIS-orange.svg)]()
[![License](https://img.shields.io/badge/license-Academic%20Use-lightgrey.svg)]()

> A complete, reproducible research simulation framework for 3D agricultural UAV coverage path planning combining **Hybrid Ant Colony Optimization - Genetic Algorithm (ACO-GA)** metaheuristics with **Mamdani Fuzzy Inference System (FIS)** environmental cost adaptation under controlled wind and elevation regimes.

Developed as a B.Tech Final-Year Capstone Research Project for the **Soft Computing and Optimization Algorithms (SCOA)** course.

---

## Key Highlights & Research Contributions

1. **3D Agricultural Terrain Modeling**: Deterministic mathematical terrain surfaces (Flat $T_1$, Rolling $T_2$, Complex $T_3$) with constant terrain-following altitude clearance ($h_{clearance} = 20\text{ m}$).
2. **Obstacle-Free Guarantee**: Circular 3D obstacle cylinders with safety buffers ($d_{safe} = 3\text{ m}$). All evaluated paths achieve strictly **0 obstacle violations** and **100% field coverage**.
3. **Mamdani Fuzzy Inference System**: Pure NumPy implementation of a 7-rule Mamdani FIS modeling wind speed ($0-12\text{ m/s}$) and relative flight heading ($0^\circ-180^\circ$) to evaluate dynamic aerodynamic energy multipliers ($\mu_{wind} \in [1.0, 2.0]$).
4. **Dimensionally Sound Energy Formulation**: Rigorous physical formulation where flight energy equals propulsion power multiplied by flight duration ($E = P \cdot t = [W] \cdot [s] = [J]$).
5. **Hybrid ACO-GA with Active Environmental Adaptation**: ACO establishes promising global route topology and seeds the GA population; GA refines permutations via Order Crossover (OX-1) and 2-opt repair; the Mamdani FIS actively guides fitness selection during optimization.
6. **Reproducible Experimental Suite**: Includes automated benchmarks comparing Lawnmower, GA, ACO, ACO-GA, and ACO-GA-FIS, parametric wind sweeps, terrain complexity evaluations, and non-parametric statistical tests (Friedman & Wilcoxon signed-rank).

---

## Project Structure

```text
SCOA/
├── README.md                      # Primary project overview and quickstart
├── README_STEPS.md                # 25 detailed mathematical development steps
├── README_BUGS.md                 # Troubleshooting guide across 8 bug categories
├── README_RESEARCH_PAPER.md       # Research paper draft, 9-slide plan & 24 viva Q&As
├── README_FLOW.md                 # ASCII flowcharts and module dependency diagrams
│
├── requirements.txt               # Minimal scientific dependencies (numpy, scipy, pandas, matplotlib)
├── config.py                      # Central configuration dataclasses
├── main.py                        # CLI entrypoint (--quick and --full flags)
│
├── environment.py                 # 3D terrain elevation and obstacle geometry
├── coverage.py                    # Field decomposition, distance, turns, and 2-opt repair
├── energy.py                      # Dimensionally consistent energy model & wind heading
├── fitness.py                     # Normalized multi-objective fitness evaluator
├── fuzzy_system.py                # Pure NumPy Mamdani Fuzzy Inference System
│
├── baseline.py                    # Deterministic Boustrophedon Cellular Decomposition
├── ga.py                          # Permutation Genetic Algorithm (OX, 2-opt repair)
├── aco.py                         # Ant Colony Optimization (pheromone & heuristic matrix)
├── hybrid_aco_ga.py               # Hybrid ACO-GA optimizer (with & without FIS)
│
├── experiments.py                 # Automated research experimentation harness
├── statistics_analysis.py         # Descriptive stats, Friedman & Wilcoxon tests
├── visualization.py               # Publication-grade 2D & 3D figure generation
│
├── tests/                         # Comprehensive unit test suite (24 automated tests)
│   ├── test_environment.py
│   ├── test_coverage.py
│   ├── test_energy.py
│   ├── test_fuzzy.py
│   ├── test_ga.py
│   ├── test_aco.py
│   └── test_hybrid.py
│
├── results/                       # Generated CSV result datasets
│   ├── main_comparison.csv
│   ├── wind_speed_results.csv
│   ├── wind_direction_results.csv
│   ├── terrain_results.csv
│   ├── repeated_trials.csv
│   └── summary_statistics.csv
│
└── figures/                       # Generated publication-quality figures (300 DPI)
    ├── terrain.png
    ├── coverage_grid.png
    ├── lawnmower_path.png
    ├── best_path_3d.png
    ├── energy_comparison.png
    ├── distance_comparison.png
    ├── coverage_comparison.png
    ├── turns_comparison.png
    ├── runtime_comparison.png
    ├── energy_vs_wind_speed.png
    ├── energy_vs_wind_direction.png
    └── convergence_comparison.png
```

---

## Quickstart & Usage

### 1. Prerequisites & Installation
Ensure Python 3.10+ is installed. Clone the repository and install requirements:
```bash
pip install -r requirements.txt
```

### 2. Run Automated Unit Tests
Verify all 24 mathematical, geometric, and algorithmic unit tests pass:
```bash
python -m pytest -v
```

### 3. Run Quick Smoke Test (~5 seconds)
Executes a lightweight run ($10 \times 10$ grid, 1 trial) verifying end-to-end execution, CSV exports, plot generation, and validation checks:
```bash
python main.py --quick
```

### 4. Run Full Research Experiment
Executes the full research study ($20 \times 20$ grid, 400 cells, repeated trials, sensitivity sweeps):
```bash
python main.py --full
```

---

## Research Hypotheses & Experimental Findings

| Hypothesis | Description | Experimental Finding | Status |
| :--- | :--- | :--- | :---: |
| **H1** | ACO-GA-FIS achieves lower estimated energy consumption than standalone GA, ACO, and standard ACO-GA while maintaining 100% coverage. | ACO-GA-FIS achieved ~537.5 kJ vs ~565.1 kJ (ACO-GA) and ~2221.2 kJ (GA) with 100% coverage. | **Supported** |
| **H2** | The benefit of fuzzy environmental adaptation becomes more relevant as wind speed increases. | At 0 m/s wind, savings are 0%; at 6 m/s savings reach ~4.8%; at 12 m/s savings persist against headwinds. | **Supported** |
| **H3** | Hybrid ACO-GA produces lower-cost coverage paths than standalone ACO or GA on complex terrain. | ACO-GA-FIS achieved significantly lower cost on Complex Terrain ($T_3$) than standalone GA. | **Supported** |

---

## Generated Visualizations

All figures are automatically generated in `figures/` at 300 DPI:
- **3D Terrain & Trajectories**: `terrain.png`, `coverage_grid.png`, `lawnmower_path.png`, `best_path_3d.png`
- **Performance Comparisons**: `energy_comparison.png`, `distance_comparison.png`, `coverage_comparison.png`, `turns_comparison.png`, `runtime_comparison.png`
- **Wind Sensitivity Curves**: `energy_vs_wind_speed.png`, `energy_vs_wind_direction.png`
- **Convergence**: `convergence_comparison.png`

---

## Documentation Links
- [Detailed 25-Step Development Guide (README_STEPS.md)](file:///c:/Users/Piyush%20Daspute/OneDrive/Desktop/Projects/SCOA/README_STEPS.md)
- [Bugs & Troubleshooting Manual (README_BUGS.md)](file:///c:/Users/Piyush%20Daspute/OneDrive/Desktop/Projects/SCOA/README_BUGS.md)
- [Research Paper Draft, Presentation & 24 Viva Q&As (README_RESEARCH_PAPER.md)](file:///c:/Users/Piyush%20Daspute/OneDrive/Desktop/Projects/SCOA/README_RESEARCH_PAPER.md)
- [System Architecture & Flowchart (README_FLOW.md)](file:///c:/Users/Piyush%20Daspute/OneDrive/Desktop/Projects/SCOA/README_FLOW.md)
