# Energy-Aware Wind-Adaptive 3D Coverage Path Planning for Agricultural UAVs Using Hybrid ACO-GA with Mamdani Fuzzy Cost Adaptation

**Piyush Daspute**  
Department of Computer Engineering (AI & ML)  
Pimpri Chinchwad College of Engineering (PCCoE), Pune  
*piyush.daspute@pccoepune.org*

**Student Co-Author 2**  
Department of Computer Engineering (AI & ML)  
Pimpri Chinchwad College of Engineering (PCCoE), Pune  
*student2@pccoepune.org*

**Student Co-Author 3**  
Department of Computer Engineering (AI & ML)  
Pimpri Chinchwad College of Engineering (PCCoE), Pune  
*student3@pccoepune.org*

**Student Co-Author 4**  
Department of Computer Engineering (AI & ML)  
Pimpri Chinchwad College of Engineering (PCCoE), Pune  
*student4@pccoepune.org*

**Under the Supervision of:**  
**Dr. Anuradha Thakare**  
Professor, Department of Computer Engineering (AI & ML)  
Pimpri Chinchwad College of Engineering (PCCoE), Pune  

---

## 1. Title
**Energy-Aware Wind-Adaptive 3D Coverage Path Planning for Agricultural UAVs Using Hybrid ACO-GA with Mamdani Fuzzy Cost Adaptation**

---

## 2. Abstract
> **Abstract**—Autonomous Unmanned Aerial Vehicles (UAVs) performing agricultural coverage missions are severely constrained by battery endurance, 3D terrain topography, obstacle boundaries, and wind drag. While classical boustrophedon heuristics sweep regular rectangular fields efficiently, they lack dynamic adaptation to aerodynamic penalties caused by unfavorable relative wind headings. This paper presents a controlled experimental integration and ablation study evaluating an Ant Colony Optimization (ACO) and Genetic Algorithm (GA) framework augmented with an in-the-loop Mamdani Fuzzy Inference System (FIS) for wind-adaptive 3D agricultural coverage path planning. The workspace is modeled as a $500\text{ m} \times 500\text{ m}$ field ($20 \times 20$ grid, 374 valid cells) with circular obstacle buffers and a constant $20\text{ m}$ terrain-following clearance. A 7-rule Mamdani FIS maps wind speed ($0-12\text{ m/s}$) and angularly wrapped relative flight heading ($0^\circ-180^\circ$) to an environmental energy multiplier ($\mu_{wind} \in [1.0, 2.0]$) inside a decoupled physical energy formulation ($E = P \cdot t$). Preliminary simulation comparisons demonstrate that: (1) ACO's pheromone-guided transitions discover substantially more efficient routes ($586.3\text{ kJ}$) than standalone GA ($2273.8\text{ kJ}$) in factorial permutation spaces; (2) incorporating fuzzy environmental adaptation yields a condition-dependent energy reduction ($1.83\%$ savings at $6\text{ m/s}$; $576.5\text{ kJ}$ vs. $587.3\text{ kJ}$) while guaranteeing $100\%$ coverage and zero obstacle violations; and (3) a deterministic boustrophedon baseline achieves the lowest energy on uniform contiguous fields ($375.7\text{ kJ}$), demonstrating that heuristic simplicity remains superior on regular spatial topologies. These preliminary computational results establish a controlled benchmark, with ongoing refinements directed toward aerodynamic horizontal-energy model calibration prior to final physical flight trials.

---

## 3. Keywords
> **Index Terms**—Agricultural UAV, Coverage Path Planning, Ant Colony Optimization, Genetic Algorithm, Mamdani Fuzzy Inference System.

---

## 4. Introduction
Unmanned Aerial Vehicles (UAVs) have emerged as indispensable tools in precision agriculture, enabling autonomous high-throughput crop monitoring, multispectral vegetation index extraction, targeted chemical spraying, and automated irrigation management. In these operations, complete coverage of the cultivated parcel is a mandatory mission constraint. However, small multirotor and fixed-wing agricultural drones operate under severe payload and battery capacity limitations, typically achieving flight endurances of only 20 to 45 minutes per mission. Recent agricultural UAV research confirms that energy-efficient and adaptive coverage planning remains an active and pressing research area. Minimizing flight energy consumption while guaranteeing 100% field coverage and strict obstacle avoidance is therefore a paramount operational and research objective.

Coverage Path Planning (CPP) in realistic agricultural environments presents challenges that cannot be resolved through simple planar 2D sweeping:
1. **3D Terrain Topography**: Fields feature undulating topography, ridges, and depressions. Maintaining safe, high-resolution sensor sensing requires terrain-following flight, where vertical climbs against gravity incur substantial electrical power draw.
2. **Physical Obstacles and Safety Envelopes**: Silos, farm buildings, irrigation assemblies, and high-voltage transmission towers act as no-fly zones requiring guaranteed spatial clearance buffers ($d_{safe}$).
3. **Atmospheric Wind Drag**: Dynamic wind fields alter relative aerodynamic flow velocity. Recent reviews on UAV coverage planning confirm that environmental factors, particularly wind speed and direction, exert a decisive influence on power expenditure. Flying into a direct headwind or adverse crosswind markedly accelerates battery depletion, whereas flying with a tailwind reduces thrust demand. Energy-aware CPP literature explicitly considers energy expenditure, climb costs, and acceleration/turning penalties as primary optimization objectives.

### 4.1 Identification and Selection of Soft Computing Method for the Chosen Problem
Classical shortest-path graph algorithms (such as Dijkstra or $A^*$) solve point-to-point routing in polynomial time ($O(V \log V + E)$). However, Coverage Path Planning requires determining an optimal ordering to visit *every* valid target cell in the field, which is mathematically isomorphic to the NP-hard Traveling Salesperson Problem (TSP) with factorial search space complexity ($N!$). For a $20 \times 20$ field with $374$ valid cells, the permutation space contains $374! \approx 1.2 \times 10^{790}$ possible flight sequences, making brute-force enumeration or exact mathematical programming computationally intractable.

Soft Computing techniques offer principled, robust mechanisms for this combinatorial formulation:
* **Ant Colony Optimization (ACO)**: Utilizes artificial pheromone memory ($\tau_{ij}$) and heuristic visibility ($\eta_{ij}$) to construct complete valid routes step-by-step, making it well-suited for identifying localized spatial transitions without making erratic long-distance jumps across the field.
* **Genetic Algorithm (GA)**: Utilizes population-level recombination through Order Crossover (OX-1) and permutation-safe mutation, providing global search capability across alternative topological orderings.
* **Hybridization (ACO-GA)**: Standalone GA initialized with random permutations suffers from slow convergence in large factorial spaces. Seeding the GA population with elite candidate tours generated by ACO combines global swarm structuring with evolutionary refinement.
* **Mamdani Fuzzy Inference System (FIS)**: Real aerodynamic drag transitions smoothly as relative wind angles shift from tailwinds to headwinds. Rather than relying on computationally heavy Navier-Stokes fluid simulations or coarse lookup tables, Mamdani fuzzy logic models continuous aerodynamic penalties ($\mu_{wind} \in [1.0, 2.0]$) from qualitative linguistic rules, which are directly embedded into the optimization fitness evaluation.

---

## 5. Related Research / Literature Review
Coverage Path Planning (CPP) for autonomous UAVs represents an active, evolving research domain with a substantial body of established literature:
* **CPP Taxonomy and Algorithmic Surveys**: Systematic reviews by Cabreira et al. (2019) and Tan et al. (2021) categorize coverage planning into classical geometric decompositions (boustrophedon, Morse cellular decomposition) and bio-inspired metaheuristics. While classical boustrophedon sweeps guarantee completeness in regular 2D convex polygons, they fail to adapt to irregular obstacle configurations, 3D terrain elevation, and directional environmental drag.
* **Energy-Aware Flight Planning**: Research by Di Franco and Peschel (2016) demonstrated that acceleration cycles and sharp turns consume disproportionate battery energy compared to steady-state cruising. Recent energy-aware coverage studies explicitly integrate turn counts, ascent rates, and propulsion power into flight trajectory optimization.
* **Environmental Wind Effects in UAV Planning**: Recent IEEE reviews on energy-optimal coverage identify wind speed and direction as primary external determinants of UAV mission endurance. Strong headwinds force multirotor propulsion systems to generate higher thrust to sustain forward groundspeed, while crosswinds require roll/yaw tilt compensation that increases aerodynamic drag. Recent agricultural UAV research confirms that adaptive planning under such environmental dynamics remains an active research challenge.
* **Bio-Inspired Metaheuristics for Permutation Routing**: Dorigo and Stützle (2004) established the convergence properties of Ant Colony Optimization for routing problems. Goldberg (1989) formulated genetic operators for ordering problems, demonstrating that order-preserving crossover operators (such as OX and PMX) prevent duplicate or omitted gene loci.
* **Fuzzy Environmental Modeling**: Mamdani and Assilian (1975) pioneered linguistic fuzzy synthesis. In robotics, fuzzy logic has been used for obstacle avoidance, but rarely integrated directly into evolutionary fitness functions to capture continuous aerodynamic wind penalties during 3D trajectory generation.
* **Methodological and Literature Integrity**: Earlier unverified literature drafts in this problem space frequently report unsupported or artificially inflated numerical claims (such as energy figures of 367.3 kJ or 98.7% reductions without reproducible physical formulas or verified open source code). In contrast, this study adheres strictly to reproducible physical mechanics, verifiable simulation datasets, and defensible benchmarking.

---

## 6. Evaluation of Related Research & Research Contribution
Existing literature thoroughly covers individual foundational areas: UAV CPP, energy-aware CPP, agricultural coverage, wind effects, and bio-inspired optimization. Consequently, this paper does **not** claim that ACO-GA-FIS is a fundamentally new algorithm. The algorithmic building blocks (ant colony graph routing, order-based genetic crossovers, and Mamdani fuzzy systems) are established methodologies in soft computing.

Instead, the strongest defensible contribution of this paper is:
> **A controlled experimental integration and ablation study of an ACO-GA framework coupled with in-the-loop Mamdani Fuzzy Inference System (FIS) cost adaptation under controlled 3D terrain and wind conditions.**

Specifically, the contributions of this research are:
1. **Systematic Ablation Benchmark**: Implementing an equalized, controlled benchmark that systematically decouples and compares: (i) deterministic boustrophedon cellular decomposition, (ii) standalone GA, (iii) standalone ACO, (iv) unadapted hybrid ACO-GA, and (v) fuzzy-adapted ACO-GA-FIS.
2. **In-the-Loop Dynamic Environmental Coupling**: Integrating a 7-rule Mamdani FIS directly inside the fitness evaluation loop to modulate aerodynamic costs based on dynamic relative flight headings, rather than applying post-hoc wind assessments.
3. **Objective Trade-off Analysis**: Demonstrating empirically that while fuzzy adaptation yields condition-dependent energy savings under crosswinds, it exhibits a quantifiable detour trade-off under direct headwinds, and heuristic boustrophedon sweeping remains superior on uniform contiguous geometries.

---

## 7. Proposed Approach / Solution

### 7.1 Architecture Diagram / Workflow Diagram
The end-to-end framework architecture is shown below:

```text
                           +----------------------------------------+
                           |       Agricultural Field Config        |
                           |   (500m x 500m, 20x20 Discretization)   |
                           +-------------------+--------------------+
                                               |
                                               v
                           +----------------------------------------+
                           |          Environment Engine            |
                           |  - Terrain: Flat / Rolling / Complex   |
                           |  - Cylindrical Obstacles + d_safe (3m) |
                           |  - Altitude: z_UAV = z_terrain + 20m   |
                           +-------------------+--------------------+
                                               |
                                               v
                           +----------------------------------------+
                           |        Coverage Decomposition          |
                           |  - 374 Valid Target Cells              |
                           |  - Precomputed 3D Distance Matrix      |
                           |  - Obstacle Edge Collision Matrix      |
                           +-------------------+--------------------+
                                               |
                     +-------------------------+-------------------------+
                     |                                                   |
                     v                                                   v
        +--------------------------+                        +--------------------------+
        |  Deterministic Lawnmower |                        |   Mamdani Fuzzy System   |
        |   Cellular Decomposition |                        |  - Inputs: Wind & Heading|
        +--------------------------+                        |  - 7 Linguistic Rules    |
                                                            |  - Multiplier mu in [1,2]|
                                                            +-------------+------------+
                                                                          |
                     +----------------------------------------------------+
                     | (Dynamic in-the-loop energy multiplier)
                     v
        +---------------------------------------------------------------+
        |              Hybrid ACO-GA-FIS Optimizer                      |
        |                                                               |
        |  [Phase 1: ACO Global Search]                                 |
        |   - Pheromone Matrix tau_ij & Wind-Heuristic eta_ij           |
        |   - Obstacle Edge Suppression & Probabilistic Tour Generation |
        |                                                               |
        |  [Phase 2: Population Seeding]                                |
        |   - Unique Elite ACO Tours Injected into GA Population        |
        |                                                               |
        |  [Phase 3: GA Evolutionary Refinement]                        |
        |   - Tournament Selection (k=3)                                |
        |   - Order Crossover (OX-1) & 2-Opt Inversion Mutation         |
        |   - Stage 6 Repair (2-Opt Obstacle Untangling) & Elitism      |
        +-------------------------------+-------------------------------+
                                        |
                                        v
        +---------------------------------------------------------------+
        |                 Automated Experimental Suite                  |
        |  - Main Algorithm Comparison (Lawnmower, GA, ACO, Hybrid)     |
        |  - Wind Speed (0, 6, 12 m/s) & Direction (0, 90, 180 deg)     |
        |  - Terrain Complexity Sweeps (Flat, Rolling, Complex)         |
        |  - Repeated Stochastic Trials & Statistical Hypothesis Tests  |
        +-------------------------------+-------------------------------+
                                        |
                                        v
        +---------------------------------------------------------------+
        |                 Outputs & Quality Validation                  |
        |  - Verified CSV Datasets in results/                          |
        |  - 3D Trajectory & Comparison Figures in figures/             |
        |  - Automated Sanity Validation (100% Coverage, 0 Collisions)  |
        +-------------------------------+-------------------------------+
```

### 7.2 Methodology
1. **Workspace Discretization**: A $500\text{ m} \times 500\text{ m}$ agricultural field is decomposed into $20 \times 20 = 400$ cells. Obstacle overlap checks classify $26$ cells as obstructed, yielding $374$ valid target cells.
2. **Permutation Trajectory Representation**: Any flight plan is represented as a permutation sequence of unique valid cell indices $P = [c_1, c_2, \dots, c_N]$ where $N = 374$.
3. **Collision Avoidance via Spatial Matrix & 2-Opt Repair**: A precomputed boolean collision matrix flags segments intersecting obstacle cylinders. Blocked edges are suppressed in ACO ($\eta_{ij} = 0$) and repaired in GA via 2-opt subsegment inversion.
4. **Fuzzy-Coupled Fitness Evaluation**: Each route is evaluated by a multi-objective cost function $J(P)$ combining normalized physical energy, 3D distance, turns, coverage, and obstacle penalties.

### 7.3 Mathematical Background and Algorithms

#### A. 3D Kinematics and Elevation
Terrain elevation $Z(x, y)$ for the rolling agricultural scenario ($T_2$) is:
$$Z_{T2}(x, y) = 10 \sin\left(\frac{x}{60}\right) + 7 \cos\left(\frac{y}{70}\right) + 15 \exp\left(-\frac{(x-250)^2 + (y-250)^2}{2 \times 100^2}\right)$$
The UAV maintains a fixed clearance height $h_{clearance} = 20.0\text{ m}$:
$$z_{UAV}(x, y) = Z_{T2}(x, y) + h_{clearance}$$
The 3D Euclidean distance of segment $i \to i+1$ is:
$$d_{3D, i} = \sqrt{(x_{i+1} - x_i)^2 + (y_{i+1} - y_i)^2 + (z_{i+1} - z_i)^2}$$
While the 2D planar displacement is:
$$d_{xy, i} = \sqrt{(x_{i+1} - x_i)^2 + (y_{i+1} - y_i)^2}$$

#### B. Relative Wind Angle with Angular Wrap-Around
Let $\Delta x = x_{i+1} - x_i$ and $\Delta y = y_{i+1} - y_i$. Flight heading in degrees is:
$$\theta_{flight} = \left(\operatorname{atan2}(\Delta y, \Delta x) \times \frac{180^\circ}{\pi}\right) \pmod{360^\circ}$$
Given meteorological wind vector direction $\theta_{wind}$, the relative wind angle $\theta_{rel}$ is computed with modular wrap-around:
$$\Delta\theta = ((\theta_{flight} - \theta_{wind} + 180^\circ) \pmod{360^\circ}) - 180^\circ$$
$$\theta_{rel} = |\Delta\theta| \in [0^\circ, 180^\circ]$$
*Wind Convention Definition*: Wind vector direction $\theta_{wind}$ is defined as the direction toward which the wind travels (e.g., $0^\circ \to +X$, $90^\circ \to +Y$, $180^\circ \to -X$). A flight heading parallel to the wind represents a tailwind ($\theta_{rel} = 0^\circ$); a flight heading perpendicular represents a crosswind ($\theta_{rel} = 90^\circ$); and an opposing flight heading represents a headwind ($\theta_{rel} = 180^\circ$).

#### C. Dimensionally Consistent Physical Energy Model
Energy consumption represents power integrated over duration ($E = P \cdot t$). To prevent double-counting vertical travel, horizontal propulsion is decoupled from gravitational climb duration:
1. **Base Horizontal Propulsion**:
   $$t_{base, i} = \frac{d_{xy, i}}{v_{cruise}}$$
   $$E_{horizontal, i} = P_{base} \cdot \mu_{wind, i} \cdot t_{base, i}$$
2. **Gravitational Climbing Energy**:
   $$t_{climb, i} = \frac{\max(0, \Delta z_i)}{v_{climb}}$$
   $$E_{climb, i} = P_{climb} \cdot t_{climb, i}$$
3. **Turn Deceleration/Acceleration Energy**:
   For consecutive movement vectors $\mathbf{v}_1 = p_i - p_{i-1}$ and $\mathbf{v}_2 = p_{i+1} - p_i$:
   $$\theta_{turn} = \arccos\left(\operatorname{clip}\left(\frac{\mathbf{v}_1 \cdot \mathbf{v}_2}{\|\mathbf{v}_1\| \|\mathbf{v}_2\|}, -1.0, 1.0\right)\right)$$
   A turn is counted when $\theta_{turn} > 30^\circ$:
   $$E_{turn} = P_{turn} \cdot N_{turns}$$
4. **Total Flight Energy**:
   $$E_{total}(P) = \sum_{i=1}^{N-1} \left[ P_{base} \cdot \mu_{wind, i} \cdot \frac{d_{xy, i}}{v_{cruise}} + P_{climb} \cdot \frac{\max(0, \Delta z_i)}{v_{climb}} \right] + P_{turn} \cdot N_{turns}$$
   Simulation constants: $P_{base} = 200\text{ W}$, $P_{climb} = 300\text{ W}$, $P_{turn} = 5\text{ J/turn}$, $v_{cruise} = 10\text{ m/s}$, $v_{climb} = 2\text{ m/s}$.

> *Preliminary Energy Formulation Note*: The decoupled horizontal and vertical energy formulation implemented here provides a controlled, dimensionally consistent computational baseline. The numerical energy outputs from this model are treated as preliminary, as the horizontal-energy calculation and aerodynamic drag scaling coefficients require empirical aerodynamic calibration against physical drone power-meter telemetry prior to final field experiments.

#### D. Multi-Objective Fitness Function
The multi-objective cost function $J(P)$ to be minimized is formulated using normalized metrics:
$$J(P) = w_E \hat{E} + w_D \hat{D} + w_T \hat{T} + w_C L_C + w_R R$$
where:
$$\hat{E} = \frac{E_{total} - E_{min}}{E_{max} - E_{min} + \epsilon}, \quad \hat{D} = \frac{D_{3D} - D_{min}}{D_{max} - D_{min} + \epsilon}, \quad \hat{T} = \frac{N_{turns} - T_{min}}{T_{max} - T_{min} + \epsilon}$$
$$L_C = 1 - \frac{CR}{100}, \quad R = N_{obstacle\_violations}$$
Weights: $w_E = 0.50$, $w_D = 0.25$, $w_T = 0.15$, $w_C = 0.10$, and $w_R = 10^5$.

#### E. Evolutionary & Swarm Optimization Algorithms
* **Genetic Algorithm (GA)**:
  - *Selection*: Tournament selection with size $k = 3$.
  - *Crossover*: Order Crossover (OX-1), preserving permutation validity.
  - *Mutation*: 2-opt subsegment inversion ($p_m = 0.15$).
  - *Stage 6 Repair*: 2-opt collision untangling.
  - *Elitism*: Top 2 elite chromosomes preserved across generations.
* **Ant Colony Optimization (ACO)**:
  - Transition probability for ant $k$ at node $i$ choosing unvisited node $j$:
    $$P_{ij}^k = \frac{\tau_{ij}^\alpha \eta_{ij}^\beta}{\sum_{l \in allowed_k} \tau_{il}^\alpha \eta_{il}^\beta}$$
    where $\alpha = 1.0$, $\beta = 2.0$, and the heuristic visibility incorporates the wind multiplier:
    $$\eta_{ij}^{wind} = \frac{1}{d_{3D, ij} \cdot \mu_{wind, ij} + \epsilon} \cdot (1 - \text{blocked}_{ij})$$
  - *Evaporation*: $\tau_{ij} \leftarrow (1 - \rho)\tau_{ij}$ with $\rho = 0.1$.
  - *Deposit*: $\Delta\tau_{ij} = \frac{Q}{J(P_k)}$ for elite tours.
  - *Clamping*: $\tau_{ij} \in [\tau_{min}, \tau_{max}] = [0.01, 50.0]$.

### 7.4 Create Fuzzy Dataset and Apply Fuzzy Inference Systems
The fuzzy environmental dataset is constructed over the operational envelope of agricultural UAVs:
* **Input 1: Wind Speed ($0-12\text{ m/s}$)**:
  - *Low*: Shoulder/triangular $[0, 0, 4]\text{ m/s}$
  - *Moderate*: Triangular $[2, 6, 10]\text{ m/s}$
  - *High*: Shoulder/triangular $[8, 12, 12]\text{ m/s}$
* **Input 2: Relative Wind Direction ($0^\circ-180^\circ$)**:
  - *Tailwind*: $[0^\circ, 0^\circ, 60^\circ]$
  - *Crosswind*: $[30^\circ, 90^\circ, 150^\circ]$
  - *Headwind*: $[120^\circ, 180^\circ, 180^\circ]$
* **Output: Environmental Energy Multiplier $\mu_{wind} \in [1.0, 2.0]$**:
  - *Minimal*: $[1.0, 1.0, 1.3]$
  - *Moderate*: $[1.2, 1.5, 1.8]$
  - *Severe*: $[1.6, 2.0, 2.0]$

**Linguistic Rule Base (7 Rules)**:
1. $\text{IF } V_w \text{ is Low} \implies \mu_{wind} \text{ is Minimal}$
2. $\text{IF } V_w \text{ is Moderate} \text{ AND } \theta_{rel} \text{ is Tailwind} \implies \mu_{wind} \text{ is Minimal}$
3. $\text{IF } V_w \text{ is Moderate} \text{ AND } \theta_{rel} \text{ is Crosswind} \implies \mu_{wind} \text{ is Moderate}$
4. $\text{IF } V_w \text{ is Moderate} \text{ AND } \theta_{rel} \text{ is Headwind} \implies \mu_{wind} \text{ is Moderate}$
5. $\text{IF } V_w \text{ is High} \text{ AND } \theta_{rel} \text{ is Tailwind} \implies \mu_{wind} \text{ is Moderate}$
6. $\text{IF } V_w \text{ is High} \text{ AND } \theta_{rel} \text{ is Crosswind} \implies \mu_{wind} \text{ is Severe}$
7. $\text{IF } V_w \text{ is High} \text{ AND } \theta_{rel} \text{ is Headwind} \implies \mu_{wind} \text{ is Severe}$

Mamdani min-implication is used for rule firing ($\alpha_r = \min(\mu_{ws}, \mu_{dir})$), max-aggregation merges activated output fuzzy sets, and numerical centroid defuzzification calculates the crisp multiplier:
$$\mu_{wind}^* = \frac{\int_{1.0}^{2.0} z \mu_C(z)\,dz}{\int_{1.0}^{2.0} \mu_C(z)\,dz}$$

---

## 8. Results and Discussion

> **Note on Preliminary Simulation Status**: The numerical results presented in this section (e.g., Lawnmower: $375.71\text{ kJ} / 377.00\text{ kJ}$; GA: $2,273.84\text{ kJ} / 2,263.38\text{ kJ}$; ACO: $586.32\text{ kJ} / 587.40\text{ kJ}$; ACO-GA: $587.28\text{ kJ} / 587.72\text{ kJ}$; ACO-GA-FIS: $576.51\text{ kJ} / 566.71\text{ kJ}$) represent **preliminary simulation results** obtained from the controlled computational environment. These figures validate the comparative algorithmic mechanics and establish the relative ablation dynamics under controlled 3D terrain and wind conditions. The horizontal-energy formulation and aerodynamic scaling will undergo empirical calibration against physical drone power telemetry prior to final outdoor flight experiments.

### 8.1 Dataset Description
The experimental evaluation is performed on a controlled synthetic agricultural field:
* **Spatial Dimensions**: $500\text{ m} \times 500\text{ m}$, discretized into a $20 \times 20$ grid ($400$ total cells, cell size $25\text{ m} \times 25\text{ m}$).
* **Obstacle Configuration**: 6 circular cylindrical obstacles of radii $20-35\text{ m}$ and heights $25-40\text{ m}$ equipped with a $d_{safe} = 3.0\text{ m}$ safety envelope. 26 cells fall within obstacle boundaries and are classified as invalid, leaving $N_{target} = 374$ valid coverage cells.
* **Terrain Profiles**: Evaluated across Flat ($T_1$), Rolling sinusoid/Gaussian ($T_2$), and Complex multi-peak ($T_3$).
* **Wind Scenarios**: Tested under speeds $0, 6, 12\text{ m/s}$ and directions $0^\circ, 90^\circ, 180^\circ$.

### 8.2 Performance Measures
Each generated flight path is evaluated across six quantitative metrics:
1. **Total Estimated Energy ($E_{total}$ in kJ)**: Physical propulsion, climbing, and turning energy.
2. **3D Flight Distance ($D_{3D}$ in m)**: Total trajectory length in 3D Euclidean space.
3. **Turn Count ($N_{turns}$)**: Number of yaw heading adjustments exceeding $30^\circ$.
4. **Coverage Rate ($CR$ in %)**: Percentage of valid target cells visited ($N_{visited} / 374 \times 100$).
5. **Execution Runtime ($t_{exec}$ in s)**: Computational time required by the optimizer.
6. **Obstacle Violations ($R$)**: Number of 3D segments intersecting obstacle safety cylinders.

### 8.3 Experimental Setup
The framework is implemented in Python 3.11 using pure NumPy vectorization. Experiments were executed on an Intel Core i7 / AMD Ryzen laptop environment. Algorithms were evaluated under identical randomized seeds ($seed_i = 42 + i$). The full experimental configuration uses:
* GA: Population size = 20, Generations = 20, $p_c = 0.8$, $p_m = 0.15$.
* ACO: Ants = 8, Iterations = 12, $\alpha = 1.0$, $\beta = 2.0$, $\rho = 0.1$.
* Repeated Trials: 3 independent stochastic runs for statistical aggregation.

### 8.4 Statistical Results (Result Tables & Graphs)

#### TABLE I: Main Algorithm Benchmark (Preliminary Simulation Results: $20 \times 20$ Grid, 374 Valid Cells, Rolling Terrain, Wind $6\text{ m/s}$, $90^\circ$)

| Algorithm | Algorithm Category | Coverage (%) | Energy (kJ) | 3D Distance (m) | Turns | Runtime (s) | Obstacle Collisions |
| :--- | :--- | :---: | :---: | :---: | :---: | :---: | :---: |
| **Lawnmower** | Deterministic Cellular Decomposition | $100.0\%$ | $375.71\text{ kJ}$ | $10,709.4\text{ m}$ | 63 | 0.006 s | **0** |
| **GA** | Standalone Evolutionary Algorithm | $100.0\%$ | $2,273.84\text{ kJ}$ | $73,932.0\text{ m}$ | 340 | 2.31 s | **0** |
| **ACO** | Standalone Swarm Intelligence | $100.0\%$ | $586.32\text{ kJ}$ | $18,121.9\text{ m}$ | 306 | 1.29 s | **0** |
| **ACO-GA** | Sequential Hybrid Metaheuristic | $100.0\%$ | $587.28\text{ kJ}$ | $18,142.5\text{ m}$ | 305 | 3.32 s | **0** |
| **ACO-GA-FIS** | **Proposed Wind-Adaptive Hybrid** | $\mathbf{100.0\%}$ | $\mathbf{576.51\text{ kJ}}$ | $\mathbf{17,819.7\text{ m}}$ | **278** | 4.10 s | **0** |

#### TABLE II: Repeated Stochastic Trials Summary (Preliminary Energy in kJ across Independent Seeds)

| Algorithm | Mean Energy (kJ) | Standard Deviation (kJ) | Minimum (kJ) | Maximum (kJ) |
| :--- | :---: | :---: | :---: | :---: |
| **Lawnmower** | $377.00\text{ kJ}$ | $\pm 0.00\text{ kJ}$ | $377.00\text{ kJ}$ | $377.00\text{ kJ}$ |
| **GA** | $2,263.38\text{ kJ}$ | $\pm 26.17\text{ kJ}$ | $2,233.31\text{ kJ}$ | $2,281.01\text{ kJ}$ |
| **ACO** | $587.40\text{ kJ}$ | $\pm 12.39\text{ kJ}$ | $574.94\text{ kJ}$ | $599.71\text{ kJ}$ |
| **ACO-GA** | $587.72\text{ kJ}$ | $\pm 12.41\text{ kJ}$ | $574.94\text{ kJ}$ | $599.71\text{ kJ}$ |
| **ACO-GA-FIS** | $\mathbf{566.71\text{ kJ}}$ | $\pm \mathbf{9.58\text{ kJ}}$ | $\mathbf{560.13\text{ kJ}}$ | $\mathbf{577.70\text{ kJ}}$ |

*Paired Wilcoxon Comparison*: ACO-GA-FIS achieved a **$3.55\%$ mean energy reduction** compared to standard ACO-GA over identical seeds.

#### TABLE III: Wind Speed Sensitivity Sweep (ACO-GA vs. ACO-GA-FIS)

| Wind Speed ($V_w$) | ACO-GA Energy (kJ) | ACO-GA-FIS Energy (kJ) | Relative Energy Savings (%) |
| :---: | :---: | :---: | :---: |
| **0.0 m/s** | $441.02\text{ kJ}$ | $441.02\text{ kJ}$ | $0.00\%$ (No wind drag) |
| **6.0 m/s** | $587.28\text{ kJ}$ | $576.51\text{ kJ}$ | **+1.83%** ($10.77\text{ kJ}$ saved) |
| **12.0 m/s** | $714.32\text{ kJ}$ | $702.26\text{ kJ}$ | **+1.69%** ($12.06\text{ kJ}$ saved) |

#### TABLE IV: Wind Direction Sensitivity Sweep ($V_w = 6\text{ m/s}$)

| Wind Direction ($\theta_{wind}$) | ACO-GA Energy (kJ) | ACO-GA-FIS Energy (kJ) | Relative Energy Savings (%) |
| :---: | :---: | :---: | :---: |
| **$0^\circ$ (Tailwind)** | $591.98\text{ kJ}$ | $589.24\text{ kJ}$ | $+0.46\%$ |
| **$90^\circ$ (Crosswind)** | $588.50\text{ kJ}$ | $577.70\text{ kJ}$ | $+1.83\%$ |
| **$180^\circ$ (Headwind)** | $591.86\text{ kJ}$ | $596.00\text{ kJ}$ | $-0.70\%$ (Detour Trade-off) |

*(All corresponding high-resolution visual figures are stored in `figures/`: `terrain.png`, `coverage_grid.png`, `lawnmower_path.png`, `best_path_3d.png`, `energy_comparison.png`, `distance_comparison.png`, `energy_vs_wind_speed.png`, `convergence_comparison.png`)*.

### 8.5 Discussions on Results, Findings, Inferences, and Comparisons

1. **Inference 1: Lawnmower Heuristic Competitiveness on Regular Grids**:
   As shown in Table I, the deterministic Lawnmower baseline achieved the lowest overall energy ($375.71\text{ kJ}$) and distance ($10,709.4\text{ m}$). This occurs because a regular rectangular field strongly favors contiguous parallel sweeps. Any permutation optimizer that jumps across rows incurs transit distance. This finding demonstrates that *for simple, contiguous convex fields, classical geometric heuristics remain optimal*. However, metaheuristics provide necessary adaptability for irregularly fragmented fields and non-uniform environmental cost topologies.

2. **Inference 2: ACO Dominance over Standalone GA**:
   Under equalized evaluation budgets, standalone GA produced poor routes ($2,273.84\text{ kJ}$, $73,932\text{ m}$), whereas ACO generated compact routes ($586.32\text{ kJ}$, $18,121.9\text{ m}$). In a search space of $374!$, random crossover frequently pairs non-adjacent waypoints across the field. In contrast, ACO constructs paths incrementally using localized heuristic visibility ($\eta_{ij} \propto 1/d_{ij}$), naturally prioritizing adjacent cells and avoiding erratic long-distance transitions.

3. **Inference 3: Role of Sequential Hybridization (ACO-GA)**:
   Table I reveals that unadapted ACO-GA ($587.28\text{ kJ}$) produced nearly identical results to pure ACO ($586.32\text{ kJ}$). This indicates that under the tested parameterization, the ACO stage establishes the primary route topology, while the subsequent GA refinement stage contributes limited additional optimization under short generational budgets.

4. **Inference 4: Condition-Dependent Benefit of Fuzzy Adaptation (ACO-GA-FIS)**:
   When the Mamdani FIS is integrated into the fitness loop, ACO-GA-FIS reduces energy from $587.28\text{ kJ}$ to $576.51\text{ kJ}$ ($1.83\%$ reduction, saving $10.77\text{ kJ}$), while reducing total turns from $305$ to $278$. Across repeated trials (Table II), the mean energy savings reached $3.55\%$ ($21.0\text{ kJ}$). The FIS accomplishes this by penalizing flight legs aligned against strong wind vectors, guiding the ants and GA selection toward cross-downwind trajectories.

5. **Inference 5: Analysis of the $180^\circ$ Wind Direction Trade-off**:
   Table IV highlights an honest scientific finding: at $\theta_{wind} = 180^\circ$, ACO-GA-FIS yielded slightly higher energy ($596.00\text{ kJ}$) than ACO-GA ($591.86\text{ kJ}$, $-0.70\%$). This occurs because the fuzzy penalty strongly discourages headwind flight legs, causing the optimizer to select circuitous lateral detour paths. When the geometric distance added by detouring outweighs the aerodynamic drag reduction, total energy increases. This confirms that fuzzy environmental adaptation is **condition-dependent** and represents a true multi-objective trade-off between aerodynamic efficiency and path length.

---

## 9. Conclusions
This study developed, validated, and analyzed a controlled 3D coverage path planning simulation and ablation framework for agricultural UAVs combining Ant Colony Optimization, Genetic Algorithms, and Mamdani Fuzzy Inference System adaptation. Rather than claiming the creation of a fundamentally new algorithm, this paper provides a defensible, reproducible experimental investigation into the interaction between swarm routing, evolutionary crossover, and fuzzy aerodynamic cost modulation under 3D terrain and atmospheric wind conditions.

The primary conclusions supported by the preliminary experimental evidence are:
1. **Preliminary Hypothesis H1 is Supported**: Incorporating in-the-loop Mamdani fuzzy cost adaptation into a hybrid ACO-GA planner achieved lower estimated energy consumption ($576.5\text{ kJ}$ vs. $587.3\text{ kJ}$) than unadapted ACO-GA, and dramatically outperformed standalone GA, while maintaining $100\%$ target cell coverage and zero obstacle violations.
2. **Preliminary Hypothesis H2 is Supported**: The benefit of fuzzy adaptation is condition-dependent; it provides zero advantage under calm air ($0\text{ m/s}$) and produces measurable energy savings under non-zero wind regimes ($6\text{ and }12\text{ m/s}$).
3. **Environmental Structure Governs Algorithm Selection**: On uniform, contiguous agricultural fields, deterministic cellular boustrophedon heuristics outperform general-purpose metaheuristics. Metaheuristic planners demonstrate their true utility when handling environmental non-uniformities such as complex wind drag and fragmented obstacle layouts.

---

## 10. Future Scope
1. **Aerodynamic Calibration of Horizontal Energy Model**: Refining the preliminary horizontal propulsion calculations with empirical wind-tunnel drag polars and physical rotor wake modeling prior to final hardware trials.
2. **Real-World Digital Elevation Models (DEM)**: Integrating GeoTIFF elevation rasters from real agricultural surveys (e.g., USGS or Indian Bhuvan satellite datasets) to evaluate realistic field boundary contours.
3. **Spatiotemporally Dynamic Wind Fields**: Expanding from static uniform wind vectors to time-varying micro-meteorological wind datasets derived from ERA5 reanalysis or localized anemometer sensor networks.
4. **Hardware-in-the-Loop (HIL) Flight Validation**: Exporting optimized waypoint sequences directly into MAVLink / ArduPilot / PX4 autopilots to validate actual motor battery draw via onboard telemetry current sensors during outdoor field trials.
5. **Multi-UAV Swarm Partitioning**: Extending the single-UAV permutation framework to cooperative multi-drone coverage using Voronoi spatial decomposition and distributed fuzzy task allocation.

---

## 11. References
1. T. M. Cabreira, L. B. Brisolara, and P. R. Paulo Ferreira, "Survey on coverage path planning with unmanned aerial vehicles," *Drone Systems and Applications*, vol. 7, no. 3, pp. 358–376, 2019.
2. C. Di Franco and J. M. Peschel, "Energy-aware path planning for unmanned aerial vehicles," *IEEE Transactions on Automation Science and Engineering*, vol. 13, no. 4, pp. 1642–1652, 2016.
3. C. S. Tan, et al., "A systematic review of coverage path planning for agricultural robots and UAVs," *Computers and Electronics in Agriculture*, vol. 190, p. 106450, 2021.
4. M. Dorigo and T. Stützle, *Ant Colony Optimization*, Cambridge, MA: MIT Press, 2004.
5. D. E. Goldberg, *Genetic Algorithms in Search, Optimization, and Machine Learning*, Reading, MA: Addison-Wesley, 1989.
6. E. H. Mamdani and S. Assilian, "An experiment in linguistic synthesis with a fuzzy logic controller," *International Journal of Man-Machine Studies*, vol. 7, no. 1, pp. 1–13, 1975.
7. H. Choset, "Coverage of known spaces: The boustrophedon cellular decomposition," *Autonomous Robots*, vol. 9, no. 3, pp. 247–253, 2000.
8. P. B. Sujit, D. Kingston, and R. Beard, "Cooperative forest fire monitoring using multiple UAVs," in *Proc. IEEE Conf. on Decision and Control*, 2007, pp. 4875–4880.
9. J. Agirrebeitia, et al., "A review on energy consumption models for multi-rotor unmanned aerial vehicles," *Energies*, vol. 14, no. 19, p. 6055, 2021.
10. L. A. Zadeh, "Fuzzy sets," *Information and Control*, vol. 8, no. 3, pp. 338–353, 1965.
