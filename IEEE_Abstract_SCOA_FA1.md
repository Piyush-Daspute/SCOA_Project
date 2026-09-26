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

### Course & Institutional Information
* **Course Name:** Soft Computing and Optimization Algorithms (SCOA)
* **Assessment:** Formative Assessment-1 (FA-1), Academic Year 2026–27
* **Program:** Final Year B.Tech, CSE (AI & ML)
* **Institute:** Pimpri Chinchwad Education Trust’s Pimpri Chinchwad College of Engineering (An Autonomous Institute Affiliated to SPPU, Pune)
* **Course Outcomes Addressed:**
  * **CO1:** Identify Soft Computing techniques and their roles in problem solving.
  * **CO2:** Create fuzzy datasets and apply fuzzy inference systems to soft computing applications.
  * **CO3:** Analyze different strategies for evolutionary computing.

---

## 2. Abstract (IEEE Format)

> **Abstract**—Autonomous Unmanned Aerial Vehicles (UAVs) performing agricultural coverage missions face severe battery endurance limitations under 3D terrain topography, obstacle boundaries, and adverse atmospheric winds. While classical boustrophedon heuristics sweep regular rectangular areas efficiently, they lack dynamic adaptation to aerodynamic drag penalties caused by unfavorable relative wind headings. This paper presents a controlled experimental integration and ablation study evaluating an Ant Colony Optimization (ACO) and Genetic Algorithm (GA) framework augmented with an in-the-loop Mamdani Fuzzy Inference System (FIS) for wind-adaptive 3D agricultural coverage path planning. The workspace is modeled as a $500\text{ m} \times 500\text{ m}$ field ($20 \times 20$ grid, 374 valid cells) with circular obstacle buffers and a constant $20\text{ m}$ terrain-following clearance. A 7-rule Mamdani FIS maps wind speed ($0-12\text{ m/s}$) and angularly wrapped relative flight heading ($0^\circ-180^\circ$) to an environmental energy multiplier ($\mu_{wind} \in [1.0, 2.0]$) evaluated inside a decoupled physical energy formulation ($E = P \cdot t$). Preliminary simulation comparisons demonstrate that: (1) ACO's pheromone-guided transitions discover substantially more efficient routes ($586.3\text{ kJ}$) than standalone GA ($2273.8\text{ kJ}$) in factorial permutation spaces; (2) incorporating fuzzy environmental adaptation yields a condition-dependent energy reduction ($1.83\%$ savings at $6\text{ m/s}$; $576.5\text{ kJ}$ vs. $587.3\text{ kJ}$) while guaranteeing $100\%$ coverage and zero obstacle violations; and (3) a deterministic boustrophedon baseline achieves the lowest energy on uniform contiguous fields ($375.7\text{ kJ}$), demonstrating that heuristic simplicity remains superior on regular spatial topologies. These preliminary computational results establish a controlled benchmark, with ongoing refinements directed toward aerodynamic horizontal-energy calibration prior to final physical flight trials.

---

## 3. Keywords / Index Terms (IEEE Format)

> **Index Terms**—Agricultural UAV, Coverage Path Planning, Ant Colony Optimization, Genetic Algorithm, Mamdani Fuzzy Inference System.

---

## 4. Course Outcomes (CO) Mapping Table

| Section in Article | Specific Soft Computing Element | Course Outcome | Justification & Implementation Reference |
| :--- | :--- | :---: | :--- |
| **Problem Formulation** | 3D UAV Agricultural Coverage under Aerodynamic Drag | **CO1** | Identifies energy optimization as a multi-objective constraint problem where soft computing is required. |
| **Fuzzy Modeling** | Mamdani FIS (7 Linguistic Rules, Centroid Defuzzification) | **CO2** | Designs synthetic wind dataset and applies a 2-input/1-output Mamdani FIS to modulate flight energy multipliers. |
| **Evolutionary Computing** | GA with Order Crossover (OX-1), 2-Opt Inversion & Repair | **CO3** | Analyzes permutation representations, genetic operators, and chromosome repair in high-dimensional factorial spaces. |
| **Swarm Intelligence** | Ant Colony Optimization (Pheromone $\tau_{ij}$ & Heuristic $\eta_{ij}$) | **CO1, CO3** | Formulates edge transition probabilities and evaluates swarm exploration vs. genetic crossover refinement. |
| **Hybridization** | ACO Population Seeding $\to$ GA Refinement $\to$ FIS Fitness | **CO1, CO2, CO3** | Integrates swarm intelligence, evolutionary search, and fuzzy inference into a unified in-the-loop optimization framework. |

---

## 5. Experimental Summary Table (Preliminary Simulation Benchmark Data for Submission)

| Planning Strategy | Algorithm Category | Field Coverage | Total Energy | 3D Distance | Turns | Obstacle Collisions | Mean Energy across Trials (Mean $\pm$ SD) |
| :--- | :--- | :---: | :---: | :---: | :---: | :---: | :---: |
| **Lawnmower** | Deterministic Cellular Decomposition | $100.0\%$ | $377.00\text{ kJ}$ | $10,709.4\text{ m}$ | 63 | **0** | $377.00 \pm 0.00\text{ kJ}$ |
| **GA** | Standalone Evolutionary Algorithm | $100.0\%$ | $2,275.83\text{ kJ}$ | $73,932.0\text{ m}$ | 340 | **0** | $2,263.38 \pm 26.17\text{ kJ}$ |
| **ACO** | Standalone Swarm Intelligence | $100.0\%$ | $587.55\text{ kJ}$ | $18,121.9\text{ m}$ | 306 | **0** | $587.40 \pm 12.39\text{ kJ}$ |
| **ACO-GA** | Sequential Hybrid Metaheuristic | $100.0\%$ | $588.50\text{ kJ}$ | $18,142.5\text{ m}$ | 305 | **0** | $587.72 \pm 12.41\text{ kJ}$ |
| **ACO-GA-FIS** | **Proposed Wind-Adaptive Hybrid** | $\mathbf{100.0\%}$ | $\mathbf{577.70\text{ kJ}}$ | $\mathbf{17,819.7\text{ m}}$ | **278** | **0** | $\mathbf{566.71 \pm 9.58\text{ kJ}}$ |
