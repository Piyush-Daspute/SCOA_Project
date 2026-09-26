# IEEE Formatted Abstracts for SCOA FA-1 Research Article

**Course:** Soft Computing and Optimization Algorithms (SCOA)  
**Department:** Computer Engineering (AI & ML), Final Year B.Tech, PCCoE, Pune  
**Faculty Evaluator:** Dr. Anuradha Thakare  
**Course Outcomes Addressed:** CO1 (Soft Computing Techniques), CO2 (Fuzzy Inference Systems), CO3 (Evolutionary Computing Analysis)  
**Project Reference:** *Energy-Aware Wind-Adaptive 3D Coverage Path Planning for Agricultural UAVs Using Hybrid ACO-GA with Mamdani Fuzzy Cost Adaptation*

---

## Abstract Variant 1: Comprehensive & Scientifically Balanced (Recommended for Final Submission)
*Best suited for the main submission of the IEEE research paper. Accurately reflects experimental data without overclaiming, mentions the condition-dependent nature of wind adaptation, and highlights both baseline and metaheuristic findings.*

> **Abstract**—Autonomous Unmanned Aerial Vehicles (UAVs) performing agricultural coverage missions face severe battery endurance limitations under 3D terrain topography, obstacle boundaries, and adverse atmospheric winds. While classical boustrophedon heuristics sweep regular rectangular areas efficiently, they lack dynamic adaptation to aerodynamic drag penalties caused by unfavorable relative wind headings. This paper presents a controlled experimental integration and ablation study evaluating an Ant Colony Optimization (ACO) and Genetic Algorithm (GA) framework augmented with an in-the-loop Mamdani Fuzzy Inference System (FIS) for wind-adaptive 3D agricultural coverage path planning. The workspace is modeled as a $500\text{ m} \times 500\text{ m}$ field ($20 \times 20$ grid, 374 valid cells) with circular obstacle buffers and a constant $20\text{ m}$ terrain-following clearance. A 7-rule Mamdani FIS maps wind speed ($0-12\text{ m/s}$) and angularly wrapped relative flight heading ($0^\circ-180^\circ$) to an environmental energy multiplier ($\mu_{wind} \in [1.0, 2.0]$) evaluated inside a decoupled physical energy formulation ($E = P \cdot t$). Preliminary simulation comparisons demonstrate that: (1) ACO's pheromone-guided transitions discover substantially more efficient routes ($586.3\text{ kJ}$) than standalone GA ($2273.8\text{ kJ}$) in factorial permutation spaces; (2) incorporating fuzzy environmental adaptation yields a condition-dependent energy reduction ($1.83\%$ savings at $6\text{ m/s}$; $576.5\text{ kJ}$ vs. $587.3\text{ kJ}$) while guaranteeing $100\%$ coverage and zero obstacle violations; and (3) a deterministic boustrophedon baseline achieves the lowest energy on uniform contiguous fields ($375.7\text{ kJ}$), demonstrating that heuristic simplicity remains superior on regular spatial topologies. These preliminary computational results establish a controlled benchmark, with ongoing refinements directed toward aerodynamic horizontal-energy calibration prior to final physical flight trials.
>
> **Index Terms**—Agricultural UAV, Coverage Path Planning, Ant Colony Optimization, Genetic Algorithm, Mamdani Fuzzy Inference System.

---

## Abstract Variant 2: Soft Computing & Evolutionary Optimization Focus (CO1 & CO3 Emphasized)
*Best suited for viva and presentation contexts emphasizing algorithm dynamics, search space complexity, and hybridization mechanisms.*

> **Abstract**—Coverage path planning for agricultural drones operating in 3D terrain can be formulated as a constrained Hamiltonian path optimization problem characterized by a combinatorial search space of factorial complexity ($N!$). This study presents a comparative evolutionary computing analysis evaluating Genetic Algorithms (GA), Ant Colony Optimization (ACO), a sequential Hybrid ACO-GA, and an environmentally adaptive ACO-GA-FIS framework on a $20 \times 20$ discretized agricultural field ($374$ valid nodes). The standalone GA utilizes Order Crossover (OX-1) and 2-opt inversion mutation but exhibits slow convergence across high-dimensional permutation spaces under limited evaluation budgets ($2275.8\text{ kJ}$). In contrast, ACO utilizes positive pheromone feedback ($\tau_{ij} \in [0.01, 50.0]$) and heuristic visibility ($\eta_{ij}$) to identify compact edge transitions, reducing route energy to $587.5\text{ kJ}$. In the hybrid configuration, elite ACO tours seed the initial GA population, while an embedded 7-rule Mamdani Fuzzy Inference System dynamically penalizes headwind and crosswind transitions during fitness evaluation. Repeated stochastic trials reveal that while the sequential GA stage contributes marginal local refinement over pure ACO under equalized budgets, fuzzy cost adaptation actively discourages flights into adverse wind vectors, achieving a $3.55\%$ mean energy reduction over standard ACO-GA with verified $100\%$ coverage completeness and zero obstacle collisions.
>
> **Index Terms**—Evolutionary Computing, Genetic Algorithms, Ant Colony Optimization, Hybrid Metaheuristics, Combinatorial Permutation Optimization, Fitness Modulation, Soft Computing.

---

## Abstract Variant 3: Fuzzy Inference & Aerodynamic Wind-Adaptation Focus (CO2 Emphasized)
*Best suited for evaluation sections emphasizing fuzzy dataset design, linguistic rules, membership functions, and defuzzification accuracy.*

> **Abstract**—Environmental wind vectors significantly distort the energy requirements of agricultural UAV missions, as aerodynamic drag increases substantially during headwind and crosswind flight legs. To address the lack of computationally efficient aerodynamic models in discrete path planners, this paper develops a pure numerical Mamdani Fuzzy Inference System (FIS) embedded directly into the optimization loop of a 3D coverage planner. The FIS accepts two continuous inputs: simulated wind velocity ($0-12\text{ m/s}$) fuzzified via triangular and shoulder membership functions (Low, Moderate, High), and relative flight heading ($\theta_{rel} \in [0^\circ, 180^\circ]$) computed through modular angular wrap-around to eliminate boundary cut-off errors (Tailwind, Crosswind, Headwind). A linguistic rule base of seven inference rules utilizes min-implication and max-aggregation to yield a defuzzified centroid multiplier $\mu_{wind} \in [1.0, 2.0]$. When integrated into a Hybrid ACO-GA optimizer over a synthetic rolling agricultural terrain, parametric sweeps demonstrate zero penalty at $0\text{ m/s}$ wind, a $1.83\%$ energy reduction at $6\text{ m/s}$ crosswind ($577.7\text{ kJ}$ vs. $588.5\text{ kJ}$), and a $1.70\%$ reduction at $12\text{ m/s}$. The results confirm that qualitative aerodynamic heuristics can be formalized mathematically into a continuous penalty function that guides path optimization toward aerodynamically favorable trajectories.
>
> **Index Terms**—Fuzzy Inference System, Mamdani FIS, Centroid Defuzzification, Wind Drag Adaptation, Relative Heading Wrap-Around, Soft Computing, Environmental Modeling.

---

## Abstract Variant 4: Precision Agriculture & 3D Terrain-Following Focus
*Best suited for an application-centric or precision agriculture presentation, highlighting terrain elevation, safety buffers, and practical flight feasibility.*

> **Abstract**—Autonomous agricultural UAVs conducting crop health monitoring, field mapping, and targeted spraying must navigate realistic 3D elevation variations while strictly adhering to no-fly obstacle buffers. Conventional 2D planar coverage models produce inaccurate energy estimations and risk ground collisions over rolling topography. This paper develops a 3D terrain-following coverage path planning framework tested across synthetic agricultural scenarios: Flat ($T_1$), Rolling ($T_2$), and Complex multi-hill terrain ($T_3$). The UAV maintains a constant $20\text{ m}$ clearance altitude above terrain ($z_{UAV} = z_{terrain} + 20\text{ m}$), where vertical climb power ($P_{climb} = 300\text{ W}$) and base cruise propulsion ($P_{base} = 200\text{ W}$) are decoupled into horizontal 2D displacement duration and positive vertical climb time to ensure dimensional validity ($E = P \cdot t$). Circular obstacles equipped with a $3\text{ m}$ safety buffer are protected via precomputed spatial collision matrices and 2-opt untangling repair operators. Optimization via a Mamdani-adapted Hybrid ACO-GA demonstrates full $100\%$ target cell coverage across all terrain types without collision violations. Furthermore, the planner selectively avoids flight headings that combine steep terrain climbing with adverse headwinds, providing a robust optimization baseline for precision agriculture missions.
>
> **Index Terms**—Precision Agriculture, 3D Terrain-Following, Obstacle Clearance, Digital Elevation Model, UAV Trajectory Planning, Multi-Objective Optimization.

---

## Abstract Variant 5: Concise Executive / High-Impact Conference Abstract (< 200 Words)
*Strictly condensed (< 180 words) for space-constrained conference templates, two-column formats, or executive slide summaries.*

> **Abstract**—Agricultural UAV coverage missions are severely constrained by battery endurance, 3D terrain elevation, obstacle boundaries, and wind-induced drag. This paper evaluates an integrated simulation framework combining Ant Colony Optimization (ACO), Genetic Algorithms (GA), and a Mamdani Fuzzy Inference System (FIS) for wind-adaptive 3D coverage path planning. The proposed ACO-GA-FIS method optimizes discrete route permutations over a $500\text{ m} \times 500\text{ m}$ terrain grid with circular obstacles and $20\text{ m}$ clearance altitude. A 7-rule Mamdani FIS evaluates wind speed ($0-12\text{ m/s}$) and relative flight heading ($0^\circ-180^\circ$) to assign dynamic energy multipliers in an energy model ($E = P \cdot t$). Experimental comparisons across 374 valid cells show that ACO outperforms standalone GA by $74.2\%$ in energy, while the Mamdani FIS achieves a $3.55\%$ mean energy savings over unadapted ACO-GA across repeated trials with verified $100\%$ coverage and zero collisions. However, a deterministic cellular lawnmower heuristic achieves the lowest energy ($377.0\text{ kJ}$), demonstrating that heuristic simplicity remains superior on uniform contiguous fields.
>
> **Index Terms**—Agricultural UAV, Coverage Path Planning, ACO-GA Hybrid, Mamdani FIS, Wind-Aware Optimization.
