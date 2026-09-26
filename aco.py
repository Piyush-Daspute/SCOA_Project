"""
aco.py - Ant Colony Optimization for Agricultural UAV Coverage Ordering
"""
from typing import List, Tuple, Optional
import numpy as np
from config import ACOConfig
from coverage import CoverageGrid
from fitness import FitnessEvaluator, EvaluationMetrics


class AntColonyOptimizer:
    """
    Ant Colony Optimization (ACO) algorithm for Coverage Path Planning:
    - Probabilistic path construction using pheromone and Euclidean heuristic
    - Pheromone evaporation and elitist deposit
    - Bounds clamping to prevent numerical underflow/overflow
    - Convergence logging
    """

    def __init__(self,
                 grid: CoverageGrid,
                 evaluator: FitnessEvaluator,
                 aco_cfg: Optional[ACOConfig] = None,
                 seed: int = 42):
        self.grid = grid
        self.evaluator = evaluator
        self.cfg = aco_cfg or ACOConfig()
        self.rng = np.random.default_rng(seed)

        self.valid_cell_ids = list(grid.valid_cell_ids)
        self.n_nodes = len(self.valid_cell_ids)

        # Distance matrix index mapping (0 to n_nodes-1)
        self.dist_matrix = grid.distance_matrix.copy()

        # Heuristic visibility: eta_ij = 1 / (cost_ij + eps)
        eps = 1e-4
        if self.evaluator.use_fis and self.evaluator.energy_model.wind_cfg.speed > 0.0:
            # Calculate pairwise wind multipliers
            n = self.n_nodes
            coords = np.array([[c.x, c.y] for c in self.grid.valid_cells], dtype=float)
            dx_mat = coords[:, np.newaxis, 0] - coords[np.newaxis, :, 0]
            dy_mat = coords[:, np.newaxis, 1] - coords[np.newaxis, :, 1]
            hdg_mat = np.degrees(np.arctan2(dy_mat, dx_mat)) % 360.0
            wind_dir = self.evaluator.energy_model.wind_cfg.direction
            rel_mat = np.abs(((hdg_mat - wind_dir + 180.0) % 360.0) - 180.0)
            mu_mat = self.evaluator.energy_model.fis.evaluate_batch(
                self.evaluator.energy_model.wind_cfg.speed, rel_mat.flatten()
            ).reshape(n, n)
            effective_cost = self.dist_matrix * mu_mat
        else:
            effective_cost = self.dist_matrix

        self.heuristic = 1.0 / (effective_cost + eps)
        np.fill_diagonal(self.heuristic, 0.0)
        # Suppress blocked edges
        self.heuristic[self.grid.is_edge_blocked] = 0.0

        # Pheromone matrix tau_ij
        self.tau_min = 0.01
        self.tau_max = 50.0
        self.pheromone = np.full((self.n_nodes, self.n_nodes),
                                 self.cfg.initial_pheromone,
                                 dtype=float)

        self.convergence_history: List[float] = []

    def _construct_ant_path(self) -> List[int]:
        """Construct a complete valid cell permutation tour for a single ant."""
        n = self.n_nodes
        unvisited = np.ones(n, dtype=bool)

        # Start at random node
        start_idx = int(self.rng.integers(0, n))
        path_indices = [start_idx]
        unvisited[start_idx] = False

        curr_node = start_idx
        alpha = self.cfg.alpha
        beta = self.cfg.beta

        for _ in range(n - 1):
            allowed_indices = np.where(unvisited)[0]

            # Prefer unblocked edges over obstacle-violating edges
            unblocked_mask = ~self.grid.is_edge_blocked[curr_node, allowed_indices]
            if np.any(unblocked_mask):
                candidate_indices = allowed_indices[unblocked_mask]
            else:
                candidate_indices = allowed_indices

            # Compute desirability: tau^alpha * eta^beta
            tau_vals = self.pheromone[curr_node, candidate_indices] ** alpha
            eta_vals = self.heuristic[curr_node, candidate_indices] ** beta
            weights = tau_vals * eta_vals

            w_sum = np.sum(weights)
            if w_sum < 1e-12:
                # Uniform random fallback if probabilities underflow
                next_node = int(self.rng.choice(candidate_indices))
            else:
                probs = weights / w_sum
                next_node = int(self.rng.choice(candidate_indices, p=probs))

            path_indices.append(next_node)
            unvisited[next_node] = False
            curr_node = next_node

        # Map matrix indices back to actual cell_ids
        path_cell_ids = [self.valid_cell_ids[idx] for idx in path_indices]
        # Repair any residual collisions
        repaired_path = self.grid.repair_obstacle_collisions(path_cell_ids)
        return repaired_path

    def optimize(self) -> Tuple[List[int], EvaluationMetrics, List[List[int]]]:
        """
        Execute ACO optimization loop.
        Returns:
            best_path: list of cell IDs of the best solution found
            best_metrics: EvaluationMetrics of best solution
            top_candidates: list of unique high-quality paths to seed GA in hybrid mode
        """
        best_path: Optional[List[int]] = None
        best_cost = float("inf")
        top_candidates: List[Tuple[float, List[int]]] = []

        self.convergence_history = []

        for it in range(self.cfg.iterations):
            iteration_paths = []
            iteration_costs = []

            # Construct solutions for all ants
            for ant in range(self.cfg.num_ants):
                path = self._construct_ant_path()
                cost = self.evaluator(path)
                iteration_paths.append(path)
                iteration_costs.append(cost)

                if cost < best_cost:
                    best_cost = cost
                    best_path = list(path)

            self.convergence_history.append(best_cost)

            # Store best paths for hybrid seeding
            for path, cost in zip(iteration_paths, iteration_costs):
                top_candidates.append((cost, path))

            # Evaporation
            self.pheromone = (1.0 - self.cfg.evaporation_rate) * self.pheromone

            # Pheromone deposit (top elite ants in this iteration)
            best_iter_indices = np.argsort(iteration_costs)[:max(1, self.cfg.num_ants // 3)]
            for idx in best_iter_indices:
                p = iteration_paths[idx]
                c = iteration_costs[idx]
                delta_tau = self.cfg.q_constant / (c + 1e-4)

                # Deposit along edges of path
                p_indices = [self.grid.valid_id_to_index[cid] for cid in p]
                for k in range(len(p_indices) - 1):
                    u = p_indices[k]
                    v = p_indices[k + 1]
                    self.pheromone[u, v] += delta_tau
                    self.pheromone[v, u] += delta_tau

            # Clamp pheromone values for numerical stability
            self.pheromone = np.clip(self.pheromone, self.tau_min, self.tau_max)

        # Sort top candidate paths and keep unique ones
        top_candidates.sort(key=lambda item: item[0])
        unique_candidates: List[List[int]] = []
        seen_first_five = set()
        for _, path in top_candidates:
            # Use tuple of first 5 nodes as lightweight uniqueness signature
            sig = tuple(path[:5])
            if sig not in seen_first_five:
                seen_first_five.add(sig)
                unique_candidates.append(path)
            if len(unique_candidates) >= 15:
                break

        assert best_path is not None, "ACO failed to construct any valid path"
        best_metrics = self.evaluator.evaluate_path(best_path)
        return best_path, best_metrics, unique_candidates
