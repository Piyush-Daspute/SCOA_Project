"""
fitness.py - Multi-Objective Fitness and Cost Evaluation Function
"""
from dataclasses import dataclass
from typing import List, Dict, Any, Optional
import numpy as np
from config import FitnessConfig
from coverage import CoverageGrid
from energy import EnergyModel


@dataclass
class EvaluationMetrics:
    """Detailed performance metrics for an evaluated flight path."""
    cost: float
    energy_joules: float
    distance_meters: float
    turns: int
    coverage_rate: float
    obstacle_violations: int
    e_horizontal: float
    e_climb: float
    e_turn: float

    def to_dict(self) -> Dict[str, Any]:
        return {
            "cost": self.cost,
            "energy_kJ": self.energy_joules / 1000.0,
            "energy_J": self.energy_joules,
            "distance_m": self.distance_meters,
            "turns": self.turns,
            "coverage_pct": self.coverage_rate,
            "obstacle_violations": self.obstacle_violations,
            "e_horizontal_kJ": self.e_horizontal / 1000.0,
            "e_climb_kJ": self.e_climb / 1000.0,
            "e_turn_J": self.e_turn,
        }


class FitnessEvaluator:
    """
    Evaluates path candidate solutions against normalized objectives:
      J(P) = w_E * E_hat + w_D * D_hat + w_T * T_hat + w_C * L_C + w_R * Obstacle_Penalty
    """

    def __init__(self,
                 grid: CoverageGrid,
                 energy_model: EnergyModel,
                 fitness_cfg: Optional[FitnessConfig] = None,
                 use_fis: bool = True):
        self.grid = grid
        self.energy_model = energy_model
        self.cfg = fitness_cfg or FitnessConfig()
        self.use_fis = use_fis

        # Normalization reference bounds
        n_cells = max(1, self.grid.num_target_cells)
        cell_size = self.grid.field_cfg.cell_width

        # Minimum theoretical distance: contiguous sweep of all cells
        self.d_min = n_cells * cell_size
        # Maximum realistic distance: random permutation hopping
        self.d_max = n_cells * cell_size * 5.0

        # Minimum energy estimate
        self.e_min = (self.energy_model.energy_cfg.p_base * (self.d_min / self.energy_model.energy_cfg.v_cruise))
        self.e_max = self.e_min * 10.0

        # Turn estimates
        self.t_min = float(self.grid.field_cfg.grid_n_y)
        self.t_max = float(n_cells)

        self.eps = 1e-6

    def evaluate_path(self, path: List[int]) -> EvaluationMetrics:
        """Evaluate complete path and return cost J(P) and detailed metrics."""
        # 1. 3D Distance
        distance = self.grid.calculate_path_length(path)

        # 2. Coverage Rate
        coverage = self.grid.calculate_coverage_rate(path)
        loss_coverage = 1.0 - (coverage / 100.0)

        # 3. Turns
        turns, _ = self.grid.calculate_turn_count(path, self.energy_model.energy_cfg.turn_threshold_deg)

        # 4. Energy (with or without FIS wind cost adaptation)
        e_total, e_horiz, e_climb, e_turn = self.energy_model.calculate_path_energy(
            path, self.grid, use_fis=self.use_fis
        )

        # 5. Obstacle violations
        violations = self.grid.count_obstacle_violations(path)

        # Normalizations
        norm_e = (e_total - self.e_min) / (self.e_max - self.e_min + self.eps)
        norm_e = float(np.clip(norm_e, 0.0, 10.0))

        norm_d = (distance - self.d_min) / (self.d_max - self.d_min + self.eps)
        norm_d = float(np.clip(norm_d, 0.0, 10.0))

        norm_t = (turns - self.t_min) / (self.t_max - self.t_min + self.eps)
        norm_t = float(np.clip(norm_t, 0.0, 10.0))

        # Scalar weighted objective (minimized)
        cost = (self.cfg.w_energy * norm_e +
                self.cfg.w_distance * norm_d +
                self.cfg.w_turns * norm_t +
                self.cfg.w_coverage * loss_coverage +
                self.cfg.w_obstacle_penalty * violations)

        return EvaluationMetrics(
            cost=cost,
            energy_joules=e_total,
            distance_meters=distance,
            turns=turns,
            coverage_rate=coverage,
            obstacle_violations=violations,
            e_horizontal=e_horiz,
            e_climb=e_climb,
            e_turn=e_turn
        )

    def __call__(self, path: List[int]) -> float:
        """Callable shorthand returning scalar cost J(P) for optimization algorithms."""
        return self.evaluate_path(path).cost
