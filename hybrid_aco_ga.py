"""
hybrid_aco_ga.py - Hybrid ACO-GA Optimization Framework with Fuzzy Environmental Adaptation
"""
from typing import List, Tuple, Optional
from config import GAConfig, ACOConfig
from coverage import CoverageGrid
from fitness import FitnessEvaluator, EvaluationMetrics
from ga import GeneticAlgorithm
from aco import AntColonyOptimizer


class HybridACOGAOptimizer:
    """
    Hybrid ACO-GA Optimizer:
    Phase 1: ACO performs global probabilistic exploration, discovering promising topology.
    Phase 2: Best candidate solutions from ACO seed the initial population of the GA.
    Phase 3: GA executes genetic operators (OX crossover, inversion mutation, elitism)
             for local exploitation and route refinement.
    """

    def __init__(self,
                 grid: CoverageGrid,
                 evaluator: FitnessEvaluator,
                 ga_cfg: Optional[GAConfig] = None,
                 aco_cfg: Optional[ACOConfig] = None,
                 seed: int = 42):
        self.grid = grid
        self.evaluator = evaluator
        self.ga_cfg = ga_cfg or GAConfig()
        self.aco_cfg = aco_cfg or ACOConfig()
        self.seed = seed

        self.aco = AntColonyOptimizer(
            grid=self.grid,
            evaluator=self.evaluator,
            aco_cfg=self.aco_cfg,
            seed=self.seed
        )

        self.ga = GeneticAlgorithm(
            valid_cell_ids=self.grid.valid_cell_ids,
            evaluator=self.evaluator,
            ga_cfg=self.ga_cfg,
            seed=self.seed + 1000
        )

        self.convergence_history: List[float] = []
        self.aco_convergence: List[float] = []
        self.ga_convergence: List[float] = []

    def optimize(self) -> Tuple[List[int], EvaluationMetrics]:
        """
        Execute full two-stage hybrid optimization.
        Returns: (best_path, best_metrics)
        """
        # Phase 1: ACO global search
        aco_best_path, aco_metrics, top_candidates = self.aco.optimize()
        self.aco_convergence = list(self.aco.convergence_history)

        # Phase 2 & 3: Seed GA with ACO elite paths and refine
        seed_pool = [aco_best_path] + top_candidates
        ga_best_path, ga_metrics = self.ga.optimize(seed_individuals=seed_pool)
        self.ga_convergence = list(self.ga.convergence_history)

        # Combined convergence trajectory (ACO iterations followed by GA generations)
        self.convergence_history = self.aco_convergence + self.ga_convergence

        # Select whichever is strictly best
        if ga_metrics.cost <= aco_metrics.cost:
            return ga_best_path, ga_metrics
        else:
            return aco_best_path, aco_metrics
