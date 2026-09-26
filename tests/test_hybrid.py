"""
tests/test_hybrid.py - Unit Tests for Hybrid ACO-GA Optimizer
"""
import pytest
from config import GAConfig, ACOConfig, FieldConfig, TerrainConfig, ObstacleConfig, EnergyConfig, WindConfig
from environment import Environment
from coverage import CoverageGrid
from energy import EnergyModel
from fitness import FitnessEvaluator
from hybrid_aco_ga import HybridACOGAOptimizer


def test_hybrid_aco_ga_execution():
    f_cfg = FieldConfig(width=200.0, length=200.0, grid_n_x=5, grid_n_y=5)
    t_cfg = TerrainConfig(terrain_type="rolling")
    o_cfg = ObstacleConfig(num_obstacles=1)
    env = Environment(field_cfg=f_cfg, terrain_cfg=t_cfg, obstacle_cfg=o_cfg)
    grid = CoverageGrid(f_cfg, env)

    w_cfg = WindConfig(speed=6.0, direction=90.0)
    em = EnergyModel(EnergyConfig(), w_cfg)
    evaluator = FitnessEvaluator(grid, em, use_fis=True)

    ga_cfg = GAConfig(population_size=6, generations=3)
    aco_cfg = ACOConfig(num_ants=3, iterations=2)

    hybrid = HybridACOGAOptimizer(grid, evaluator, ga_cfg, aco_cfg, seed=42)
    best_path, metrics = hybrid.optimize()

    # Solution verification
    assert len(best_path) == len(grid.valid_cell_ids)
    assert set(best_path) == set(grid.valid_cell_ids)
    assert metrics.coverage_rate == 100.0
    assert metrics.energy_joules > 0
    assert metrics.distance_meters > 0
    assert len(hybrid.convergence_history) == aco_cfg.iterations + ga_cfg.generations + 1
