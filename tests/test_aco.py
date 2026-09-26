"""
tests/test_aco.py - Unit Tests for Ant Colony Optimization
"""
import pytest
import numpy as np
from config import ACOConfig, FieldConfig, TerrainConfig, ObstacleConfig, EnergyConfig, WindConfig
from environment import Environment
from coverage import CoverageGrid
from energy import EnergyModel
from fitness import FitnessEvaluator
from aco import AntColonyOptimizer


@pytest.fixture
def aco_setup():
    f_cfg = FieldConfig(width=200.0, length=200.0, grid_n_x=5, grid_n_y=5)
    t_cfg = TerrainConfig(terrain_type="rolling")
    o_cfg = ObstacleConfig(num_obstacles=1)
    env = Environment(field_cfg=f_cfg, terrain_cfg=t_cfg, obstacle_cfg=o_cfg)
    grid = CoverageGrid(f_cfg, env)

    em = EnergyModel(EnergyConfig(), WindConfig())
    evaluator = FitnessEvaluator(grid, em)
    aco_cfg = ACOConfig(num_ants=4, iterations=3)
    aco = AntColonyOptimizer(grid, evaluator, aco_cfg, seed=42)
    return aco, grid


def test_aco_matrices_dimensions(aco_setup):
    aco, grid = aco_setup
    n = len(grid.valid_cell_ids)
    assert aco.pheromone.shape == (n, n)
    assert aco.heuristic.shape == (n, n)
    assert np.all(aco.pheromone >= aco.tau_min)
    assert np.all(aco.pheromone <= aco.tau_max)


def test_aco_ant_path_construction(aco_setup):
    aco, grid = aco_setup
    path = aco._construct_ant_path()
    valid_ids = grid.valid_cell_ids

    # Path must be a valid permutation of all valid cells
    assert len(path) == len(valid_ids)
    assert set(path) == set(valid_ids)


def test_aco_optimization_run(aco_setup):
    aco, grid = aco_setup
    best_path, metrics, top_candidates = aco.optimize()

    assert len(best_path) == len(grid.valid_cell_ids)
    assert set(best_path) == set(grid.valid_cell_ids)
    assert metrics.coverage_rate == 100.0
    assert metrics.energy_joules > 0
    assert len(aco.convergence_history) == aco.cfg.iterations
    assert len(top_candidates) > 0
