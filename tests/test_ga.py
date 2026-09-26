"""
tests/test_ga.py - Unit Tests for Genetic Algorithm Permutation Operators
"""
import pytest
from config import GAConfig, FieldConfig, TerrainConfig, ObstacleConfig, EnergyConfig, WindConfig
from environment import Environment
from coverage import CoverageGrid
from energy import EnergyModel
from fitness import FitnessEvaluator
from ga import GeneticAlgorithm


@pytest.fixture
def ga_setup():
    f_cfg = FieldConfig(width=200.0, length=200.0, grid_n_x=5, grid_n_y=5)
    t_cfg = TerrainConfig(terrain_type="flat")
    o_cfg = ObstacleConfig(num_obstacles=1)
    env = Environment(field_cfg=f_cfg, terrain_cfg=t_cfg, obstacle_cfg=o_cfg)
    grid = CoverageGrid(f_cfg, env)

    em = EnergyModel(EnergyConfig(), WindConfig())
    evaluator = FitnessEvaluator(grid, em)
    ga_cfg = GAConfig(population_size=10, generations=3)
    ga = GeneticAlgorithm(grid.valid_cell_ids, evaluator, ga_cfg, seed=42)
    return ga, grid.valid_cell_ids


def test_ga_initialization(ga_setup):
    ga, valid_ids = ga_setup
    pop = ga.initialize_population()
    assert len(pop) == ga.cfg.population_size
    for ind in pop:
        assert len(ind) == len(valid_ids)
        assert set(ind) == set(valid_ids)


def test_order_crossover(ga_setup):
    ga, valid_ids = ga_setup
    p1 = list(valid_ids)
    p2 = list(reversed(valid_ids))

    c1, c2 = ga.order_crossover(p1, p2)

    # Verify permutation invariants:
    # 1. No duplicates
    assert len(set(c1)) == len(valid_ids)
    assert len(set(c2)) == len(valid_ids)
    # 2. No missing cells
    assert set(c1) == set(valid_ids)
    assert set(c2) == set(valid_ids)
    # 3. Correct length
    assert len(c1) == len(valid_ids)
    assert len(c2) == len(valid_ids)


def test_ga_mutation(ga_setup):
    ga, valid_ids = ga_setup
    p = list(valid_ids)
    mutated = ga.mutate(p)

    # Invariants preserved
    assert len(mutated) == len(valid_ids)
    assert set(mutated) == set(valid_ids)


def test_ga_optimization_run(ga_setup):
    ga, valid_ids = ga_setup
    best_path, metrics = ga.optimize()

    assert len(best_path) == len(valid_ids)
    assert set(best_path) == set(valid_ids)
    assert metrics.cost > 0
    assert len(ga.convergence_history) == ga.cfg.generations + 1
