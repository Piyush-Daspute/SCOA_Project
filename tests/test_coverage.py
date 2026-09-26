"""
tests/test_coverage.py - Unit Tests for Coverage Grid Decomposition and Geometry
"""
import pytest
import numpy as np
from config import FieldConfig, TerrainConfig, ObstacleConfig
from environment import Environment
from coverage import CoverageGrid


@pytest.fixture
def sample_grid():
    f_cfg = FieldConfig(width=500.0, length=500.0, grid_n_x=10, grid_n_y=10)
    t_cfg = TerrainConfig(terrain_type="flat", clearance_height=20.0)
    o_cfg = ObstacleConfig(num_obstacles=2)
    env = Environment(field_cfg=f_cfg, terrain_cfg=t_cfg, obstacle_cfg=o_cfg)
    return CoverageGrid(f_cfg, env)


def test_grid_decomposition(sample_grid):
    assert len(sample_grid.cells) == 100
    assert sample_grid.num_target_cells > 0
    assert sample_grid.num_target_cells < 100
    assert len(sample_grid.valid_cells) == sample_grid.num_target_cells


def test_coverage_rate(sample_grid):
    valid_ids = sample_grid.valid_cell_ids
    # Full coverage
    cr_full = sample_grid.calculate_coverage_rate(valid_ids)
    assert pytest.approx(cr_full, 0.01) == 100.0

    # Half coverage
    half_ids = valid_ids[:len(valid_ids) // 2]
    cr_half = sample_grid.calculate_coverage_rate(half_ids)
    expected_half = (len(half_ids) / len(valid_ids)) * 100.0
    assert pytest.approx(cr_half, 0.01) == expected_half


def test_distance_calculation(sample_grid):
    # Test known path of two adjacent cells horizontally
    dx = sample_grid.field_cfg.cell_width
    c1 = sample_grid.cells[0]
    c2 = sample_grid.cells[1]
    dist = sample_grid.calculate_path_length([c1.cell_id, c2.cell_id])
    assert pytest.approx(dist, 0.01) == dx


def test_turn_count(sample_grid):
    # Path with 90 degree turn: (0,0) -> (10,0) -> (10,10)
    c0 = sample_grid.cells[0]  # (25, 25)
    c1 = sample_grid.cells[1]  # (75, 25) - moves East
    c11 = sample_grid.cells[11] # (75, 75) - moves North (90 degree turn)
    turns, angles = sample_grid.calculate_turn_count([c0.cell_id, c1.cell_id, c11.cell_id], threshold_deg=30.0)
    assert turns == 1
    assert pytest.approx(angles[0], 0.1) == 90.0
