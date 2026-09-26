"""
tests/test_energy.py - Unit Tests for Dimensional Consistency and Energy Calculations
"""
import pytest
import numpy as np
from config import EnergyConfig, WindConfig, FieldConfig, TerrainConfig, ObstacleConfig
from environment import Environment
from coverage import CoverageGrid
from energy import EnergyModel, calculate_flight_heading_deg, calculate_relative_wind_angle


def test_flight_heading_deg():
    # East (+X) is 0 deg
    assert pytest.approx(calculate_flight_heading_deg(10.0, 0.0), 0.1) == 0.0
    # North (+Y) is 90 deg
    assert pytest.approx(calculate_flight_heading_deg(0.0, 10.0), 0.1) == 90.0
    # West (-X) is 180 deg
    assert pytest.approx(calculate_flight_heading_deg(-10.0, 0.0), 0.1) == 180.0
    # South (-Y) is 270 deg
    assert pytest.approx(calculate_flight_heading_deg(0.0, -10.0), 0.1) == 270.0


def test_relative_wind_angle_wrap_around():
    # Section 12 test: 350 deg flight vs 10 deg wind should give 20 deg
    rel_ang = calculate_relative_wind_angle(350.0, 10.0)
    assert pytest.approx(rel_ang, 0.1) == 20.0

    # Opposite directions: 0 deg flight vs 180 deg wind -> 180 deg (headwind)
    assert pytest.approx(calculate_relative_wind_angle(0.0, 180.0), 0.1) == 180.0

    # Identical directions: 90 deg flight vs 90 deg wind -> 0 deg (tailwind)
    assert pytest.approx(calculate_relative_wind_angle(90.0, 90.0), 0.1) == 0.0


def test_energy_dimensional_consistency():
    # E = P * t = P * (d / v)
    e_cfg = EnergyConfig(p_base=200.0, p_climb=300.0, p_turn=5.0, v_cruise=10.0, v_climb=2.0)
    w_cfg = WindConfig(speed=0.0, direction=0.0)
    f_cfg = FieldConfig(width=500.0, length=500.0, grid_n_x=10, grid_n_y=10)
    t_cfg = TerrainConfig(terrain_type="flat", clearance_height=20.0)
    o_cfg = ObstacleConfig(num_obstacles=0)

    env = Environment(field_cfg=f_cfg, terrain_cfg=t_cfg, obstacle_cfg=o_cfg)
    grid = CoverageGrid(f_cfg, env)
    em = EnergyModel(e_cfg, w_cfg)

    # Move between 2 adjacent cells: dist = 50m
    # t = 50 / 10 = 5 s
    # E_horiz = 200 W * 5 s = 1000 J
    path = [0, 1]
    e_tot, e_h, e_c, e_t = em.calculate_path_energy(path, grid, use_fis=False)
    assert pytest.approx(e_h, 0.1) == 1000.0
    assert pytest.approx(e_c, 0.1) == 0.0  # Flat terrain, no climb
    assert pytest.approx(e_t, 0.1) == 0.0  # 2 points = no turns
    assert pytest.approx(e_tot, 0.1) == 1000.0
