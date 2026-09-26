"""
tests/test_environment.py - Unit Tests for Terrain and Obstacle Models
"""
import pytest
import numpy as np
from config import FieldConfig, TerrainConfig, ObstacleConfig
from environment import Environment, Obstacle


def test_terrain_elevation_flat():
    cfg = TerrainConfig(terrain_type="flat", clearance_height=20.0)
    env = Environment(terrain_cfg=cfg)
    assert env.elevation(100.0, 100.0) == 0.0
    assert env.uav_altitude(100.0, 100.0) == 20.0


def test_terrain_elevation_rolling():
    cfg = TerrainConfig(terrain_type="rolling", clearance_height=20.0)
    env = Environment(terrain_cfg=cfg)
    z = env.elevation(250.0, 250.0)
    # At center (250, 250), Gaussian peak is 15.0
    assert isinstance(z, float)
    assert env.uav_altitude(250.0, 250.0) == z + 20.0


def test_terrain_elevation_complex():
    cfg = TerrainConfig(terrain_type="complex", clearance_height=20.0)
    env = Environment(terrain_cfg=cfg)
    z = env.elevation(150.0, 350.0)
    assert isinstance(z, float)
    assert env.uav_altitude(150.0, 350.0) == z + 20.0


def test_obstacle_contains_point():
    obs = Obstacle(id=0, x=100.0, y=100.0, radius=20.0, height=30.0, safety_buffer=3.0)
    # Inside radius
    assert obs.contains_point(100.0, 100.0) is True
    assert obs.contains_point(115.0, 100.0) is True
    # Inside safety buffer (effective radius 23.0)
    assert obs.contains_point(122.0, 100.0) is True
    # Outside safety buffer
    assert obs.contains_point(125.0, 100.0) is False


def test_obstacle_segment_intersection():
    obs = Obstacle(id=0, x=100.0, y=100.0, radius=20.0, height=30.0, safety_buffer=3.0)
    # Segment cutting straight across the obstacle center
    assert obs.intersects_segment_2d(50.0, 100.0, 150.0, 100.0) is True
    # Segment clearly away from obstacle
    assert obs.intersects_segment_2d(50.0, 200.0, 150.0, 200.0) is False
