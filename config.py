
"""
config.py - Central Configuration Module for SCOA UAV Coverage Path Planning
"""
import dataclasses
from dataclasses import dataclass
from typing import List, Tuple

# Global seed for reproducibility
SEED = 42


@dataclass
class FieldConfig:
    """Field geometry and discretization parameters."""
    width: float = 500.0   # Field dimension X (meters)
    length: float = 500.0  # Field dimension Y (meters)
    grid_n_x: int = 20     # Number of cells along X
    grid_n_y: int = 20     # Number of cells along Y

    @property
    def cell_width(self) -> float:
        return self.width / self.grid_n_x

    @property
    def cell_length(self) -> float:
        return self.length / self.grid_n_y

    @property
    def total_cells(self) -> int:
        return self.grid_n_x * self.grid_n_y


@dataclass
class TerrainConfig:
    """Terrain model parameters for T1 (Flat), T2 (Rolling), and T3 (Complex)."""
    terrain_type: str = "rolling"  # "flat", "rolling", or "complex"
    clearance_height: float = 20.0  # Constant UAV clearance above terrain (meters)


@dataclass
class ObstacleConfig:
    """Obstacle generation and safety parameters."""
    num_obstacles: int = 6
    safety_buffer: float = 3.0  # d_safe (meters)
    # Pre-defined deterministic obstacle centers (x, y), radius, height
    # Designed within 500m x 500m field
    default_obstacles: List[Tuple[float, float, float, float]] = dataclasses.field(
        default_factory=lambda: [
            (120.0, 150.0, 30.0, 35.0),  # x, y, radius, height
            (350.0, 120.0, 25.0, 30.0),
            (250.0, 320.0, 35.0, 40.0),
            (100.0, 380.0, 20.0, 25.0),
            (400.0, 380.0, 28.0, 30.0),
            (280.0, 180.0, 22.0, 28.0),
        ]
    )


@dataclass
class WindConfig:
    """Synthetic wind condition parameters."""
    speed: float = 6.0          # Wind speed in m/s (0, 3, 6, 9, 12)
    direction: float = 90.0     # Meteorological wind direction in degrees [0, 360)


@dataclass
class EnergyConfig:
    """Simulation-level energy consumption parameters."""
    p_base: float = 200.0       # Base horizontal propulsion power (Watts)
    p_climb: float = 300.0      # Additional climb power (Watts)
    p_turn: float = 5.0         # Energy per turn (Joules/turn)
    v_cruise: float = 10.0      # Nominal horizontal speed (m/s)
    v_climb: float = 2.0        # Vertical climb speed (m/s)
    turn_threshold_deg: float = 30.0  # Heading change angle threshold (degrees)


@dataclass
class FitnessConfig:
    """Multi-objective fitness function weights and penalties."""
    w_energy: float = 0.50
    w_distance: float = 0.25
    w_turns: float = 0.15
    w_coverage: float = 0.10
    w_obstacle_penalty: float = 1e5  # Penalty per obstacle violation


@dataclass
class GAConfig:
    """Genetic Algorithm parameters."""
    population_size: int = 40
    generations: int = 60
    crossover_prob: float = 0.8
    mutation_prob: float = 0.15
    tournament_size: int = 3
    elite_count: int = 2


@dataclass
class ACOConfig:
    """Ant Colony Optimization parameters."""
    num_ants: int = 20
    iterations: int = 50
    alpha: float = 1.0          # Pheromone importance factor
    beta: float = 2.0           # Heuristic desirability factor
    evaporation_rate: float = 0.1
    q_constant: float = 100.0   # Pheromone deposit scaling constant
    initial_pheromone: float = 1.0


@dataclass
class SystemConfig:
    """Top-level system configuration aggregating all subsystems."""
    mode: str = "full"          # "quick" or "full"
    field: FieldConfig = dataclasses.field(default_factory=FieldConfig)
    terrain: TerrainConfig = dataclasses.field(default_factory=TerrainConfig)
    obstacle: ObstacleConfig = dataclasses.field(default_factory=ObstacleConfig)
    wind: WindConfig = dataclasses.field(default_factory=WindConfig)
    energy: EnergyConfig = dataclasses.field(default_factory=EnergyConfig)
    fitness: FitnessConfig = dataclasses.field(default_factory=FitnessConfig)
    ga: GAConfig = dataclasses.field(default_factory=GAConfig)
    aco: ACOConfig = dataclasses.field(default_factory=ACOConfig)
    num_trials: int = 5
    seed: int = SEED


def get_quick_config() -> SystemConfig:
    """Returns lightweight configuration for quick testing and smoke tests."""
    return SystemConfig(
        mode="quick",
        field=FieldConfig(width=500.0, length=500.0, grid_n_x=10, grid_n_y=10),
        terrain=TerrainConfig(terrain_type="rolling", clearance_height=20.0),
        obstacle=ObstacleConfig(num_obstacles=4),
        wind=WindConfig(speed=6.0, direction=90.0),
        energy=EnergyConfig(),
        fitness=FitnessConfig(),
        ga=GAConfig(population_size=15, generations=5, crossover_prob=0.8, mutation_prob=0.15),
        aco=ACOConfig(num_ants=5, iterations=5, alpha=1.0, beta=2.0, evaporation_rate=0.1),
        num_trials=1,
        seed=SEED,
    )


def get_full_config() -> SystemConfig:
    """Returns full experimental configuration for 20x20 grid."""
    return SystemConfig(
        mode="full",
        field=FieldConfig(width=500.0, length=500.0, grid_n_x=20, grid_n_y=20),
        terrain=TerrainConfig(terrain_type="rolling", clearance_height=20.0),
        obstacle=ObstacleConfig(num_obstacles=6),
        wind=WindConfig(speed=6.0, direction=90.0),
        energy=EnergyConfig(),
        fitness=FitnessConfig(),
        ga=GAConfig(population_size=20, generations=20, crossover_prob=0.8, mutation_prob=0.15),
        aco=ACOConfig(num_ants=8, iterations=12, alpha=1.0, beta=2.0, evaporation_rate=0.1),
        num_trials=3,
        seed=SEED,
    )
