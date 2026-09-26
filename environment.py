"""
environment.py - 3D Agricultural Terrain and Obstacle Environment
"""
from dataclasses import dataclass
from typing import List, Tuple, Optional
import numpy as np
from config import TerrainConfig, ObstacleConfig, FieldConfig


@dataclass
class Obstacle:
    """3D circular obstacle model."""
    id: int
    x: float
    y: float
    radius: float
    height: float
    safety_buffer: float = 3.0

    @property
    def effective_radius(self) -> float:
        return self.radius + self.safety_buffer

    def contains_point(self, px: float, py: float) -> bool:
        """Check if 2D coordinate falls within effective radius of obstacle."""
        return bool(np.hypot(px - self.x, py - self.y) <= self.effective_radius)

    def intersects_segment_2d(self, x1: float, y1: float, x2: float, y2: float) -> bool:
        """
        Check if 2D line segment between (x1, y1) and (x2, y2) intersects
        the circular buffer of the obstacle.
        """
        dx = x2 - x1
        dy = y2 - y1
        seg_len_sq = dx * dx + dy * dy
        if seg_len_sq < 1e-9:
            return self.contains_point(x1, y1)

        # Projection factor t of obstacle center onto the line segment
        t = ((self.x - x1) * dx + (self.y - y1) * dy) / seg_len_sq
        t = np.clip(t, 0.0, 1.0)

        # Closest point on line segment
        closest_x = x1 + t * dx
        closest_y = y1 + t * dy

        dist = np.hypot(self.x - closest_x, self.y - closest_y)
        return bool(dist <= self.effective_radius)


class Environment:
    """
    Simulated 3D Agricultural Environment containing terrain elevation,
    UAV clearance altitude, and 3D obstacles.
    """

    def __init__(self,
                 field_cfg: Optional[FieldConfig] = None,
                 terrain_cfg: Optional[TerrainConfig] = None,
                 obstacle_cfg: Optional[ObstacleConfig] = None):
        self.field_cfg = field_cfg or FieldConfig()
        self.terrain_cfg = terrain_cfg or TerrainConfig()
        self.obstacle_cfg = obstacle_cfg or ObstacleConfig()

        self.obstacles: List[Obstacle] = self._create_obstacles()

    def _create_obstacles(self) -> List[Obstacle]:
        """Construct circular obstacles with safety buffers."""
        obs_list = []
        specs = self.obstacle_cfg.default_obstacles[:self.obstacle_cfg.num_obstacles]
        for idx, (x, y, r, h) in enumerate(specs):
            obs = Obstacle(
                id=idx,
                x=x,
                y=y,
                radius=r,
                height=h,
                safety_buffer=self.obstacle_cfg.safety_buffer
            )
            obs_list.append(obs)
        return obs_list

    def elevation(self, x: np.ndarray | float, y: np.ndarray | float) -> np.ndarray | float:
        """
        Calculate terrain elevation Z(x, y) based on terrain scenario.
        T1: Flat
        T2: Rolling (sinusoid + Gaussian)
        T3: Complex (multi-frequency sinusoid + dual Gaussians)
        """
        t_type = self.terrain_cfg.terrain_type.lower()
        if t_type == "flat":
            if isinstance(x, np.ndarray):
                return np.zeros_like(x, dtype=float)
            return 0.0

        elif t_type == "rolling":
            # Formula: 10*sin(x/60) + 7*cos(y/70) + 15*exp(-((x-250)^2 + (y-250)^2)/(2*100^2))
            g = np.exp(-((x - 250.0)**2 + (y - 250.0)**2) / (2.0 * (100.0**2)))
            z = 10.0 * np.sin(x / 60.0) + 7.0 * np.cos(y / 70.0) + 15.0 * g
            return z

        elif t_type == "complex":
            # Multi-hill and valley terrain
            g1 = np.exp(-((x - 150.0)**2 + (y - 350.0)**2) / (2.0 * (70.0**2)))
            g2 = np.exp(-((x - 350.0)**2 + (y - 150.0)**2) / (2.0 * (60.0**2)))
            z = (8.0 * np.sin(x / 40.0) + 6.0 * np.cos(y / 50.0) +
                 12.0 * g1 + 10.0 * g2)
            return z

        else:
            raise ValueError(f"Unknown terrain type: {t_type}")

    def uav_altitude(self, x: np.ndarray | float, y: np.ndarray | float) -> np.ndarray | float:
        """Calculate UAV altitude following terrain clearance: z_uav = z_terrain + h_clearance."""
        return self.elevation(x, y) + self.terrain_cfg.clearance_height

    def is_point_obstructed(self, px: float, py: float) -> bool:
        """Check if 2D coordinate is inside any obstacle safety buffer."""
        for obs in self.obstacles:
            if obs.contains_point(px, py):
                return True
        return False

    def segment_violates_obstacle(self, x1: float, y1: float, z1: float,
                                   x2: float, y2: float, z2: float) -> bool:
        """
        Check if 3D segment intersects an obstacle cylinder horizontally AND
        is below obstacle top elevation. Uses bounding-box pre-filter.
        """
        min_x = min(x1, x2)
        max_x = max(x1, x2)
        min_y = min(y1, y2)
        max_y = max(y1, y2)
        min_z = min(z1, z2)

        for obs in self.obstacles:
            r_eff = obs.effective_radius
            if obs.x < min_x - r_eff or obs.x > max_x + r_eff:
                continue
            if obs.y < min_y - r_eff or obs.y > max_y + r_eff:
                continue

            obs_top_z = self.elevation(obs.x, obs.y) + obs.height
            if min_z >= obs_top_z:
                continue

            if obs.intersects_segment_2d(x1, y1, x2, y2):
                return True
        return False

    def get_terrain_mesh(self, res: int = 50) -> Tuple[np.ndarray, np.ndarray, np.ndarray]:
        """Generate (X, Y, Z) meshgrid matrices for 3D visualization."""
        xs = np.linspace(0.0, self.field_cfg.width, res)
        ys = np.linspace(0.0, self.field_cfg.length, res)
        X, Y = np.meshgrid(xs, ys)
        Z = self.elevation(X, Y)
        return X, Y, Z
