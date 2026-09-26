"""
energy.py - Dimensionally Consistent UAV Energy Consumption Model with Wind Adaptation
"""
from typing import List, Tuple, Optional
import numpy as np
from config import EnergyConfig, WindConfig
from coverage import CoverageGrid
from fuzzy_system import MamdaniWindFIS


def calculate_flight_heading_deg(dx: float, dy: float) -> float:
    """
    Calculate UAV horizontal heading angle in degrees in [0, 360).
    atan2 returns angle where +X is 0 deg, +Y is 90 deg.
    """
    heading = float(np.degrees(np.arctan2(dy, dx)))
    if heading < 0.0:
        heading += 360.0
    return heading


def calculate_relative_wind_angle(flight_heading_deg: float, wind_dir_deg: float) -> float:
    """
    Calculate relative wind angle theta_rel in [0, 180] with correct wrap-around.
    0 deg = pure tailwind
    90 deg = pure crosswind
    180 deg = pure headwind
    """
    diff = ((flight_heading_deg - wind_dir_deg + 180.0) % 360.0) - 180.0
    return float(abs(diff))


class EnergyModel:
    """
    Dimensionally consistent energy model for 3D agricultural UAV flight:
      E_total = sum(E_horizontal + E_climb) + E_turns
    where:
      E_horizontal,i = P_base * mu_wind,i * (d_i / v_cruise)
      E_climb,i      = P_climb * max(0, dz_i) / v_climb
      E_turns        = P_turn * N_turns
    """

    def __init__(self,
                 energy_cfg: EnergyConfig,
                 wind_cfg: WindConfig,
                 fis: Optional[MamdaniWindFIS] = None):
        self.energy_cfg = energy_cfg
        self.wind_cfg = wind_cfg
        self.fis = fis or MamdaniWindFIS()

    def calculate_path_energy(self,
                              path: List[int],
                              grid: CoverageGrid,
                              use_fis: bool = True) -> Tuple[float, float, float, float]:
        """
        Compute path energy consumption.
        Returns: (E_total_joules, E_horizontal_joules, E_climb_joules, E_turn_joules)
        """
        if len(path) <= 1:
            return 0.0, 0.0, 0.0, 0.0

        n_pts = len(path)
        xs = np.array([grid.cell_map[cid].x for cid in path], dtype=float)
        ys = np.array([grid.cell_map[cid].y for cid in path], dtype=float)
        zs = np.array([grid.cell_map[cid].uav_z for cid in path], dtype=float)

        dx = xs[1:] - xs[:-1]
        dy = ys[1:] - ys[:-1]
        dz = zs[1:] - zs[:-1]

        # 2D horizontal distances and 3D total distances
        d_xy = np.sqrt(dx * dx + dy * dy)
        d_segs = np.sqrt(dx * dx + dy * dy + dz * dz)

        # 1. Base / Horizontal Energy with Wind Multiplier (uses 2D displacement)
        t_base = d_xy / self.energy_cfg.v_cruise

        if use_fis and self.wind_cfg.speed > 0.0:
            # Flight headings in degrees [0, 360)
            headings = np.degrees(np.arctan2(dy, dx)) % 360.0
            # Wrap-around relative wind angles [0, 180]
            rel_angles = np.abs(((headings - self.wind_cfg.direction + 180.0) % 360.0) - 180.0)
            # Evaluate Mamdani FIS multipliers
            mu_wind = self.fis.evaluate_batch(self.wind_cfg.speed, rel_angles)
        else:
            mu_wind = np.ones_like(d_xy)

        e_horizontal_seg = self.energy_cfg.p_base * mu_wind * t_base
        e_horizontal = float(np.sum(e_horizontal_seg))

        # 2. Climbing Energy (only positive vertical changes consume climb power)
        climb_dz = np.maximum(0.0, dz)
        t_climb = climb_dz / self.energy_cfg.v_climb
        e_climb_seg = self.energy_cfg.p_climb * t_climb
        e_climb = float(np.sum(e_climb_seg))

        # 3. Turning Energy
        turn_count, _ = grid.calculate_turn_count(path, self.energy_cfg.turn_threshold_deg)
        e_turn = float(self.energy_cfg.p_turn * turn_count)

        e_total = e_horizontal + e_climb + e_turn
        return e_total, e_horizontal, e_climb, e_turn
