"""
coverage.py - Agricultural Coverage Grid Decomposition, Metrics, and Geometry
"""
from dataclasses import dataclass
from typing import List, Dict, Tuple
import numpy as np
from config import FieldConfig
from environment import Environment


@dataclass
class Cell:
    """Representation of an individual agricultural coverage grid cell."""
    cell_id: int
    grid_i: int
    grid_j: int
    x: float
    y: float
    terrain_z: float
    uav_z: float
    valid: bool
    obstacle: bool


class CoverageGrid:
    """
    Grid decomposition of the 3D agricultural field.
    Handles cell indexing, validity filtering, distance calculation,
    coverage rate, and turn counting.
    """

    def __init__(self, field_cfg: FieldConfig, env: Environment):
        self.field_cfg = field_cfg
        self.env = env

        self.cells: List[Cell] = []
        self.valid_cells: List[Cell] = []
        self.valid_cell_ids: List[int] = []
        self.cell_map: Dict[int, Cell] = {}
        self.valid_id_to_index: Dict[int, int] = {}

        self._decompose()
        self._precompute_distance_matrix()

    def _decompose(self) -> None:
        """Decompose field into grid cells, evaluate obstacle overlap and elevations."""
        nx = self.field_cfg.grid_n_x
        ny = self.field_cfg.grid_n_y
        dx = self.field_cfg.cell_width
        dy = self.field_cfg.cell_length

        cell_id = 0
        for j in range(ny):
            for i in range(nx):
                cx = (i + 0.5) * dx
                cy = (j + 0.5) * dy
                tz = float(self.env.elevation(cx, cy))
                uz = float(self.env.uav_altitude(cx, cy))

                is_obs = self.env.is_point_obstructed(cx, cy)
                is_valid = not is_obs

                cell = Cell(
                    cell_id=cell_id,
                    grid_i=i,
                    grid_j=j,
                    x=cx,
                    y=cy,
                    terrain_z=tz,
                    uav_z=uz,
                    valid=is_valid,
                    obstacle=is_obs
                )
                self.cells.append(cell)
                self.cell_map[cell_id] = cell

                if is_valid:
                    self.valid_id_to_index[cell_id] = len(self.valid_cells)
                    self.valid_cells.append(cell)
                    self.valid_cell_ids.append(cell_id)

                cell_id += 1

    def _precompute_distance_matrix(self) -> None:
        """Precompute 3D Euclidean distances and obstacle collision flags between valid cells."""
        n = len(self.valid_cells)
        coords = np.zeros((n, 3), dtype=float)
        for idx, cell in enumerate(self.valid_cells):
            coords[idx] = [cell.x, cell.y, cell.uav_z]

        # Pairwise distance matrix of valid cells
        diff = coords[:, np.newaxis, :] - coords[np.newaxis, :, :]
        self.distance_matrix = np.sqrt(np.sum(diff ** 2, axis=-1))

        # Pairwise obstacle intersection matrix between valid cells
        self.is_edge_blocked = np.zeros((n, n), dtype=bool)
        for i in range(n):
            c1 = self.valid_cells[i]
            for j in range(i + 1, n):
                c2 = self.valid_cells[j]
                if self.env.segment_violates_obstacle(c1.x, c1.y, c1.uav_z, c2.x, c2.y, c2.uav_z):
                    self.is_edge_blocked[i, j] = True
                    self.is_edge_blocked[j, i] = True

    @property
    def num_target_cells(self) -> int:
        """Number of valid coverage cells requiring UAV visit."""
        return len(self.valid_cells)

    def calculate_path_length(self, path: List[int]) -> float:
        """Calculate total 3D Euclidean distance traveled along the path."""
        if len(path) <= 1:
            return 0.0

        total_d = 0.0
        for k in range(len(path) - 1):
            c1 = self.cell_map[path[k]]
            c2 = self.cell_map[path[k + 1]]
            dx = c2.x - c1.x
            dy = c2.y - c1.y
            dz = c2.uav_z - c1.uav_z
            total_d += float(np.sqrt(dx * dx + dy * dy + dz * dz))
        return total_d

    def calculate_coverage_rate(self, path: List[int]) -> float:
        """
        Calculate Coverage Rate (CR%):
        CR = (N_covered / N_target) * 100
        Only valid target cells are counted toward coverage.
        """
        if self.num_target_cells == 0:
            return 100.0

        visited_valid = set()
        for cid in path:
            if cid in self.cell_map and self.cell_map[cid].valid:
                visited_valid.add(cid)

        return (len(visited_valid) / self.num_target_cells) * 100.0

    def calculate_turn_count(self, path: List[int], threshold_deg: float = 30.0) -> Tuple[int, List[float]]:
        """
        Count turns where angle between consecutive movement vectors exceeds threshold.
        Returns: (turn_count, list_of_turn_angles_deg)
        """
        if len(path) < 3:
            return 0, []

        coords = np.array([[self.cell_map[cid].x,
                            self.cell_map[cid].y,
                            self.cell_map[cid].uav_z] for cid in path], dtype=float)

        # Movement vectors
        v = coords[1:] - coords[:-1]
        v_norms = np.linalg.norm(v, axis=1)

        turns = 0
        turn_angles = []
        for k in range(len(v) - 1):
            n1 = v_norms[k]
            n2 = v_norms[k + 1]
            if n1 < 1e-6 or n2 < 1e-6:
                continue

            dot = np.dot(v[k], v[k + 1])
            cos_theta = np.clip(dot / (n1 * n2), -1.0, 1.0)
            angle_deg = float(np.degrees(np.arccos(cos_theta)))
            turn_angles.append(angle_deg)
            if angle_deg > threshold_deg:
                turns += 1

        return turns, turn_angles

    def count_obstacle_violations(self, path: List[int]) -> int:
        """Count how many line segments between consecutive waypoints intersect obstacles."""
        if len(path) <= 1:
            return 0

        violations = 0
        for k in range(len(path) - 1):
            cid1 = path[k]
            cid2 = path[k + 1]
            if cid1 in self.valid_id_to_index and cid2 in self.valid_id_to_index:
                u = self.valid_id_to_index[cid1]
                v = self.valid_id_to_index[cid2]
                if self.is_edge_blocked[u, v]:
                    violations += 1
            else:
                c1 = self.cell_map[cid1]
                c2 = self.cell_map[cid2]
                if self.env.segment_violates_obstacle(c1.x, c1.y, c1.uav_z, c2.x, c2.y, c2.uav_z):
                    violations += 1
        return violations

    def repair_obstacle_collisions(self, path: List[int], max_passes: int = 100) -> List[int]:
        """
        Repair path using 2-opt inversion moves to untangle segments crossing obstacles.
        Guarantees that the resulting path is a valid permutation of the original path.
        """
        if len(path) <= 3:
            return list(path)

        p = [self.valid_id_to_index[cid] for cid in path]
        n = len(p)

        def count_colls(seq):
            c = 0
            for k in range(len(seq) - 1):
                if self.is_edge_blocked[seq[k], seq[k + 1]]:
                    c += 1
            return c

        current_colls = count_colls(p)
        if current_colls == 0:
            return list(path)

        for _ in range(max_passes):
            improved = False
            for i in range(n - 2):
                if not self.is_edge_blocked[p[i], p[i + 1]]:
                    continue
                # Try 2-opt reversals
                for j in range(i + 2, n):
                    # Invert slice p[i+1 : j+1]
                    # Affected edges: (p[i], p[i+1]) and (p[j], p[j+1] if j+1 < n)
                    # Replaced by:   (p[i], p[j])   and (p[i+1], p[j+1] if j+1 < n)
                    old_cost = (1 if self.is_edge_blocked[p[i], p[i + 1]] else 0)
                    new_cost = (1 if self.is_edge_blocked[p[i], p[j]] else 0)

                    if j + 1 < n:
                        old_cost += (1 if self.is_edge_blocked[p[j], p[j + 1]] else 0)
                        new_cost += (1 if self.is_edge_blocked[p[i + 1], p[j + 1]] else 0)

                    if new_cost < old_cost:
                        p[i + 1 : j + 1] = reversed(p[i + 1 : j + 1])
                        current_colls -= (old_cost - new_cost)
                        improved = True
                        break
                if improved:
                    break

            if not improved or current_colls == 0:
                break

        # Convert back to cell_ids
        repaired_path = [self.valid_cells[idx].cell_id for idx in p]
        return repaired_path
