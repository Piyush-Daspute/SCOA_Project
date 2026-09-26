"""
baseline.py - Deterministic Boustrophedon (Lawnmower) Coverage Baseline
"""
from typing import List, Tuple
from coverage import CoverageGrid
from fitness import FitnessEvaluator, EvaluationMetrics


class LawnmowerPlanner:
    """
    Deterministic Boustrophedon (Lawnmower) coverage path planner.
    Sweeps alternating rows from left-to-right and right-to-left,
    visiting every valid cell while skipping obstacle-obstructed cells.
    """

    def __init__(self, grid: CoverageGrid):
        self.grid = grid

    def plan_path(self) -> List[int]:
        """
        Generate deterministic boustrophedon sequence of valid cell IDs using
        cellular decomposition into contiguous obstacle-free row segments.
        """
        nx = self.grid.field_cfg.grid_n_x
        ny = self.grid.field_cfg.grid_n_y

        # Step 1: Decompose grid into contiguous valid row segments
        row_segments: List[List[int]] = []
        for j in range(ny):
            curr_seg: List[int] = []
            for i in range(nx):
                cid = j * nx + i
                if self.grid.cell_map[cid].valid:
                    curr_seg.append(cid)
                else:
                    if curr_seg:
                        row_segments.append(curr_seg)
                        curr_seg = []
            if curr_seg:
                row_segments.append(curr_seg)

        if not row_segments:
            return []

        # Step 2: Connect contiguous segments minimizing transition distance and avoiding obstacle crossings
        unvisited_indices = set(range(len(row_segments)))
        path: List[int] = []

        # Start with the bottom-left segment
        curr_idx = 0
        path.extend(row_segments[curr_idx])
        unvisited_indices.remove(curr_idx)

        while unvisited_indices:
            last_node = path[-1]
            last_u = self.grid.valid_id_to_index[last_node]

            best_cand = None
            best_score = float("inf")
            best_reverse = False

            for s_idx in unvisited_indices:
                seg = row_segments[s_idx]
                # Forward connection
                u_start = self.grid.valid_id_to_index[seg[0]]
                d1 = self.grid.distance_matrix[last_u, u_start]
                b1 = self.grid.is_edge_blocked[last_u, u_start]

                # Reverse connection
                u_end = self.grid.valid_id_to_index[seg[-1]]
                d2 = self.grid.distance_matrix[last_u, u_end]
                b2 = self.grid.is_edge_blocked[last_u, u_end]

                score1 = d1 + (1e6 if b1 else 0)
                score2 = d2 + (1e6 if b2 else 0)

                if score1 < best_score:
                    best_score = score1
                    best_cand = s_idx
                    best_reverse = False
                if score2 < best_score:
                    best_score = score2
                    best_cand = s_idx
                    best_reverse = True

            chosen_seg = row_segments[best_cand]
            if best_reverse:
                chosen_seg = list(reversed(chosen_seg))

            path.extend(chosen_seg)
            unvisited_indices.remove(best_cand)

        # Final safety check / repair
        repaired_path = self.grid.repair_obstacle_collisions(path)
        return repaired_path

    def evaluate(self, evaluator: FitnessEvaluator) -> Tuple[List[int], EvaluationMetrics]:
        """Plan path and evaluate using the fitness evaluator."""
        path = self.plan_path()
        metrics = evaluator.evaluate_path(path)
        return path, metrics
