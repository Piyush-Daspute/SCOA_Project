"""
visualization.py - 2D and 3D Visualizations for Agricultural UAV Path Planning
"""
import os
from typing import List, Dict, Any, Optional
import numpy as np
import matplotlib
matplotlib.use("Agg")  # Non-interactive backend for headless file generation
import matplotlib.pyplot as plt
from mpl_toolkits.mplot3d import Axes3D

from environment import Environment
from coverage import CoverageGrid


class Visualizer:
    """
    Visualization engine for generating publication-quality figures:
    - 3D Terrain & Obstacles
    - 2D/3D Coverage Decomposition
    - 3D Trajectory Plots (Lawnmower vs Hybrid ACO-GA-FIS)
    - Performance Comparison Charts (Energy, Distance, Turns, Coverage, Runtime)
    - Wind Sensitivity Curves
    - Convergence Comparison Plots
    """

    def __init__(self, output_dir: str = "figures"):
        self.output_dir = output_dir
        os.makedirs(output_dir, exist_ok=True)
        # Apply standard clean styling
        plt.style.use("seaborn-v0_8-whitegrid" if "seaborn-v0_8-whitegrid" in plt.style.available else "default")
        plt.rcParams.update({
            "font.family": "sans-serif",
            "font.size": 11,
            "axes.titlesize": 13,
            "axes.labelsize": 11,
            "xtick.labelsize": 10,
            "ytick.labelsize": 10,
            "legend.fontsize": 10,
            "figure.titlesize": 14,
        })

    def plot_terrain_3d(self, env: Environment, filename: str = "terrain.png") -> str:
        """Figure 1: 3D Surface representation of the agricultural terrain."""
        fig = plt.figure(figsize=(9, 7))
        ax = fig.add_subplot(111, projection="3d")

        X, Y, Z = env.get_terrain_mesh(res=60)
        surf = ax.plot_surface(X, Y, Z, cmap="terrain", edgecolor="none", alpha=0.9)
        cbar = fig.colorbar(surf, ax=ax, shrink=0.5, aspect=10, pad=0.1)
        cbar.set_label("Elevation (m)")

        ax.set_title(f"3D Agricultural Terrain Model ({env.terrain_cfg.terrain_type.capitalize()})", pad=15)
        ax.set_xlabel("X (m)")
        ax.set_ylabel("Y (m)")
        ax.set_zlabel("Elevation Z (m)")
        ax.view_init(elev=32, azim=-55)

        out_path = os.path.join(self.output_dir, filename)
        fig.tight_layout()
        fig.savefig(out_path, dpi=300)
        plt.close(fig)
        return out_path

    def plot_coverage_grid(self, grid: CoverageGrid, filename: str = "coverage_grid.png") -> str:
        """Figure 2: 2D Field Decomposition showing cells and obstacle safety zones."""
        fig, ax = plt.subplots(figsize=(8, 8))

        # Plot cells
        valid_xs = [c.x for c in grid.valid_cells]
        valid_ys = [c.y for c in grid.valid_cells]
        ax.scatter(valid_xs, valid_ys, c="#2ecc71", s=18, alpha=0.6, label="Valid Target Cells")

        obs_cells = [c for c in grid.cells if not c.valid]
        if obs_cells:
            obs_xs = [c.x for c in obs_cells]
            obs_ys = [c.y for c in obs_cells]
            ax.scatter(obs_xs, obs_ys, c="#e74c3c", s=25, alpha=0.8, marker="x", label="Invalid Obstacle Cells")

        # Plot obstacle circles and safety buffers
        for obs in grid.env.obstacles:
            circle_obs = plt.Circle((obs.x, obs.y), obs.radius, color="#c0392b", alpha=0.5)
            circle_buf = plt.Circle((obs.x, obs.y), obs.effective_radius, color="#e67e22",
                                    fill=False, linestyle="--", linewidth=1.5,
                                    label="Safety Buffer" if obs.id == 0 else "")
            ax.add_patch(circle_obs)
            ax.add_patch(circle_buf)

        ax.set_xlim(0, grid.field_cfg.width)
        ax.set_ylim(0, grid.field_cfg.length)
        ax.set_title(f"Coverage Grid Decomposition ({grid.field_cfg.grid_n_x}x{grid.field_cfg.grid_n_y})", pad=12)
        ax.set_xlabel("X Field Dimension (m)")
        ax.set_ylabel("Y Field Dimension (m)")
        ax.set_aspect("equal")
        ax.legend(loc="upper right", framealpha=0.9)

        out_path = os.path.join(self.output_dir, filename)
        fig.tight_layout()
        fig.savefig(out_path, dpi=300)
        plt.close(fig)
        return out_path

    def plot_trajectory_3d(self,
                           grid: CoverageGrid,
                           path: List[int],
                           title: str,
                           filename: str,
                           color: str = "#2980b9") -> str:
        """3D Trajectory visualization over the terrain with obstacles."""
        fig = plt.figure(figsize=(10, 8))
        ax = fig.add_subplot(111, projection="3d")

        # Plot terrain surface
        X, Y, Z = grid.env.get_terrain_mesh(res=50)
        ax.plot_surface(X, Y, Z, cmap="terrain", alpha=0.55, edgecolor="none")

        # Plot obstacle cylinders
        for obs in grid.env.obstacles:
            theta = np.linspace(0, 2 * np.pi, 25)
            z_top = grid.env.elevation(obs.x, obs.y) + obs.height
            z_bot = grid.env.elevation(obs.x, obs.y)
            z_cyl = np.linspace(z_bot, z_top, 2)
            theta_grid, z_grid = np.meshgrid(theta, z_cyl)
            x_cyl = obs.x + obs.radius * np.cos(theta_grid)
            y_cyl = obs.y + obs.radius * np.sin(theta_grid)
            ax.plot_surface(x_cyl, y_cyl, z_grid, color="#e74c3c", alpha=0.7)

        # Plot UAV Flight Path
        px = [grid.cell_map[cid].x for cid in path]
        py = [grid.cell_map[cid].y for cid in path]
        pz = [grid.cell_map[cid].uav_z for cid in path]

        ax.plot(px, py, pz, color=color, linewidth=2.0, alpha=0.9, label="UAV Flight Path")
        # Markers for Start and End
        if len(px) > 0:
            ax.scatter([px[0]], [py[0]], [pz[0]], color="#27ae60", s=100, marker="^", label="Start Waypoint")
            ax.scatter([px[-1]], [py[-1]], [pz[-1]], color="#8e44ad", s=100, marker="v", label="End Waypoint")

        ax.set_title(title, pad=15)
        ax.set_xlabel("X (m)")
        ax.set_ylabel("Y (m)")
        ax.set_zlabel("Altitude Z (m)")
        ax.view_init(elev=34, azim=-50)
        ax.legend(loc="upper right", framealpha=0.9)

        out_path = os.path.join(self.output_dir, filename)
        fig.tight_layout()
        fig.savefig(out_path, dpi=300)
        plt.close(fig)
        return out_path

    def plot_comparison_bars(self,
                             results: List[Dict[str, Any]],
                             metric_key: str,
                             ylabel: str,
                             title: str,
                             filename: str,
                             color: str = "#3498db") -> str:
        """Generic comparison bar chart for algorithms."""
        fig, ax = plt.subplots(figsize=(8, 5))

        names = [r["algorithm"] for r in results]
        vals = [r[metric_key] for r in results]

        # Use distinct pleasant palette
        colors = ["#7f8c8d", "#e67e22", "#9b59b6", "#2980b9", "#27ae60"]
        bar_colors = colors[:len(names)]

        bars = ax.bar(names, vals, color=bar_colors, width=0.55, edgecolor="black", linewidth=0.8)

        # Add value labels on top of bars
        for bar in bars:
            height = bar.get_height()
            ax.annotate(f"{height:.2f}",
                        xy=(bar.get_x() + bar.get_width() / 2, height),
                        xytext=(0, 4),
                        textcoords="offset points",
                        ha="center", va="bottom", fontsize=9, fontweight="bold")

        ax.set_title(title, pad=12)
        ax.set_ylabel(ylabel)
        ax.set_xlabel("Optimization Algorithm")
        ax.set_ylim(0, max(vals) * 1.18 if vals and max(vals) > 0 else 1.0)

        out_path = os.path.join(self.output_dir, filename)
        fig.tight_layout()
        fig.savefig(out_path, dpi=300)
        plt.close(fig)
        return out_path

    def plot_wind_sensitivity(self,
                              param_values: List[float],
                              results_no_fis: List[float],
                              results_fis: List[float],
                              param_name: str,
                              unit: str,
                              title: str,
                              filename: str) -> str:
        """Line chart comparing ACO-GA vs ACO-GA-FIS under varying wind parameter."""
        fig, ax = plt.subplots(figsize=(8, 5))

        ax.plot(param_values, results_no_fis, marker="o", linewidth=2.2,
                color="#e67e22", label="ACO-GA (Standard)")
        ax.plot(param_values, results_fis, marker="s", linewidth=2.2,
                color="#27ae60", label="ACO-GA-FIS (Wind-Adaptive)")

        for x, y1, y2 in zip(param_values, results_no_fis, results_fis):
            ax.annotate(f"{y1:.1f}", (x, y1), textcoords="offset points", xytext=(-10, 7), fontsize=9)
            ax.annotate(f"{y2:.1f}", (x, y2), textcoords="offset points", xytext=(5, -12), fontsize=9)

        ax.set_title(title, pad=12)
        ax.set_xlabel(f"{param_name} ({unit})")
        ax.set_ylabel("Estimated Energy (kJ)")
        ax.legend(loc="best", framealpha=0.9)

        out_path = os.path.join(self.output_dir, filename)
        fig.tight_layout()
        fig.savefig(out_path, dpi=300)
        plt.close(fig)
        return out_path

    def plot_convergence_comparison(self,
                                   curves: Dict[str, List[float]],
                                   filename: str = "convergence_comparison.png") -> str:
        """Plot comparative convergence trajectory of metaheuristics."""
        fig, ax = plt.subplots(figsize=(9, 5.5))

        palette = {
            "GA": "#e67e22",
            "ACO": "#9b59b6",
            "ACO-GA": "#2980b9",
            "ACO-GA-FIS": "#27ae60"
        }

        for algo_name, cost_history in curves.items():
            if not cost_history:
                continue
            ax.plot(range(1, len(cost_history) + 1), cost_history,
                    label=algo_name, linewidth=2.0, color=palette.get(algo_name, "#34495e"))

        ax.set_title("Fitness Cost Convergence Comparison", pad=12)
        ax.set_xlabel("Iteration / Generation Index")
        ax.set_ylabel("Best Objective Cost J(P)")
        ax.legend(loc="upper right", framealpha=0.9)

        out_path = os.path.join(self.output_dir, filename)
        fig.tight_layout()
        fig.savefig(out_path, dpi=300)
        plt.close(fig)
        return out_path
