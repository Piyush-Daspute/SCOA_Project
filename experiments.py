"""
experiments.py - Automated Research Experiments Suite for UAV Coverage Path Planning
"""
import os
import time
from typing import List, Dict, Any, Tuple
import pandas as pd
import numpy as np

from config import SystemConfig, WindConfig, TerrainConfig
from environment import Environment
from coverage import CoverageGrid
from energy import EnergyModel
from fitness import FitnessEvaluator
from baseline import LawnmowerPlanner
from ga import GeneticAlgorithm
from aco import AntColonyOptimizer
from hybrid_aco_ga import HybridACOGAOptimizer
from visualization import Visualizer
from statistics_analysis import StatisticalAnalyzer


class ExperimentRunner:
    """
    Executes and records:
    1. Main Algorithm Comparison (Lawnmower, GA, ACO, ACO-GA, ACO-GA-FIS)
    2. Wind Speed Sensitivity Sweep (0, 6, 12 m/s)
    3. Wind Direction Sensitivity Sweep (0, 90, 180 deg)
    4. Terrain Complexity Analysis (Flat, Rolling, Complex)
    5. Ablation Study (ACO-GA vs ACO-GA-FIS)
    6. Repeated Trials & Statistical Hypothesis Testing
    """

    def __init__(self, sys_cfg: SystemConfig, results_dir: str = "results", figures_dir: str = "figures"):
        self.cfg = sys_cfg
        self.results_dir = results_dir
        self.figures_dir = figures_dir
        os.makedirs(results_dir, exist_ok=True)
        os.makedirs(figures_dir, exist_ok=True)

        self.visualizer = Visualizer(output_dir=figures_dir)

    def run_single_algorithm(self,
                             algo_name: str,
                             grid: CoverageGrid,
                             evaluator: FitnessEvaluator,
                             seed: int) -> Tuple[List[int], Dict[str, Any], List[float]]:
        """Run a single optimizer, timing its execution."""
        t0 = time.time()
        convergence: List[float] = []

        if algo_name == "Lawnmower":
            planner = LawnmowerPlanner(grid)
            path, metrics = planner.evaluate(evaluator)
            runtime = time.time() - t0
            convergence = [metrics.cost]

        elif algo_name == "GA":
            optimizer = GeneticAlgorithm(
                valid_cell_ids=grid.valid_cell_ids,
                evaluator=evaluator,
                ga_cfg=self.cfg.ga,
                seed=seed
            )
            path, metrics = optimizer.optimize()
            runtime = time.time() - t0
            convergence = optimizer.convergence_history

        elif algo_name == "ACO":
            optimizer = AntColonyOptimizer(
                grid=grid,
                evaluator=evaluator,
                aco_cfg=self.cfg.aco,
                seed=seed
            )
            path, metrics, _ = optimizer.optimize()
            runtime = time.time() - t0
            convergence = optimizer.convergence_history

        elif algo_name in ("ACO-GA", "ACO-GA-FIS"):
            optimizer = HybridACOGAOptimizer(
                grid=grid,
                evaluator=evaluator,
                ga_cfg=self.cfg.ga,
                aco_cfg=self.cfg.aco,
                seed=seed
            )
            path, metrics = optimizer.optimize()
            runtime = time.time() - t0
            convergence = optimizer.convergence_history

        else:
            raise ValueError(f"Unknown algorithm: {algo_name}")

        res = metrics.to_dict()
        res["algorithm"] = algo_name
        res["runtime_s"] = round(runtime, 3)

        return path, res, convergence

    def run_experiment_1_main(self) -> Tuple[pd.DataFrame, Dict[str, Any]]:
        """
        Experiment 1: Main Comparison under standard conditions (Rolling, 6 m/s, 90 deg).
        Compares all 5 algorithms: Lawnmower, GA, ACO, ACO-GA, ACO-GA-FIS.
        """
        print("\n--- Running Experiment 1: Main Algorithm Comparison ---")
        env = Environment(field_cfg=self.cfg.field,
                          terrain_cfg=self.cfg.terrain,
                          obstacle_cfg=self.cfg.obstacle)
        grid = CoverageGrid(self.cfg.field, env)

        # Plot terrain and grid structure
        self.visualizer.plot_terrain_3d(env, "terrain.png")
        self.visualizer.plot_coverage_grid(grid, "coverage_grid.png")

        # Two evaluators: with FIS (proposed) and without FIS (standard)
        energy_model = EnergyModel(self.cfg.energy, self.cfg.wind)
        evaluator_fis = FitnessEvaluator(grid, energy_model, self.cfg.fitness, use_fis=True)
        evaluator_no_fis = FitnessEvaluator(grid, energy_model, self.cfg.fitness, use_fis=False)

        algorithms = ["Lawnmower", "GA", "ACO", "ACO-GA", "ACO-GA-FIS"]
        results_list = []
        paths: Dict[str, List[int]] = {}
        convergence_dict: Dict[str, List[float]] = {}

        for algo in algorithms:
            # Only ACO-GA-FIS incorporates FIS inside fitness during optimization
            ev = evaluator_fis if algo == "ACO-GA-FIS" else evaluator_no_fis

            path, res, conv = self.run_single_algorithm(algo, grid, ev, seed=self.cfg.seed)

            # Re-evaluate in the true ground-truth wind environment for fair comparison
            final_metrics = evaluator_fis.evaluate_path(path).to_dict()
            final_metrics["algorithm"] = algo
            final_metrics["runtime_s"] = res["runtime_s"]

            results_list.append(final_metrics)
            paths[algo] = path
            convergence_dict[algo] = conv
            print(f"  [{algo:10s}] Energy: {final_metrics['energy_kJ']:7.2f} kJ | "
                  f"Dist: {final_metrics['distance_m']:7.1f} m | "
                  f"Turns: {final_metrics['turns']:3d} | "
                  f"Coverage: {final_metrics['coverage_pct']:5.1f}% | "
                  f"Time: {final_metrics['runtime_s']:5.2f} s")

        df = pd.DataFrame(results_list)
        df.to_csv(os.path.join(self.results_dir, "main_comparison.csv"), index=False)

        # Generate required comparison plots
        self.visualizer.plot_comparison_bars(results_list, "energy_kJ", "Energy (kJ)",
                                            "Estimated Energy Consumption by Algorithm", "energy_comparison.png")
        self.visualizer.plot_comparison_bars(results_list, "distance_m", "Distance (m)",
                                            "Total 3D Path Distance by Algorithm", "distance_comparison.png")
        self.visualizer.plot_comparison_bars(results_list, "turns", "Turn Count",
                                            "Number of Sharp Turns by Algorithm", "turns_comparison.png")
        self.visualizer.plot_comparison_bars(results_list, "coverage_pct", "Coverage (%)",
                                            "Target Field Coverage Rate (%)", "coverage_comparison.png")
        self.visualizer.plot_comparison_bars(results_list, "runtime_s", "Runtime (s)",
                                            "Computational Execution Time", "runtime_comparison.png")

        # Plot 3D Trajectories: Lawnmower and Best ACO-GA-FIS
        self.visualizer.plot_trajectory_3d(grid, paths["Lawnmower"],
                                          "Deterministic Lawnmower Coverage Trajectory",
                                          "lawnmower_path.png", color="#e67e22")
        self.visualizer.plot_trajectory_3d(grid, paths["ACO-GA-FIS"],
                                          "Optimal Wind-Adaptive ACO-GA-FIS 3D Coverage Trajectory",
                                          "best_path_3d.png", color="#27ae60")

        # Convergence comparison
        conv_to_plot = {k: v for k, v in convergence_dict.items() if k != "Lawnmower"}
        self.visualizer.plot_convergence_comparison(conv_to_plot, "convergence_comparison.png")

        return df, paths

    def run_experiment_2_wind_speed(self) -> pd.DataFrame:
        """
        Experiment 2: Wind Speed Sensitivity (0, 6, 12 m/s at 90 deg).
        Compares ACO-GA vs ACO-GA-FIS.
        """
        print("\n--- Running Experiment 2: Wind Speed Sensitivity Sweep ---")
        speeds = [0.0, 6.0, 12.0]
        results_no_fis = []
        results_fis = []
        rows = []

        env = Environment(field_cfg=self.cfg.field, terrain_cfg=self.cfg.terrain, obstacle_cfg=self.cfg.obstacle)
        grid = CoverageGrid(self.cfg.field, env)

        for spd in speeds:
            w_cfg = WindConfig(speed=spd, direction=90.0)
            e_model = EnergyModel(self.cfg.energy, w_cfg)

            ev_no_fis = FitnessEvaluator(grid, e_model, self.cfg.fitness, use_fis=False)
            ev_fis = FitnessEvaluator(grid, e_model, self.cfg.fitness, use_fis=True)

            # 1. ACO-GA (Standard)
            p_no, _, _ = self.run_single_algorithm("ACO-GA", grid, ev_no_fis, seed=self.cfg.seed)
            m_no = ev_fis.evaluate_path(p_no).to_dict()
            results_no_fis.append(m_no["energy_kJ"])

            # 2. ACO-GA-FIS (Wind-Adaptive)
            p_fis, _, _ = self.run_single_algorithm("ACO-GA-FIS", grid, ev_fis, seed=self.cfg.seed)
            m_fis = ev_fis.evaluate_path(p_fis).to_dict()
            results_fis.append(m_fis["energy_kJ"])

            delta_pct = ((m_no["energy_kJ"] - m_fis["energy_kJ"]) / m_no["energy_kJ"]) * 100.0
            print(f"  Wind Speed {spd:4.1f} m/s | ACO-GA: {m_no['energy_kJ']:6.2f} kJ | "
                  f"ACO-GA-FIS: {m_fis['energy_kJ']:6.2f} kJ | Savings: {delta_pct:5.2f}%")

            rows.append({
                "wind_speed_m_s": spd,
                "acoga_energy_kJ": m_no["energy_kJ"],
                "acoga_fis_energy_kJ": m_fis["energy_kJ"],
                "energy_savings_pct": delta_pct
            })

        df = pd.DataFrame(rows)
        df.to_csv(os.path.join(self.results_dir, "wind_speed_results.csv"), index=False)
        self.visualizer.plot_wind_sensitivity(speeds, results_no_fis, results_fis,
                                              "Wind Speed", "m/s",
                                              "Energy Consumption vs Wind Speed (ACO-GA vs ACO-GA-FIS)",
                                              "energy_vs_wind_speed.png")
        return df

    def run_experiment_3_wind_direction(self) -> pd.DataFrame:
        """
        Experiment 3: Wind Direction Sensitivity (0 deg tailwind, 90 deg crosswind, 180 deg headwind).
        Compares ACO-GA vs ACO-GA-FIS at 6 m/s wind.
        """
        print("\n--- Running Experiment 3: Wind Direction Sensitivity Sweep ---")
        directions = [0.0, 90.0, 180.0]
        results_no_fis = []
        results_fis = []
        rows = []

        env = Environment(field_cfg=self.cfg.field, terrain_cfg=self.cfg.terrain, obstacle_cfg=self.cfg.obstacle)
        grid = CoverageGrid(self.cfg.field, env)

        for d in directions:
            w_cfg = WindConfig(speed=6.0, direction=d)
            e_model = EnergyModel(self.cfg.energy, w_cfg)

            ev_no_fis = FitnessEvaluator(grid, e_model, self.cfg.fitness, use_fis=False)
            ev_fis = FitnessEvaluator(grid, e_model, self.cfg.fitness, use_fis=True)

            p_no, _, _ = self.run_single_algorithm("ACO-GA", grid, ev_no_fis, seed=self.cfg.seed)
            m_no = ev_fis.evaluate_path(p_no).to_dict()
            results_no_fis.append(m_no["energy_kJ"])

            p_fis, _, _ = self.run_single_algorithm("ACO-GA-FIS", grid, ev_fis, seed=self.cfg.seed)
            m_fis = ev_fis.evaluate_path(p_fis).to_dict()
            results_fis.append(m_fis["energy_kJ"])

            delta_pct = ((m_no["energy_kJ"] - m_fis["energy_kJ"]) / m_no["energy_kJ"]) * 100.0
            print(f"  Wind Dir {d:5.1f} deg | ACO-GA: {m_no['energy_kJ']:6.2f} kJ | "
                  f"ACO-GA-FIS: {m_fis['energy_kJ']:6.2f} kJ | Savings: {delta_pct:5.2f}%")

            rows.append({
                "wind_direction_deg": d,
                "acoga_energy_kJ": m_no["energy_kJ"],
                "acoga_fis_energy_kJ": m_fis["energy_kJ"],
                "energy_savings_pct": delta_pct
            })

        df = pd.DataFrame(rows)
        df.to_csv(os.path.join(self.results_dir, "wind_direction_results.csv"), index=False)
        self.visualizer.plot_wind_sensitivity(directions, results_no_fis, results_fis,
                                              "Wind Direction", "deg",
                                              "Energy Consumption vs Wind Direction (ACO-GA vs ACO-GA-FIS)",
                                              "energy_vs_wind_direction.png")
        return df

    def run_experiment_4_terrain(self) -> pd.DataFrame:
        """
        Experiment 4: Terrain Complexity Comparison (Flat, Rolling, Complex).
        """
        print("\n--- Running Experiment 4: Terrain Complexity Comparison ---")
        terrains = ["flat", "rolling", "complex"]
        rows = []

        for t_type in terrains:
            t_cfg = TerrainConfig(terrain_type=t_type, clearance_height=20.0)
            env = Environment(field_cfg=self.cfg.field, terrain_cfg=t_cfg, obstacle_cfg=self.cfg.obstacle)
            grid = CoverageGrid(self.cfg.field, env)

            e_model = EnergyModel(self.cfg.energy, self.cfg.wind)
            ev_fis = FitnessEvaluator(grid, e_model, self.cfg.fitness, use_fis=True)
            ev_no_fis = FitnessEvaluator(grid, e_model, self.cfg.fitness, use_fis=False)

            for algo in ["GA", "ACO", "ACO-GA", "ACO-GA-FIS"]:
                ev = ev_fis if algo == "ACO-GA-FIS" else ev_no_fis
                path, _, _ = self.run_single_algorithm(algo, grid, ev, seed=self.cfg.seed)
                m = ev_fis.evaluate_path(path).to_dict()
                rows.append({
                    "terrain": t_type,
                    "algorithm": algo,
                    "energy_kJ": m["energy_kJ"],
                    "distance_m": m["distance_m"],
                    "e_climb_kJ": m["e_climb_kJ"],
                    "cost": m["cost"]
                })
                print(f"  Terrain: {t_type:8s} | Algo: {algo:10s} | Energy: {m['energy_kJ']:6.2f} kJ | "
                      f"Climb: {m['e_climb_kJ']:5.2f} kJ")

        df = pd.DataFrame(rows)
        df.to_csv(os.path.join(self.results_dir, "terrain_results.csv"), index=False)
        return df

    def run_repeated_trials(self) -> Tuple[pd.DataFrame, pd.DataFrame, Dict[str, Any]]:
        """
        Execute repeated trials across seeds (42 + i) for statistical significance testing.
        """
        num_trials = self.cfg.num_trials
        print(f"\n--- Running Repeated Experimental Trials ({num_trials} runs) ---")

        env = Environment(field_cfg=self.cfg.field, terrain_cfg=self.cfg.terrain, obstacle_cfg=self.cfg.obstacle)
        grid = CoverageGrid(self.cfg.field, env)

        e_model = EnergyModel(self.cfg.energy, self.cfg.wind)
        ev_fis = FitnessEvaluator(grid, e_model, self.cfg.fitness, use_fis=True)
        ev_no_fis = FitnessEvaluator(grid, e_model, self.cfg.fitness, use_fis=False)

        algorithms = ["Lawnmower", "GA", "ACO", "ACO-GA", "ACO-GA-FIS"]
        trial_records = []

        for trial in range(num_trials):
            current_seed = self.cfg.seed + trial
            print(f"  Trial {trial + 1}/{num_trials} (seed={current_seed})")

            for algo in algorithms:
                ev = ev_fis if algo == "ACO-GA-FIS" else ev_no_fis
                path, res, _ = self.run_single_algorithm(algo, grid, ev, seed=current_seed)
                m = ev_fis.evaluate_path(path).to_dict()
                m["trial"] = trial + 1
                m["seed"] = current_seed
                m["algorithm"] = algo
                m["runtime_s"] = res["runtime_s"]
                trial_records.append(m)

        trials_df = pd.DataFrame(trial_records)
        trials_df.to_csv(os.path.join(self.results_dir, "repeated_trials.csv"), index=False)

        # Compute summary stats & hypothesis tests
        summary_df = StatisticalAnalyzer.compute_summary_table(trials_df)
        summary_df.to_csv(os.path.join(self.results_dir, "summary_statistics.csv"), index=False)

        hypo_results = StatisticalAnalyzer.run_hypothesis_tests(trials_df)
        return trials_df, summary_df, hypo_results
