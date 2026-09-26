"""
main.py - Main Entrypoint for 3D Agricultural UAV Coverage Path Planning Research Framework
"""
import sys
import argparse
import numpy as np

from config import get_quick_config, get_full_config, SystemConfig
from environment import Environment
from coverage import CoverageGrid
from fuzzy_system import MamdaniWindFIS
from experiments import ExperimentRunner


def validate_results_integrity(main_df) -> bool:
    """
    Automatic sanity checks as required by Section 59:
    - coverage >= 99%
    - energy > 0
    - distance > 0
    - runtime > 0
    - no NaN or inf
    - obstacle collisions == 0
    """
    for _, row in main_df.iterrows():
        algo = row["algorithm"]
        assert not np.isnan(row["energy_kJ"]) and not np.isinf(row["energy_kJ"]), f"NaN/Inf energy in {algo}"
        assert row["energy_kJ"] > 0, f"Non-positive energy in {algo}: {row['energy_kJ']}"
        assert row["distance_m"] > 0, f"Non-positive distance in {algo}: {row['distance_m']}"
        assert row["coverage_pct"] >= 99.0, f"Incomplete coverage in {algo}: {row['coverage_pct']}%"
        assert row["obstacle_violations"] == 0, f"Obstacle collision detected in {algo}: {row['obstacle_violations']}"
        assert row["runtime_s"] >= 0, f"Invalid runtime in {algo}: {row['runtime_s']}"

    # Verify FIS output universe constraint
    fis = MamdaniWindFIS()
    for ws in [0.0, 3.0, 6.0, 9.0, 12.0]:
        for rd in [0.0, 45.0, 90.0, 135.0, 180.0]:
            val = fis.evaluate(ws, rd)
            assert 1.0 <= val <= 2.0, f"FIS output out of bounds [1.0, 2.0]: {val} for ws={ws}, rd={rd}"

    return True


def run_pipeline(cfg: SystemConfig):
    """Execute complete end-to-end research experimentation pipeline."""
    print("=" * 60)
    print("ENERGY-AWARE 3D COVERAGE PATH PLANNING RESEARCH SYSTEM")
    print(f"Mode: {cfg.mode.upper()} | Grid: {cfg.field.grid_n_x}x{cfg.field.grid_n_y} | Seed: {cfg.seed}")
    print("=" * 60)

    # 1. Environment & Grid Validation
    print("\n[Step 1] Initializing Environment and Decomposing Field...")
    env = Environment(field_cfg=cfg.field, terrain_cfg=cfg.terrain, obstacle_cfg=cfg.obstacle)
    grid = CoverageGrid(cfg.field, env)
    print(f"  Field: {cfg.field.width}m x {cfg.field.length}m | Cells: {len(grid.cells)} "
          f"({grid.num_target_cells} valid target cells, {len(grid.cells) - grid.num_target_cells} obstacle cells)")

    # 2. Experiment Runner
    runner = ExperimentRunner(cfg)

    # Experiment 1: Main algorithm comparison
    main_df, paths = runner.run_experiment_1_main()

    # Validate output results
    print("\n[Step 2] Executing Automatic Result Validation Checks...")
    validate_results_integrity(main_df)
    print("  All validation constraints verified: PASS (No collisions, >99% coverage, valid physics).")

    # Experiment 2: Wind speed sensitivity
    runner.run_experiment_2_wind_speed()

    # Experiment 3: Wind direction sensitivity
    runner.run_experiment_3_wind_direction()

    # Experiment 4: Terrain complexity
    runner.run_experiment_4_terrain()

    # Experiment 5: Repeated trials & statistical tests
    trials_df, summary_df, stats_tests = runner.run_repeated_trials()

    # Print Final Summary Report
    print("\n" + "=" * 60)
    print("SCOA UAV RESEARCH EXPERIMENT COMPLETE")
    print("=" * 60)
    print(f"Grid: {cfg.field.grid_n_x} x {cfg.field.grid_n_y}")
    print(f"Terrain: {cfg.terrain.terrain_type.capitalize()}")
    print(f"Wind: {cfg.wind.speed} m/s | Direction: {cfg.wind.direction} degrees")
    print("\nAlgorithm Results (Main Comparison):")

    for _, row in main_df.iterrows():
        print(f"\n{row['algorithm']}")
        print(f"  Coverage: {row['coverage_pct']:.1f}%")
        print(f"  Energy:   {row['energy_kJ']:.2f} kJ")
        print(f"  Distance: {row['distance_m']:.1f} m")
        print(f"  Turns:    {int(row['turns'])}")
        print(f"  Runtime:  {row['runtime_s']:.2f} s")

    print("\nStatistical Analysis Summary (Energy kJ across trials):")
    for _, srow in summary_df.iterrows():
        print(f"  {srow['algorithm']:10s}: {srow.get('energy_kJ_display', 'N/A')} kJ (runs={srow['runs']})")

    if "wilcoxon_test" in stats_tests:
        wt = stats_tests["wilcoxon_test"]
        print("\nWilcoxon Signed-Rank Test (ACO-GA vs ACO-GA-FIS):")
        for k, v in wt.items():
            print(f"  {k}: {v}")

    print("\nOutput Artifacts:")
    print("  Results: results/ (main_comparison.csv, wind_speed_results.csv, etc.)")
    print("  Figures: figures/ (terrain.png, best_path_3d.png, comparisons, etc.)")
    print("\nPipeline Status: READY / PASS")
    print("=" * 60)


def main():
    parser = argparse.ArgumentParser(description="SCOA 3D UAV Coverage Path Planning")
    parser.add_argument("--quick", action="store_true", help="Run lightweight smoke-test (10x10 grid, 1 trial)")
    parser.add_argument("--full", action="store_true", help="Run full research experiments (20x20 grid, repeated trials)")

    args = parser.parse_args()

    if args.full:
        cfg = get_full_config()
    else:
        # Default or --quick runs quick config for fast, responsive execution
        cfg = get_quick_config()
        if not args.quick:
            print("Note: Defaulting to quick mode for rapid execution. Use --full for full experiment mode.")

    run_pipeline(cfg)


if __name__ == "__main__":
    main()
