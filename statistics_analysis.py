"""
statistics_analysis.py - Statistical Testing and Aggregation for UAV Path Planning
"""
from typing import List, Dict, Any, Tuple
import numpy as np
import pandas as pd
from scipy import stats


class StatisticalAnalyzer:
    """
    Computes descriptive statistics and inferential hypothesis tests:
    - Mean, Standard Deviation, Median, IQR, Min, Max
    - Friedman non-parametric multi-algorithm test
    - Wilcoxon signed-rank test (ACO-GA vs ACO-GA-FIS paired comparison)
    """

    @staticmethod
    def compute_summary_table(repeated_trials_df: pd.DataFrame) -> pd.DataFrame:
        """
        Group repeated runs by algorithm and compute Mean +/- SD, Min, Max.
        """
        summary_rows = []
        algorithms = repeated_trials_df["algorithm"].unique()

        metrics = ["energy_kJ", "distance_m", "turns", "coverage_pct", "runtime_s", "cost"]

        for algo in algorithms:
            algo_data = repeated_trials_df[repeated_trials_df["algorithm"] == algo]
            row: Dict[str, Any] = {"algorithm": algo, "runs": len(algo_data)}

            for m in metrics:
                if m in algo_data.columns:
                    vals = algo_data[m].dropna().values
                    if len(vals) > 0:
                        mean_val = float(np.mean(vals))
                        std_val = float(np.std(vals, ddof=1)) if len(vals) > 1 else 0.0
                        row[f"{m}_mean"] = round(mean_val, 2)
                        row[f"{m}_std"] = round(std_val, 2)
                        row[f"{m}_min"] = round(float(np.min(vals)), 2)
                        row[f"{m}_max"] = round(float(np.max(vals)), 2)
                        row[f"{m}_display"] = f"{mean_val:.2f} ± {std_val:.2f}"

            summary_rows.append(row)

        return pd.DataFrame(summary_rows)

    @staticmethod
    def run_hypothesis_tests(repeated_trials_df: pd.DataFrame) -> Dict[str, Any]:
        """
        Run Friedman test across all algorithms, and Wilcoxon signed-rank test
        specifically for the paired comparison: ACO-GA vs ACO-GA-FIS.
        """
        results: Dict[str, Any] = {}

        # Pivot energy by trial and algorithm
        try:
            pivot_energy = repeated_trials_df.pivot(index="trial", columns="algorithm", values="energy_kJ")
        except Exception:
            return {"status": "insufficient_data_to_pivot"}

        # 1. Multi-algorithm Friedman Test
        algos = list(pivot_energy.columns)
        if len(algos) >= 3 and len(pivot_energy) >= 3:
            samples = [pivot_energy[col].values for col in algos]
            try:
                stat_f, p_f = stats.friedmanchisquare(*samples)
                results["friedman_test"] = {
                    "statistic": float(stat_f),
                    "p_value": float(p_f),
                    "significant_at_05": bool(p_f < 0.05),
                    "note": "Non-parametric repeated measures test across all compared algorithms."
                }
            except Exception as e:
                results["friedman_test"] = {"error": str(e)}
        else:
            results["friedman_test"] = {
                "note": "Fewer than 3 trials or algorithms; descriptive statistics reported instead."
            }

        # 2. Focused Paired Comparison: ACO-GA vs ACO-GA-FIS
        if "ACO-GA" in pivot_energy.columns and "ACO-GA-FIS" in pivot_energy.columns:
            s_no_fis = pivot_energy["ACO-GA"].values
            s_fis = pivot_energy["ACO-GA-FIS"].values
            diff = s_no_fis - s_fis

            if len(diff) >= 5 and np.any(diff != 0):
                try:
                    stat_w, p_w = stats.wilcoxon(s_no_fis, s_fis)
                    results["wilcoxon_test"] = {
                        "comparison": "ACO-GA vs ACO-GA-FIS",
                        "statistic": float(stat_w),
                        "p_value": float(p_w),
                        "significant_at_05": bool(p_w < 0.05),
                        "mean_energy_reduction_pct": float(np.mean(diff / s_no_fis) * 100.0)
                    }
                except Exception as e:
                    results["wilcoxon_test"] = {"error": str(e)}
            else:
                mean_red = float(np.mean(diff / np.maximum(1e-6, s_no_fis)) * 100.0) if len(diff) > 0 else 0.0
                results["wilcoxon_test"] = {
                    "note": "Sample size too small (<5) or zero variance for Wilcoxon signed-rank test; reporting mean difference.",
                    "mean_energy_reduction_pct": round(mean_red, 2)
                }

        return results
