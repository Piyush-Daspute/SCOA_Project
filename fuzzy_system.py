"""
fuzzy_system.py - Mamdani Fuzzy Inference System for Wind-Adaptive Energy Cost

Inputs:
  1. Wind Speed: Universe [0, 12] m/s
  2. Relative Wind Direction: Universe [0, 180] degrees (0=tailwind, 90=crosswind, 180=headwind)
Output:
  Environmental Energy Multiplier: Universe [1.0, 2.0]
"""
import numpy as np


def trimf(x: np.ndarray, params: list) -> np.ndarray:
    """Triangular membership function with left/right shoulder handling if a==b or b==c."""
    a, b, c = params
    x = np.asarray(x, dtype=float)
    y = np.zeros_like(x)

    # Left shoulder: if a == b, full membership for x <= a
    if a == b:
        mask_left = x <= a
        y[mask_left] = 1.0
    elif b > a:
        mask_rise = (a < x) & (x < b)
        y[mask_rise] = (x[mask_rise] - a) / (b - a)

    # Point of maximum
    y[x == b] = 1.0

    # Right shoulder: if b == c, full membership for x >= c
    if b == c:
        mask_right = x >= c
        y[mask_right] = 1.0
    elif c > b:
        mask_fall = (b < x) & (x < c)
        y[mask_fall] = (c - x[mask_fall]) / (c - b)

    return np.clip(y, 0.0, 1.0)


class MamdaniWindFIS:
    """
    Mamdani Fuzzy Inference System for computing environmental energy multiplier
    based on wind speed and relative wind direction.
    """

    def __init__(self, resolution: int = 101):
        # Discretized universe of discourse for output z in [1.0, 2.0]
        self.resolution = resolution
        self.z_universe = np.linspace(1.0, 2.0, resolution)

        # Output membership functions over universe
        # Minimal: [1.0, 1.0, 1.3]
        self.mf_out_minimal = trimf(self.z_universe, [1.0, 1.0, 1.3])
        # Moderate: [1.2, 1.5, 1.8]
        self.mf_out_moderate = trimf(self.z_universe, [1.2, 1.5, 1.8])
        # Severe: [1.6, 2.0, 2.0]
        self.mf_out_severe = trimf(self.z_universe, [1.6, 2.0, 2.0])

        # Wind speed MF parameters
        self.p_ws_low = [0.0, 0.0, 4.0]
        self.p_ws_mod = [2.0, 6.0, 10.0]
        self.p_ws_high = [8.0, 12.0, 12.0]

        # Relative direction MF parameters
        self.p_dir_tail = [0.0, 0.0, 60.0]
        self.p_dir_cross = [30.0, 90.0, 150.0]
        self.p_dir_head = [120.0, 180.0, 180.0]

    def evaluate(self, wind_speed: float, rel_direction: float) -> float:
        """
        Evaluate Mamdani FIS for a single (wind_speed, rel_direction) pair.
        Returns: environmental energy multiplier factor in [1.0, 2.0].
        """
        ws = float(np.clip(wind_speed, 0.0, 12.0))
        rd = float(np.clip(rel_direction, 0.0, 180.0))

        # 1. Fuzzification
        mu_ws_low = float(trimf(np.array([ws]), self.p_ws_low)[0])
        mu_ws_mod = float(trimf(np.array([ws]), self.p_ws_mod)[0])
        mu_ws_high = float(trimf(np.array([ws]), self.p_ws_high)[0])

        mu_dir_tail = float(trimf(np.array([rd]), self.p_dir_tail)[0])
        mu_dir_cross = float(trimf(np.array([rd]), self.p_dir_cross)[0])
        mu_dir_head = float(trimf(np.array([rd]), self.p_dir_head)[0])

        # 2. Rule Activation (Min implication for AND)
        # R1: Low Wind & Any Direction -> Minimal
        alpha1 = mu_ws_low
        # R2: Moderate Wind & Tailwind -> Minimal
        alpha2 = min(mu_ws_mod, mu_dir_tail)
        # R3: Moderate Wind & Crosswind -> Moderate
        alpha3 = min(mu_ws_mod, mu_dir_cross)
        # R4: Moderate Wind & Headwind -> Moderate
        alpha4 = min(mu_ws_mod, mu_dir_head)
        # R5: High Wind & Tailwind -> Moderate
        alpha5 = min(mu_ws_high, mu_dir_tail)
        # R6: High Wind & Crosswind -> Severe
        alpha6 = min(mu_ws_high, mu_dir_cross)
        # R7: High Wind & Headwind -> Severe
        alpha7 = min(mu_ws_high, mu_dir_head)

        # 3. Output Implication & Aggregation (Max over all rules)
        # Target: Minimal
        alpha_min = max(alpha1, alpha2)
        # Target: Moderate
        alpha_mod = max(alpha3, alpha4, alpha5)
        # Target: Severe
        alpha_sev = max(alpha6, alpha7)

        # Clip output MFs
        agg_minimal = np.minimum(alpha_min, self.mf_out_minimal)
        agg_moderate = np.minimum(alpha_mod, self.mf_out_moderate)
        agg_severe = np.minimum(alpha_sev, self.mf_out_severe)

        # Aggregated fuzzy output distribution
        aggregated = np.maximum(np.maximum(agg_minimal, agg_moderate), agg_severe)

        # 4. Centroid Defuzzification
        sum_mu = np.sum(aggregated)
        if sum_mu < 1e-9:
            return 1.0  # Fallback to minimal penalty

        centroid = np.sum(self.z_universe * aggregated) / sum_mu
        return float(np.clip(centroid, 1.0, 2.0))

    def evaluate_batch(self, wind_speeds: np.ndarray, rel_directions: np.ndarray) -> np.ndarray:
        """
        Vectorized evaluation over arrays of wind_speeds and rel_directions.
        wind_speeds: shape (N,) or float
        rel_directions: shape (N,) in [0, 180]
        """
        ws = np.clip(np.asarray(wind_speeds, dtype=float), 0.0, 12.0)
        rd = np.clip(np.asarray(rel_directions, dtype=float), 0.0, 180.0)

        if ws.ndim == 0:
            ws = np.full_like(rd, ws)

        # Fuzzification
        mu_ws_low = trimf(ws, self.p_ws_low)
        mu_ws_mod = trimf(ws, self.p_ws_mod)
        mu_ws_high = trimf(ws, self.p_ws_high)

        mu_dir_tail = trimf(rd, self.p_dir_tail)
        mu_dir_cross = trimf(rd, self.p_dir_cross)
        mu_dir_head = trimf(rd, self.p_dir_head)

        # Rule activations
        alpha_min = np.maximum(mu_ws_low, np.minimum(mu_ws_mod, mu_dir_tail))
        alpha_mod = np.maximum(np.maximum(np.minimum(mu_ws_mod, mu_dir_cross),
                                          np.minimum(mu_ws_mod, mu_dir_head)),
                               np.minimum(mu_ws_high, mu_dir_tail))
        alpha_sev = np.maximum(np.minimum(mu_ws_high, mu_dir_cross),
                               np.minimum(mu_ws_high, mu_dir_head))

        # Vectorized defuzzification over resolution
        # Shape: (N, 1) against (1, resolution)
        a_min = alpha_min[:, np.newaxis]
        a_mod = alpha_mod[:, np.newaxis]
        a_sev = alpha_sev[:, np.newaxis]

        mf_min = self.mf_out_minimal[np.newaxis, :]
        mf_mod = self.mf_out_moderate[np.newaxis, :]
        mf_sev = self.mf_out_severe[np.newaxis, :]

        agg = np.maximum(np.maximum(np.minimum(a_min, mf_min),
                                    np.minimum(a_mod, mf_mod)),
                         np.minimum(a_sev, mf_sev))

        sum_mu = np.sum(agg, axis=1)
        # Avoid division by zero
        safe_sum = np.where(sum_mu < 1e-9, 1.0, sum_mu)
        centroids = np.sum(agg * self.z_universe[np.newaxis, :], axis=1) / safe_sum
        centroids = np.where(sum_mu < 1e-9, 1.0, centroids)

        return np.clip(centroids, 1.0, 2.0)
