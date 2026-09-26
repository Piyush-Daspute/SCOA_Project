"""
tests/test_fuzzy.py - Unit Tests for Mamdani Fuzzy Inference System
"""
import pytest
import numpy as np
from fuzzy_system import MamdaniWindFIS, trimf


def test_trimf():
    # Symmetric triangle [0, 5, 10]
    p = [0.0, 5.0, 10.0]
    assert trimf(np.array([0.0]), p)[0] == 0.0
    assert trimf(np.array([5.0]), p)[0] == 1.0
    assert trimf(np.array([10.0]), p)[0] == 0.0
    assert pytest.approx(trimf(np.array([2.5]), p)[0], 0.01) == 0.5

    # Left shoulder [0, 0, 4]
    p_left = [0.0, 0.0, 4.0]
    assert trimf(np.array([0.0]), p_left)[0] == 1.0
    assert trimf(np.array([-1.0]), p_left)[0] == 1.0
    assert trimf(np.array([4.0]), p_left)[0] == 0.0


def test_fis_output_bounds():
    fis = MamdaniWindFIS()
    for ws in [0.0, 2.0, 6.0, 10.0, 12.0]:
        for rd in [0.0, 30.0, 90.0, 150.0, 180.0]:
            out = fis.evaluate(ws, rd)
            assert 1.0 <= out <= 2.0, f"Out of bounds: {out} for ws={ws}, rd={rd}"


def test_fis_rule_scenarios():
    # Section 17 explicit test requirements:
    # 1. low wind + tailwind
    # 2. moderate wind + crosswind
    # 3. high wind + headwind
    fis = MamdaniWindFIS()

    out_low_tail = fis.evaluate(0.0, 0.0)
    out_mod_cross = fis.evaluate(6.0, 90.0)
    out_high_head = fis.evaluate(12.0, 180.0)

    # All must be within [1.0, 2.0]
    assert 1.0 <= out_low_tail <= 2.0
    assert 1.0 <= out_mod_cross <= 2.0
    assert 1.0 <= out_high_head <= 2.0

    # Monotonicity test: adverse conditions strictly yield higher energy multiplier
    assert out_low_tail < out_mod_cross < out_high_head
    assert out_low_tail <= 1.2
    assert 1.4 <= out_mod_cross <= 1.6
    assert 1.7 <= out_high_head <= 2.0


def test_fis_batch_evaluation():
    fis = MamdaniWindFIS()
    ws_arr = np.array([0.0, 6.0, 12.0])
    rd_arr = np.array([0.0, 90.0, 180.0])

    batch_outs = fis.evaluate_batch(ws_arr, rd_arr)
    single_outs = np.array([fis.evaluate(ws, rd) for ws, rd in zip(ws_arr, rd_arr)])

    np.testing.assert_allclose(batch_outs, single_outs, atol=1e-3)
