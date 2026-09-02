#!/usr/bin/env python3
"""Regression tests for closed-loop AGC decisions."""

from pathlib import Path
import sys
import unittest


PACKAGE_ROOT = Path(__file__).resolve().parents[1]
sys.path.insert(0, str(PACKAGE_ROOT / "src"))

from robotic_fish_io.agc_controller import AgcController, GainController  # noqa: E402


def make_controller(**overrides):
    parameters = {
        "target_min_v": 2.5,
        "target_max_v": 3.0,
        "step_v": 0.01,
        "interval_s": 0.5,
        "consecutive_samples": 3,
        "dac_min_v": 0.0,
        "dac_max_v": 5.0,
    }
    parameters.update(overrides)
    return AgcController(**parameters)


class AgcControllerTests(unittest.TestCase):
    def test_three_low_groups_increase_dac_by_one_step(self):
        controller = make_controller()
        low = [2.0, 2.1, 2.2]
        self.assertIsNone(controller.observe(low, 1.00, 1.0))
        self.assertIsNone(controller.observe(low, 1.00, 1.1))
        self.assertEqual(controller.observe(low, 1.00, 1.2), 1.01)
        self.assertEqual(controller.last_action, "increase")
        self.assertEqual(controller.current_max_raw_v, 2.2)
        self.assertEqual(controller.below_count, 0)

    def test_three_high_groups_decrease_dac_by_one_step(self):
        controller = make_controller()
        high = [2.8, 3.2, 2.9]
        self.assertIsNone(controller.observe(high, 1.00, 1.0))
        self.assertIsNone(controller.observe(high, 1.00, 1.1))
        self.assertEqual(controller.observe(high, 1.00, 1.2), 0.99)
        self.assertEqual(controller.last_action, "decrease")
        self.assertEqual(controller.current_max_raw_v, 3.2)

    def test_in_range_and_direction_change_reset_consecutive_counts(self):
        controller = make_controller()
        low = [2.0, 2.1, 2.2]
        high = [3.1, 2.9, 2.8]
        in_range = [2.0, 2.6, 2.4]
        controller.observe(low, 1.0, 1.0)
        controller.observe(low, 1.0, 1.1)
        self.assertIsNone(controller.observe(high, 1.0, 1.2))
        self.assertEqual(controller.below_count, 0)
        self.assertEqual(controller.above_count, 1)
        self.assertIsNone(controller.observe(in_range, 1.0, 1.3))
        self.assertEqual(controller.last_action, "in_range")
        self.assertEqual(controller.below_count, 0)
        self.assertEqual(controller.above_count, 0)

    def test_minimum_adjustment_interval_is_enforced(self):
        controller = make_controller(consecutive_samples=1, interval_s=0.5)
        low = [2.0, 2.1, 2.2]
        self.assertEqual(controller.observe(low, 1.00, 1.0), 1.01)
        self.assertIsNone(controller.observe(low, 1.01, 1.1))
        self.assertIsNone(controller.observe(low, 1.01, 1.4))
        self.assertEqual(controller.observe(low, 1.01, 1.5), 1.02)

    def test_dac_bounds_stop_further_adjustment(self):
        low = [2.0, 2.1, 2.2]
        high = [3.1, 3.2, 3.3]
        controller = make_controller(consecutive_samples=1)
        self.assertIsNone(controller.observe(low, 5.0, 1.0))
        self.assertEqual(controller.last_action, "dac_max")
        self.assertIsNone(controller.observe(high, 0.0, 2.0))
        self.assertEqual(controller.last_action, "dac_min")

    def test_invalid_samples_reset_counts(self):
        controller = make_controller()
        controller.observe([2.0, 2.1, 2.2], 1.0, 1.0)
        with self.assertRaises(ValueError):
            controller.observe([2.0, 2.1], 1.0, 1.1)
        self.assertEqual(controller.below_count, 0)
        with self.assertRaises(ValueError):
            controller.observe([2.0, float("nan"), 2.2], 1.0, 1.2)

    def test_configuration_validation(self):
        invalid = [
            {"target_min_v": 3.0, "target_max_v": 2.5},
            {"step_v": 0.0},
            {"interval_s": 0.01},
            {"consecutive_samples": 0},
            {"consecutive_samples": 1.5},
            {"dac_min_v": 5.0, "dac_max_v": 0.0},
        ]
        for overrides in invalid:
            with self.subTest(overrides=overrides), self.assertRaises(ValueError):
                make_controller(**overrides)


class GainControllerTests(unittest.TestCase):
    def make_gain_controller(self, **overrides):
        parameters = {
            "mode": "fixed",
            "fixed_target_v": 1.0,
            "target_min_v": 2.5,
            "target_max_v": 3.0,
            "step_v": 0.01,
            "interval_s": 0.5,
            "consecutive_samples": 3,
            "dac_min_v": 0.0,
            "dac_max_v": 5.0,
            "safety_limit_v": 3.5,
            "safety_recovery_v": 3.3,
            "safety_step_v": 0.1,
            "safety_interval_s": 0.1,
            "fixed_ramp_step_v": 0.05,
            "fixed_ramp_interval_s": 0.1,
            "recovery_settle_s": 0.2,
            "max_normal_step_v": 0.1,
        }
        parameters.update(overrides)
        return GainController(**parameters)

    def test_fixed_mode_ramps_and_holds(self):
        controller = self.make_gain_controller()
        safe = [1.0, 1.1, 1.2]
        self.assertEqual(controller.observe(safe, 0.0, 1.0), 0.05)
        self.assertIsNone(controller.observe(safe, 0.05, 1.05))
        self.assertEqual(controller.last_action, "fixed_waiting")
        self.assertEqual(controller.observe(safe, 0.05, 1.1), 0.10)
        self.assertIsNone(controller.observe(safe, 1.0, 2.0))
        self.assertEqual(controller.last_action, "fixed_holding")

    def test_exact_safety_limit_immediately_overrides_normal_interval(self):
        controller = self.make_gain_controller(mode="closed_loop", consecutive_samples=1)
        controller.observe([1.0, 1.1, 1.2], 1.0, 1.0)
        self.assertEqual(controller.observe([3.5, 3.0, 3.0], 1.01, 1.01), 0.91)
        self.assertTrue(controller.safety_active)
        self.assertEqual(controller.last_action, "safety_decreasing")

    def test_safety_decreases_until_recovery_then_latches_fixed_mode(self):
        controller = self.make_gain_controller()
        high = [3.6, 3.2, 3.1]
        self.assertEqual(controller.observe(high, 1.0, 1.0), 0.9)
        self.assertIsNone(controller.observe(high, 0.9, 1.05))
        self.assertEqual(controller.observe(high, 0.9, 1.1), 0.8)
        self.assertIsNone(controller.observe([3.3, 3.0, 2.9], 0.8, 1.2))
        self.assertFalse(controller.safety_active)
        self.assertTrue(controller.fixed_limited)
        self.assertEqual(controller.last_action, "fixed_limited")
        self.assertIsNone(controller.observe([1.0, 1.1, 1.2], 0.8, 2.0))
        controller.reset_safety()
        self.assertEqual(controller.observe([1.0, 1.1, 1.2], 0.8, 2.1), 0.85)

    def test_closed_loop_settles_after_safety_recovery(self):
        controller = self.make_gain_controller(
            mode="closed_loop", consecutive_samples=1, recovery_settle_s=0.2
        )
        self.assertEqual(controller.observe([3.6, 3.0, 3.0], 1.0, 1.0), 0.9)
        self.assertIsNone(controller.observe([3.3, 3.0, 3.0], 0.9, 1.1))
        self.assertEqual(controller.last_action, "settling")
        self.assertIsNone(controller.observe([1.0, 1.1, 1.2], 0.9, 1.2))
        self.assertEqual(controller.observe([1.0, 1.1, 1.2], 0.9, 1.3), 0.91)

    def test_off_mode_only_acts_for_safety(self):
        controller = self.make_gain_controller(mode="off")
        self.assertIsNone(controller.observe([1.0, 1.1, 1.2], 1.0, 1.0))
        self.assertEqual(controller.last_action, "off")
        self.assertEqual(controller.observe([3.5, 3.0, 3.0], 1.0, 1.1), 0.9)

    def test_overrange_at_dac_min_is_latched_as_error_state(self):
        controller = self.make_gain_controller(mode="closed_loop")
        self.assertIsNone(controller.observe([3.6, 3.0, 3.0], 0.0, 1.0))
        self.assertTrue(controller.safety_active)
        self.assertEqual(controller.last_action, "adc_overrange_at_dac_min")

    def test_gain_configuration_rejects_unsafe_values(self):
        invalid = [
            {"mode": "automatic"},
            {"fixed_target_v": 5.01},
            {"step_v": 0.11},
            {"fixed_ramp_step_v": 0.11},
            {"safety_limit_v": 3.3, "safety_recovery_v": 3.3},
            {"safety_limit_v": 4.097},
            {"target_max_v": 3.5},
        ]
        for overrides in invalid:
            with self.subTest(overrides=overrides), self.assertRaises(ValueError):
                self.make_gain_controller(**overrides)


if __name__ == "__main__":
    unittest.main()
