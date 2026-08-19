#!/usr/bin/env python3
"""Regression tests for closed-loop AGC decisions."""

from pathlib import Path
import sys
import unittest


PACKAGE_ROOT = Path(__file__).resolve().parents[1]
sys.path.insert(0, str(PACKAGE_ROOT / "src"))

from robotic_fish_io.agc_controller import AgcController  # noqa: E402


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


if __name__ == "__main__":
    unittest.main()
