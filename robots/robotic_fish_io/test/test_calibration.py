#!/usr/bin/env python3
"""Regression tests for the packaged independent ADC calibration curves."""

import copy
from pathlib import Path
import sys
import unittest


PACKAGE_ROOT = Path(__file__).resolve().parents[1]
sys.path.insert(0, str(PACKAGE_ROOT / "src"))

from robotic_fish_io.adc_calibration import AdcCalibration  # noqa: E402


CONFIG_PATH = (
    PACKAGE_ROOT
    / "config"
    / "calibration"
    / "adc_independent_transfer_20260901_171408_calbri05_raw.json"
)


class CalibrationTests(unittest.TestCase):
    @classmethod
    def setUpClass(cls):
        cls.calibration = AdcCalibration.load(CONFIG_PATH)

    def test_packaged_curves_are_complete_and_dac_independent(self):
        self.assertEqual(
            self.calibration.calibration_id,
            "adc_independent_transfer_20260901_171408_calbri05_raw",
        )
        self.assertEqual([len(curve) for curve in self.calibration.curves], [121] * 3)
        self.assertFalse(
            self.calibration.document["processing"]["runtime_dac_dependency"]
        )
        for raw_nodes in self.calibration.raw_nodes:
            self.assertTrue(all(b > a for a, b in zip(raw_nodes, raw_nodes[1:])))

    def test_exact_nodes_and_piecewise_linear_interpolation(self):
        for index in (0, 1, 60, 120):
            raw = [
                self.calibration.curves[channel][index]["raw_voltage_v"]
                for channel in range(3)
            ]
            corrected, gains = self.calibration.apply(raw)
            expected = [
                self.calibration.curves[channel][index]["reference_voltage_v"]
                for channel in range(3)
            ]
            for channel in range(3):
                self.assertAlmostEqual(corrected[channel], expected[channel])
                self.assertAlmostEqual(gains[channel], expected[channel] / raw[channel])

        midpoint_raw = [
            (curve[0]["raw_voltage_v"] + curve[1]["raw_voltage_v"]) / 2.0
            for curve in self.calibration.curves
        ]
        midpoint, _ = self.calibration.apply(midpoint_raw)
        for channel, curve in enumerate(self.calibration.curves):
            expected = (
                curve[0]["reference_voltage_v"]
                + curve[1]["reference_voltage_v"]
            ) / 2.0
            self.assertAlmostEqual(midpoint[channel], expected)

    def test_outside_range_rejects_the_complete_group(self):
        valid = [item[0] for item in self.calibration.input_ranges]
        valid[0] -= self.calibration.endpoint_tolerance_v + 0.001
        with self.assertRaises(ValueError):
            self.calibration.apply(valid)

        valid = [item[1] for item in self.calibration.input_ranges]
        valid[2] += self.calibration.endpoint_tolerance_v + 0.001
        with self.assertRaises(ValueError):
            self.calibration.apply(valid)

    def test_api_does_not_accept_dac_voltage(self):
        values = [item[0] for item in self.calibration.input_ranges]
        with self.assertRaises(TypeError):
            self.calibration.apply(values, 0.5)

    def test_malformed_curves_are_rejected_as_a_whole(self):
        cases = []
        reordered = copy.deepcopy(self.calibration.document)
        reordered["curves"][0]["nodes"][1], reordered["curves"][0]["nodes"][2] = (
            reordered["curves"][0]["nodes"][2],
            reordered["curves"][0]["nodes"][1],
        )
        cases.append(reordered)

        wrong_channels = copy.deepcopy(self.calibration.document)
        wrong_channels["channel_order"] = ["ADC1", "ADC0", "ADC2"]
        cases.append(wrong_channels)

        dac_dependent = copy.deepcopy(self.calibration.document)
        dac_dependent["processing"]["runtime_dac_dependency"] = True
        cases.append(dac_dependent)

        missing_curve = copy.deepcopy(self.calibration.document)
        del missing_curve["curves"][2]
        cases.append(missing_curve)

        for document in cases:
            with self.subTest(channel_order=document["channel_order"]):
                with self.assertRaises(ValueError):
                    AdcCalibration(document)


if __name__ == "__main__":
    unittest.main()
