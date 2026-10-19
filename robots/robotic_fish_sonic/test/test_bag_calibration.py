"""Equal-input sweep fitting and exclusion regression tests."""
from pathlib import Path
import sys
import unittest

import numpy as np

sys.path.insert(0, str(Path(__file__).resolve().parents[1] / 'scripts'))
from calibrate_from_bag import fit, plateaus, validate


def sweep():
    rows = []
    for index, gain in enumerate([.1, .2, .3, .4, .3]):
        for n in range(80):
            common = (index + 1) * .1
            raw = [common, common * .9, common * 1.2]
            rows.append([index * 4 + n * .05, gain, *raw, *raw, 1])
    return np.array(rows)


class SweepTests(unittest.TestCase):
    def test_excludes_settling_and_recovery(self):
        groups, excluded = plateaus(sweep())
        self.assertEqual(len(groups), 4)
        self.assertEqual(len(excluded), 1)
        self.assertIn('decrease', excluded[0]['reason'])
        self.assertAlmostEqual(groups[0][0, 0], 1.)
        self.assertLessEqual(groups[0][-1, 0], 3.70)

    def test_recovers_synthetic_relative_response_on_holdout(self):
        groups, _ = plateaus(sweep())
        calibration = fit([g[:len(g)//2] for g in groups], 'synthetic')
        result = validate(groups, calibration)
        self.assertEqual(result['rejected_holdout_groups'], 0)
        self.assertLess(result['candidate']['p95_spread_v'], 1e-12)
        self.assertGreater(result['recorded']['median_spread_v'], .01)

    def test_rejects_nonmonotone_transfer(self):
        groups, _ = plateaus(sweep())
        groups[1][:, 2] = .01
        with self.assertRaises(ValueError):
            fit(groups, 'invalid')

    def test_excludes_high_voltage_and_short_plateaus(self):
        rows = sweep()
        rows[160:240, 2] = 3.9
        groups, excluded = plateaus(rows)
        self.assertEqual(len(groups), 3)
        self.assertIn('ceiling', excluded[0]['reason'])
        groups, excluded = plateaus(rows[:25])
        self.assertEqual(len(groups), 0)
        self.assertIn('16', excluded[0]['reason'])


if __name__ == '__main__':
    unittest.main()
