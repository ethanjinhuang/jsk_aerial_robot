"""Equal-input sweep fitting and exclusion regression tests."""
from pathlib import Path
import sys
import tempfile
from types import SimpleNamespace
import unittest
from unittest import mock

import numpy as np

sys.path.insert(0, str(Path(__file__).resolve().parents[1] / 'scripts'))
from calibrate_from_bag import (fit, plateaus, read_sweep_bag, require_holdout,
                               require_raw_audit, safety_limit, validate)
from rebuild_new_circuit_calibration import assess, require_levels


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

    def test_measured_boundary_nodes_preserve_variation_and_bounded_lookup(self):
        groups, _ = plateaus(sweep())
        for group in (groups[0], groups[-1]):
            group[0, 2:5] -= .004
            group[-1, 2:5] += .008
        legacy = fit(groups, 'legacy')
        supported = fit(groups, 'supported', include_measured_endpoints=True)
        self.assertEqual([len(c) for c in supported.curves], [len(groups)+2]*3)
        self.assertEqual([len(c) for c in legacy.curves], [len(groups)]*3)
        within = [nodes[-1]+.004*scale for nodes, scale in
                  zip(legacy.raw_nodes, (1., .9, 1.2))]
        with self.assertRaises(ValueError):
            legacy.apply(within)
        low, _ = supported.apply(within)
        high, _ = supported.apply([v+.001 for v in within])
        self.assertTrue(all(b>a for a,b in zip(low,high)))
        self.assertAlmostEqual(max(low)-min(low), 0., places=12)
        too_high = [r[1]+supported.endpoint_tolerance_v+.001 for r in supported.input_ranges]
        with self.assertRaises(ValueError):
            supported.apply(too_high)
        for curve in supported.document['curves']:
            self.assertTrue(curve['nodes'][-1]['reference_estimated'])
            self.assertEqual(curve['measured_support']['source'], 'first_and_last_settled_training_plateaus')

    def test_boundary_support_uses_training_only_and_rejects_saturation(self):
        groups, _ = plateaus(sweep())
        training = [g[:len(g)//2].copy() for g in groups]
        # A later observation must not enlarge support during holdout fitting.
        groups[-1][-1, 2:5] += .04
        supported = fit(training, 'trial', include_measured_endpoints=True)
        self.assertLess(supported.input_ranges[0][1], groups[-1][-1,2])
        training[-1][-1,2] = 4.095875
        with self.assertRaises(ValueError):
            fit(training, 'saturated', include_measured_endpoints=True)

    def test_excludes_high_voltage_and_short_plateaus(self):
        rows = sweep()
        rows[160:240, 2] = 4.0
        groups, excluded = plateaus(rows)
        self.assertEqual(len(groups), 3)
        self.assertIn('ceiling', excluded[0]['reason'])
        groups, excluded = plateaus(rows[:25])
        self.assertEqual(len(groups), 0)
        self.assertIn('16', excluded[0]['reason'])

    def test_uses_package_safety_threshold_when_bag_has_no_config(self):
        value, source = safety_limit()
        self.assertEqual(value, 4.0)
        self.assertIn('package_config', source)

    def test_recorded_threshold_takes_precedence_over_default(self):
        value, source = safety_limit([{'safety_limit_v': 3.5}])
        self.assertEqual(value, 3.5)
        self.assertEqual(source, 'recorded_gain_control_config')
        rows = sweep()
        rows[160:240, 2] = 3.5
        groups, excluded = plateaus(rows, maximum_v=value)
        self.assertEqual(len(groups), 3)
        self.assertIn('ADC safety', excluded[0]['reason'])

    def test_rejects_changed_or_conflicting_recorded_threshold(self):
        with self.assertRaises(ValueError):
            safety_limit([{'safety_limit_v': 3.5}, {'safety_limit_v': 4.0}])
        with self.assertRaises(ValueError):
            safety_limit([{'safety_limit_v': 3.5}], explicit_v=4.0)
        self.assertEqual(safety_limit([{'safety_limit_v': 3.5}], explicit_v=3.5)[0], 3.5)

    def test_explicit_fallback_and_invalid_thresholds(self):
        self.assertEqual(safety_limit(explicit_v=3.5)[0], 3.5)
        for value in (0, -1, float('nan'), float('inf'), 4.1, True):
            with self.subTest(value=value), self.assertRaises(ValueError):
                safety_limit(explicit_v=value)

    def test_installed_tool_finds_config_in_package_share(self):
        package = Path(__file__).resolve().parents[1]
        rospkg = SimpleNamespace(RosPack=lambda: SimpleNamespace(get_path=lambda name: str(package)))
        with tempfile.TemporaryDirectory() as directory:
            installed_script = Path(directory) / 'lib/robotic_fish_sonic/calibrate_from_bag.py'
            with mock.patch('calibrate_from_bag.__file__', str(installed_script)), \
                    mock.patch.dict(sys.modules, {'rospkg': rospkg}):
                self.assertEqual(safety_limit()[0], 4.)

    def test_raw_audit_precedes_invalid_feedback_filtering(self):
        def message(stamp, peak, feedback, code):
            return SimpleNamespace(samples=[SimpleNamespace(
                channel_id=c, timestamp=SimpleNamespace(to_sec=lambda: stamp),
                volt_raw=peak, volt_cali=peak, adc_code=code, dac_volt=.1,
                status_dac_feedback=feedback, status_cali=False) for c in range(3)])

        good = message(1., .1, True, 800)
        for peak, code in ((4.05, 32400), (.1, 32767), (float('nan'), 800)):
            with self.subTest(peak=peak, code=code):
                bag = mock.MagicMock()
                bag.__enter__.return_value.read_messages.return_value = [
                    ('/robotic_fish/adc/samples', good, None),
                    ('/robotic_fish/adc/samples', message(2., peak, False, code), None)]
                with mock.patch.dict(sys.modules, {'rosbag': SimpleNamespace(Bag=lambda path: bag)}):
                    rows, _, _, _, audit = read_sweep_bag('synthetic.bag')
                self.assertEqual(len(rows), 1)
                self.assertEqual(audit['total_groups'], 2)
                self.assertEqual(audit['invalid_feedback_or_data_groups'], 1)
                with self.assertRaises(ValueError):
                    require_raw_audit(audit, 4.)

    def test_holdout_rejection_prevents_candidate_generation(self):
        groups, _ = plateaus(sweep())
        training = [g[:len(g)//2].copy() for g in groups]
        calibration = fit(training, 'trial')
        for group in groups:
            group[len(group)//2:, 2:5] += 1.
        rejected = validate(groups, calibration)
        self.assertEqual(rejected['candidate']['count'], 0)
        with self.assertRaisesRegex(ValueError, 'coverage'):
            require_holdout(rejected)
        low_coverage = dict(candidate=dict(count=98, p95_spread_v=.01),
                            raw=dict(p95_spread_v=.02), rejected_holdout_groups=2)
        with self.assertRaisesRegex(ValueError, 'coverage'):
            require_holdout(low_coverage)
        worse = dict(candidate=dict(count=100, p95_spread_v=.03),
                     raw=dict(p95_spread_v=.02), rejected_holdout_groups=0)
        with self.assertRaisesRegex(ValueError, 'worsened'):
            require_holdout(worse)

    def test_independent_validation_gates_each_plateau_and_reports_rejections(self):
        groups, _ = plateaus(sweep())
        calibration = fit(groups, 'trial')
        passing = assess(groups, calibration)
        self.assertTrue(passing['recommended_for_deployment'])
        # Corrupt 10% of one plateau, leaving the remaining observations accurate.
        groups[-1][:6, 2:5] += 1.
        with self.assertRaisesRegex(ValueError, 'coverage'):
            assess(groups, calibration)
        rejected = assess(groups, calibration, enforce=False)
        self.assertEqual(rejected['status'], 'rejected')
        self.assertFalse(rejected['recommended_for_deployment'])
        self.assertEqual(rejected['rejected_stable_groups'], 6)
        self.assertEqual(rejected['per_plateau'][-1]['total_stable_groups'], len(groups[-1]))

    def test_rebuild_requires_all_planned_levels(self):
        groups, _ = plateaus(sweep())
        require_levels(groups, [.1, .2, .3, .4], 'training')
        with self.assertRaises(ValueError):
            require_levels(groups[:-1], [.1, .2, .3, .4], 'training')
        with self.assertRaises(ValueError):
            require_levels(groups, [.1, .2, .3, .41], 'training')

    def test_rejects_entire_plateau_for_overrange_during_settling(self):
        rows = sweep()
        rows[0, 2] = 4.0
        groups, excluded = plateaus(rows, maximum_v=4.0)
        self.assertEqual(len(groups), 3)
        self.assertEqual(excluded[0]['dac_v'], .1)
        self.assertIn('ADC safety', excluded[0]['reason'])

    def test_experimental_dac_ceiling_does_not_limit_other_experiments(self):
        rows = sweep()
        rows[:80, 1] = np.float32(1.30)
        rows[80:160, 1] = 1.31
        rows[160:240, 1] = 1.32
        rows[240:, 1] = 1.33
        groups, excluded = plateaus(rows, dac_max_v=1.30)
        self.assertEqual(len(groups), 1)
        self.assertTrue(all('DAC ceiling' in g['reason'] for g in excluded))
        groups, _ = plateaus(rows)
        self.assertEqual(len(groups), 4)
        for value in (float('nan'), -1, 5.1):
            with self.subTest(value=value), self.assertRaises(ValueError):
                plateaus(rows, dac_max_v=value)


if __name__ == '__main__':
    unittest.main()
