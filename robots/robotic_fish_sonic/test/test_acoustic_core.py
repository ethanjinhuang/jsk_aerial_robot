import json
from pathlib import Path
import tempfile
from types import SimpleNamespace as NS
import unittest
from robotic_fish_sonic.acoustic_core import CLASSES, FEATURES, Model, Estimator, Status, adapt


def batch(t, values=(1., 2., 1.), old=False, calibrated=True):
    samples = []
    for channel, value in enumerate(values):
        stamp = NS(sec=int(t), nanosec=int((t-int(t))*1e9))
        if old:
            samples.append(NS(channel=channel, header=NS(stamp=stamp), calibrated_voltage=value,
                              calibration_applied=calibrated, calibration_id='test', raw=100,
                              gain_control_voltage_valid=True, gain_control_voltage=.9))
        else:
            samples.append(NS(channel_id=channel, timestamp=stamp, volt_cali=value,
                              status_cali=calibrated, cali_id='test', adc_code=100,
                              status_dac_feedback=True, dac_volt=.9))
    return NS(samples=samples)


class CoreTests(unittest.TestCase):
    def setUp(self):
        self.temp = tempfile.TemporaryDirectory()
        path = Path(self.temp.name) / 'fixture.json'
        # Synthetic fixture only: prescribed FRD probabilities .1,.2,.3,.4.
        import math
        path.write_text(json.dumps(dict(classes=CLASSES, features=FEATURES, frame='FRD',
            weights=[[0.]*4]*4, mean=[0.]*4, scale=[1.]*4,
            bias=[math.log(p) for p in [.1,.2,.3,.4]], temperature=1., window_s=.5,
            model_id='SYNTHETIC_TEST_ONLY')))
        self.model = Model(path)

    def tearDown(self):
        self.temp.cleanup()

    def test_frd_marginals(self):
        r = Estimator(self.model).push(batch(10.), 10.)
        p = r['probabilities']
        self.assertAlmostEqual(p[2]+p[3], .7)
        self.assertAlmostEqual(p[0]+p[1], .3)
        self.assertTrue(r['updated'])

    def test_fallback_keeps_new_result(self):
        r = Estimator(self.model).push(batch(10., calibrated=False), 10.)
        self.assertTrue(r['updated'])
        self.assertTrue(r['status'] & Status.CALIBRATION_FALLBACK)

    def test_hold_does_not_retimestamp(self):
        e = Estimator(self.model)
        first = e.push(batch(10.), 10.)
        held = e.push(NS(samples=[]), 11.)
        self.assertEqual(held['probabilities'], first['probabilities'])
        self.assertEqual(held['estimate_stamp'], 10.)
        self.assertFalse(held['updated'])
        self.assertTrue(held['status'] & Status.STALE)

    def test_missing_model_never_fakes_probability(self):
        r = Estimator().push(batch(10.), 10.)
        self.assertIn('features', r)
        self.assertIsNone(r['probabilities'])
        self.assertFalse(r['has_estimate'])

    def test_old_new_numeric_equivalence(self):
        old, new = Estimator(self.model), Estimator(self.model)
        for i in range(30):
            t = 10 + i*.05
            a, b = old.push(batch(t, old=True), t), new.push(batch(t), t)
            self.assertEqual(a['features'], b['features'])
            self.assertEqual(a['probabilities'], b['probabilities'])
        self.assertNotEqual(a['timestamp_semantics'], b['timestamp_semantics'])

    def test_rewind_resets_window(self):
        e = Estimator(self.model)
        e.push(batch(20.), 20.)
        r = e.push(batch(10.), 10.)
        self.assertTrue(r['status'] & Status.TIME_RESET)
        self.assertEqual(r['valid_sample_count'], 1)

    def test_zero_signal_holds(self):
        e = Estimator(self.model)
        r = e.push(batch(10., values=(0.,0.,0.)), 10.)
        self.assertFalse(r['updated'])
        self.assertTrue(r['status'] & Status.LOW_SIGNAL)

    def test_quality_survives_heartbeat(self):
        e = Estimator(self.model)
        e.push(batch(10.), 10.)
        e.push(NS(samples=[]), 10.01)
        self.assertTrue(e.snapshot(10.02)['status'] & Status.INVALID_INPUT)

    def test_low_nonzero_signal_still_computes(self):
        r = Estimator(self.model).push(batch(10., values=(.00001, .00002, .00001)), 10.)
        self.assertTrue(r['updated'])
        self.assertTrue(r['status'] & Status.LOW_SIGNAL)

    def test_comparison_without_model(self):
        e = Estimator()
        for i in range(12):
            result = e.push(batch(10+i*.05), 10+i*.05)
        c = result['comparison']
        self.assertTrue(c['reliable'])
        self.assertAlmostEqual(c['left_right_normalized_difference'], 1/3)
        self.assertAlmostEqual(c['head_sides_normalized_difference'], -.2)
        self.assertEqual(result['features'][2:], [c['left_right_normalized_difference'],
                                                 c['head_sides_normalized_difference']])
        self.assertFalse(c['status'] & Status.MODEL_UNAVAILABLE)
        self.assertIsNone(result['probabilities'])

    def test_comparison_signs_and_symmetry(self):
        for values, expected in [((1., 1., 1.), (0., 0.)),
                                 ((3., 1., 1.), (0., .5)),
                                 ((1., 1., 3.), (-.5, -1/3)),
                                 ((1., 3., 1.), (.5, -1/3)),
                                 ((1., 0., 0.), (0., 1.))]:
            with self.subTest(values=values):
                c = Estimator().push(batch(10., values=values), 10.)['comparison']
                self.assertAlmostEqual(c['left_right_normalized_difference'], expected[0])
                self.assertAlmostEqual(c['head_sides_normalized_difference'], expected[1])
                if values == (1., 0., 0.):
                    self.assertFalse(c['reliable'])
                    self.assertTrue(c['status'] & Status.INVALID_INPUT)

    def test_comparison_without_input(self):
        c = Estimator().snapshot(10.)['comparison']
        self.assertFalse(c['has_comparison'])
        self.assertFalse(c['reliable'])

    def test_comparison_hold_preserves_stamp_and_marks_stale(self):
        e = Estimator()
        first = e.push(batch(10.), 10.)['comparison']
        held = e.snapshot(11.)['comparison']
        self.assertEqual(first['sample_stamp'], held['sample_stamp'])
        self.assertFalse(held['updated'])
        self.assertFalse(held['reliable'])
        self.assertTrue(held['status'] & Status.STALE)

    def test_comparison_zero_is_numeric_but_not_reliable(self):
        c = Estimator().push(batch(10., values=(0., 0., 0.)), 10.)['comparison']
        self.assertTrue(c['updated'])
        self.assertEqual(c['left_right_normalized_difference'], 0.)
        self.assertEqual(c['head_sides_normalized_difference'], 0.)
        self.assertFalse(c['reliable'])

    def test_comparison_fallback_never_reliable(self):
        e = Estimator()
        for i in range(12):
            c = e.push(batch(10+i*.05, calibrated=False), 10+i*.05)['comparison']
        self.assertFalse(c['reliable'])
        self.assertTrue(c['status'] & Status.CALIBRATION_FALLBACK)

    def test_gain_transition_resets_comparison_window(self):
        e = Estimator()
        for i in range(12):
            e.push(batch(10+i*.05), 10+i*.05)
        message = batch(10.6, values=(3., 2., 1.))
        for sample in message.samples:
            sample.dac_volt = 1.
        c = e.push(message, 10.6)['comparison']
        self.assertAlmostEqual(c['left_right_normalized_difference'], 1/3)
        self.assertAlmostEqual(c['head_sides_normalized_difference'], 1/3)
        self.assertFalse(c['reliable'])
        self.assertTrue(c['status'] & Status.GAIN_TRANSITION)


if __name__ == '__main__':
    unittest.main()
