"""Cycle coverage, signal quality and threshold steering without ROS."""
import math
from types import SimpleNamespace as NS
import unittest

from robotic_fish_sonic.cycle_steering import CycleSteering, SteeringFault


def batch(t, left=2., right=1., **changes):
    samples = []
    for channel, value in enumerate((1., left, right)):
        data = dict(channel_id=channel, timestamp=NS(to_sec=lambda t=t: t),
                    volt_cali=value, status_cali=True, cali_id='test',
                    adc_code=100, status_dac_feedback=True, dac_volt=.9)
        data.update(changes)
        samples.append(NS(**data))
    return NS(samples=samples)


class CycleTests(unittest.TestCase):
    def push(self, engine, t, **kwargs):
        return engine.push(batch(t, **kwargs), 100. + t, t)

    def start(self, engine):
        self.push(engine, 0.)
        engine.begin(100., engine.frequency, 100.)

    def fill(self, engine, start=0., end=.5, left=2., right=1.):
        for i in range(1, 11):
            self.push(engine, start + (end - start) * i / 10., left=left, right=right)

    def test_full_period_and_same_direction_crossing_only(self):
        for frequency in (1., 1.5, 2.):
            with self.subTest(frequency=frequency):
                e = CycleSteering(frequency)
                self.start(e)
                for i in range(1, 21):
                    t = e.period * i / 20.
                    self.push(e, t)
                    result = e.evaluate(100. + t)
                    if i < 20:
                        self.assertIsNone(result)  # Including half-cycle and 80% coverage.
                self.assertAlmostEqual(result.end - result.start, 1. / frequency)
                self.assertAlmostEqual(result.difference, 1 / 3)
                self.assertEqual(result.bias, 256.)
                self.assertIsNone(e.evaluate(100. + e.period + .001))

    def test_bias_is_held_and_never_accumulated(self):
        e = CycleSteering(2.)
        self.start(e)
        self.fill(e)
        self.assertEqual(e.evaluate(100.5).bias, 256.)
        self.fill(e, .5, 1.)
        self.assertEqual(e.evaluate(101.).bias, 256.)

    def test_window_means_exclude_the_next_cycle_boundary(self):
        e = CycleSteering(2.)
        self.start(e)
        for i in range(1, 10):
            self.push(e, i * .05)
        self.push(e, .5, left=1., right=3.)
        first = e.evaluate(100.5)
        self.assertAlmostEqual(first.left_mean, 2.)
        self.assertAlmostEqual(first.right_mean, 1.)
        self.assertEqual(first.sample_count, 10)
        self.fill(e, .5, 1., left=1., right=3.)
        second = e.evaluate(101.)
        self.assertAlmostEqual(second.left_mean, 1.)
        self.assertAlmostEqual(second.right_mean, 3.)
        self.assertEqual(second.sample_count, 10)

    def test_threshold_sign_and_neutral(self):
        for left, right, expected in [(3., 1., 128.), (1., 3., -128.),
                                       (1., 1., 0.), (1.1, 1., 0.)]:
            with self.subTest(left=left,right=right):
                e = CycleSteering(2., bias=128.)
                self.push(e, 0., left=left, right=right)
                e.begin(100., 2., 100.)
                self.fill(e, left=left, right=right)
                self.assertEqual(e.evaluate(100.5).bias, expected)

    def test_polarity_and_threshold_equality(self):
        e = CycleSteering(2., threshold=.5, left_bias_sign=-1)
        self.push(e, 0., left=3.)
        e.begin(100., 2., 100.)
        self.fill(e, left=3.)
        self.assertEqual(e.evaluate(100.5).bias, 0.)
        e.configure(.1, 128.)
        self.fill(e, .5, 1., left=3.)
        self.assertEqual(e.evaluate(101.).bias, -128.)

    def test_atomic_parameters_apply_at_boundary(self):
        e = CycleSteering(2.)
        self.start(e)
        e.configure(.6, 100.)
        self.assertEqual(e.settings.bias, 256.)
        with self.assertRaises(ValueError):
            e.configure(.1, 2048.)
        self.assertEqual((e.pending.threshold,e.pending.bias), (.6,100.))
        self.fill(e)
        result=e.evaluate(100.5)
        self.assertEqual((result.bias,e.settings.bias), (0.,100.))
        self.assertIsNone(e.pending)

    def test_missing_adc_and_timeouts(self):
        e = CycleSteering(2.)
        with self.assertRaisesRegex(SteeringFault,'no_data'):
            e.begin(100.,2.,100.)
        self.start(e)
        with self.assertRaisesRegex(SteeringFault,'adc_timeout'):
            e.evaluate(100.21)

    def test_invalid_frames_and_quality_are_rejected(self):
        cases=[NS(samples=[]), batch(0.,status_cali=False), batch(0.,status_dac_feedback=False),
               batch(0.,adc_code=32767), batch(0.,left=0.,right=0.),
               batch(0.,left=float('nan')),batch(0.,left=-1.)]
        for msg in cases:
            with self.subTest(msg=msg):
                e=CycleSteering(2.)
                self.assertFalse(e.push(msg,100.,0.))
                with self.assertRaises(SteeringFault): e.begin(100.,2.,100.)

    def test_transient_fault_cannot_be_hidden_by_later_valid_frame(self):
        e=CycleSteering(2.)
        self.start(e)
        self.push(e,.05,status_cali=False)
        self.push(e,.1)
        with self.assertRaisesRegex(SteeringFault,'calibration_fallback'):
            e.evaluate(100.1)
        e.deactivate()
        e.begin(100.1,2.,100.1)
        self.fill(e,.1,.6)
        self.assertEqual(e.evaluate(100.6).cycle,1)

    def test_fresh_frame_cannot_hide_timeout_or_sampling_gap_between_ticks(self):
        for limits, gap, reason in [({}, .21, 'adc_timeout'),
                                   ({'gap_s': .1, 'max_age_s': .3}, .15, 'adc_gap')]:
            with self.subTest(reason=reason):
                e = CycleSteering(2., **limits)
                self.start(e)
                self.assertTrue(self.push(e, gap))
                with self.assertRaisesRegex(SteeringFault, reason):
                    e.evaluate(100. + gap)
                self.push(e, gap + .05)
                with self.assertRaisesRegex(SteeringFault, reason):
                    e.evaluate(100. + gap + .05)
                e.deactivate()
                recovered = 100. + (gap + .05)
                e.begin(recovered, 2., recovered)
                self.assertIsNone(e.evaluate(recovered))

    def test_gain_or_calibration_change_requires_new_cycle(self):
        for change in ({'dac_volt':1.}, {'cali_id':'new'}):
            with self.subTest(change=change):
                e=CycleSteering(2.)
                self.start(e)
                self.push(e,.05,**change)
                with self.assertRaisesRegex(SteeringFault,'gain_or_calibration_changed'):
                    e.evaluate(100.05)

    def test_future_old_duplicate_and_ros_clock_change(self):
        for msg,received,ros_now,reason in [(batch(1.),100.,0.,'future_sample'),
                                         (batch(0.),101.,1.,'stale_sample')]:
            e=CycleSteering(2.)
            self.assertFalse(e.push(msg,received,ros_now))
            self.assertEqual(e.latest_error,reason)
        e=CycleSteering(2.)
        self.start(e)
        self.assertFalse(self.push(e,0.))
        self.assertEqual(e.latest_error,'sample_time_reset')
        e=CycleSteering(2.)
        self.start(e)
        self.assertFalse(e.push(batch(.1),100.2,.1))
        self.assertEqual(e.latest_error,'clock_changed')

    def test_small_clock_skew_is_tolerated(self):
        e=CycleSteering(2.)
        self.assertTrue(e.push(batch(.003),100.,0.))
        e.begin(100.,2.,100.)
        self.assertEqual(e.latest_time,100.)

    def test_epoch_clock_mapping_does_not_drift_with_receipt_timing(self):
        e=CycleSteering(2.)
        self.start(e)
        self.assertTrue(e.push(batch(.05),100.0501,.05))
        self.assertAlmostEqual(e.clock_offset,100.0001)
        self.assertEqual(e.epoch_clock_offset,100.)

    def test_incomplete_cycle_and_missed_crossing(self):
        e=CycleSteering(10.)
        self.start(e)
        self.push(e,.1)
        with self.assertRaisesRegex(SteeringFault,'incomplete_cycle'):
            e.evaluate(100.1)
        e=CycleSteering(2.)
        self.start(e)
        self.fill(e)
        self.fill(e,.5,1.)
        with self.assertRaisesRegex(SteeringFault,'missed_cycle'):
            e.evaluate(101.)

    def test_new_frequency_uses_a_new_complete_period(self):
        e=CycleSteering(2.)
        self.start(e)
        self.fill(e)
        e.evaluate(100.5)
        e.begin(100.5,1.,100.5)
        self.fill(e,.5,1.)
        self.assertIsNone(e.evaluate(101.))
        self.fill(e,1.,1.5)
        self.assertAlmostEqual(e.evaluate(101.5).end-e.last_decision.start,1.)

    def test_bad_configuration(self):
        for kwargs in ({'frequency':0},{'frequency':float('nan')},{'threshold':-1},
                       {'threshold':2},{'bias':-1},{'bias':1025},{'left_bias_sign':0},
                       {'left_bias_sign':True},{'gap_s':0},{'max_age_s':0},
                       {'future_tolerance_s':-1},{'low_signal_v':0}):
            with self.subTest(kwargs=kwargs),self.assertRaises(ValueError):
                CycleSteering(**dict({'frequency':2.}, **kwargs))


if __name__=='__main__':
    unittest.main()
