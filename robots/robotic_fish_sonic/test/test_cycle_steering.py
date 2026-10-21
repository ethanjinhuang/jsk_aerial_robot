"""Cycle coverage, signal quality and threshold steering without ROS."""
import math
from types import SimpleNamespace as NS
import unittest

from robotic_fish_sonic.cycle_steering import CycleSteering, SteeringFault


def batch(t, left=2., right=1., head=1., **changes):
    samples = []
    for channel, value in enumerate((head, left, right)):
        data = dict(channel_id=channel, timestamp=NS(to_sec=lambda t=t: t),
                    volt_raw=value * .5, volt_cali=value, status_cali=True, cali_id='test',
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

    def fill(self, engine, start=0., end=.5, left=2., right=1., **changes):
        for i in range(1, 11):
            self.push(engine, start + (end - start) * i / 10., left=left, right=right, **changes)

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
                self.assertAlmostEqual(result.difference, 2 / 3)
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
        self.push(e, .5, left=1., right=3., head=4.)
        first = e.evaluate(100.5)
        self.assertAlmostEqual(first.head_mean, 1.)
        self.assertAlmostEqual(first.left_mean, 2.)
        self.assertAlmostEqual(first.right_mean, 1.)
        self.assertEqual(first.raw_means, (.5, 1., .5))
        self.assertEqual(first.sample_count, 10)
        self.fill(e, .5, 1., left=1., right=3.)
        second = e.evaluate(101.)
        self.assertAlmostEqual(second.head_mean, 1.3)
        self.assertAlmostEqual(second.left_mean, 1.)
        self.assertAlmostEqual(second.right_mean, 3.)
        self.assertAlmostEqual(second.raw_means[0], .65)
        self.assertEqual(second.raw_means[1:], (.5, 1.5))
        self.assertEqual(second.sample_count, 10)

    def test_steering_offset_does_not_change_raw_or_file_calibrated_cycle_means(self):
        e = CycleSteering(2.)
        self.start(e)
        self.fill(e)
        before = e.evaluate(100.5)
        e.configure(.1, 128., -.75)
        self.fill(e, .5, 1.)
        after = e.evaluate(101.)
        self.assertEqual(before.raw_means, (.5, 1., .5))
        self.assertEqual(after.raw_means, before.raw_means)
        self.assertEqual((after.head_mean, after.left_mean, after.right_mean),
                         (before.head_mean, before.left_mean, before.right_mean))
        self.assertAlmostEqual(after.difference, before.difference)
        self.assertGreater(before.compensated_difference, 0.)
        self.assertLess(after.compensated_difference, 0.)

    def test_missing_or_invalid_raw_diagnostics_do_not_stop_steering(self):
        for value in (None, float('nan'), float('inf'), 'bad', True):
            with self.subTest(value=value):
                e = CycleSteering(2.)
                frame = batch(0.)
                if value is None:
                    del frame.samples[0].volt_raw
                else:
                    frame.samples[0].volt_raw = value
                self.assertTrue(e.push(frame, 100., 0.))
                e.begin(100., 2., 100.)
                self.fill(e)
                result = e.evaluate(100.5)
                self.assertTrue(math.isnan(result.raw_means[0]))
                self.assertEqual(result.raw_means[1:], (1., .5))
                self.assertEqual(result.bias, 256.)

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
        e = CycleSteering(2., threshold=1., left_bias_sign=-1)
        self.push(e, 0., left=3.)
        e.begin(100., 2., 100.)
        self.fill(e, left=3.)
        self.assertEqual(e.evaluate(100.5).bias, -256.)
        e.configure(.1, 128., 0.)
        self.fill(e, .5, 1., left=3.)
        self.assertEqual(e.evaluate(101.).bias, -128.)

    def test_fifteen_percent_of_mean_boundary_is_inclusive_and_symmetric(self):
        for scale in (.01, 1., 10.):
            for difference, expected in ((.15 - 1e-8, 0.), (.15, 128.), (.15 + 1e-8, 128.)):
                for sign in (-1, 1):
                    with self.subTest(scale=scale, difference=difference, sign=sign):
                        left = scale * (1. + sign * difference / 2.)
                        right = scale * (1. - sign * difference / 2.)
                        e = CycleSteering(2., threshold=.15, bias=128.)
                        self.push(e, 0., left=left, right=right)
                        e.begin(100., 2., 100.)
                        self.fill(e, left=left, right=right)
                        result = e.evaluate(100.5)
                        self.assertAlmostEqual(result.compensated_difference, sign * difference)
                        self.assertEqual(result.bias, sign * expected)

    def test_zero_threshold_equal_channels_stay_straight(self):
        for left, right, expected in ((1., 1., 0.), (1.00001, 1., 256.), (1., 1.00001, -256.)):
            e = CycleSteering(2., threshold=0.)
            self.push(e, 0., left=left, right=right)
            e.begin(100., 2., 100.)
            self.fill(e, left=left, right=right)
            self.assertEqual(e.evaluate(100.5).bias, expected)

    def test_two_hundred_percent_threshold_requires_zero_weaker_channel(self):
        for left, right, expected in ((2., 0., 256.), (0., 2., -256.), (2., .001, 0.)):
            e = CycleSteering(2., threshold=2.)
            self.push(e, 0., left=left, right=right)
            e.begin(100., 2., 100.)
            self.fill(e, left=left, right=right)
            self.assertEqual(e.evaluate(100.5).bias, expected)

    def test_atomic_parameters_apply_at_boundary(self):
        e = CycleSteering(2.)
        self.start(e)
        e.configure(1.2, 100., 0.)
        self.assertEqual(e.settings.bias, 256.)
        with self.assertRaises(ValueError):
            e.configure(.1, 2048., 0.)
        self.assertEqual((e.pending.threshold,e.pending.bias), (1.2,100.))
        self.fill(e)
        result=e.evaluate(100.5)
        self.assertEqual((result.bias,e.settings.bias), (0.,100.))
        self.assertIsNone(e.pending)

    def test_compensation_shifts_the_decision_without_changing_measured_means(self):
        cases = [(3., 1., -1., .1, 0.), (3., 1., -.5, .1, 128.),
                 (3., 1., -1.5, .1, -128.), (1., 3., 1., .1, 0.),
                 (1., 3., 1.5, .1, 128.), (1., 3., .5, .1, -128.),
                 (3., 1., -.5, .5, 128.), (3., 1., -1.5, .5, -128.)]
        for left, right, offset, threshold, bias in cases:
            for polarity in (-1, 1):
                with self.subTest(left=left, offset=offset, polarity=polarity):
                    e = CycleSteering(2., threshold=threshold, bias=128., lr_voltage_offset_v=offset,
                                      left_bias_sign=polarity)
                    self.push(e, 0., left=left, right=right)
                    e.begin(100., 2., 100.)
                    self.fill(e, left=left, right=right)
                    result = e.evaluate(100.5)
                    self.assertAlmostEqual(result.left_mean, left)
                    self.assertAlmostEqual(result.right_mean, right)
                    self.assertAlmostEqual(result.difference, (left - right) / ((left + right) / 2))
                    self.assertAlmostEqual(result.compensated_left_mean, left + offset)
                    self.assertAlmostEqual(result.compensated_right_mean, right - offset)
                    self.assertAlmostEqual(result.compensated_difference, (left - right + 2 * offset) / ((left + right) / 2))
                    self.assertEqual(result.lr_voltage_offset_v, offset)
                    self.assertEqual(result.bias, polarity * bias)

    def test_fixed_voltage_difference_is_balanced_at_different_signal_levels(self):
        for level in (.03, .1, 1.):
            e = CycleSteering(2., lr_voltage_offset_v=-.01)
            self.push(e, 0., left=level + .02, right=level)
            e.begin(100., 2., 100.)
            self.fill(e, left=level + .02, right=level)
            result = e.evaluate(100.5)
            self.assertAlmostEqual(result.compensated_difference, 0.)
            self.assertEqual(result.bias, 0.)

    def test_voltage_correction_is_not_a_dimensionless_offset(self):
        for scale, expected in ((.5, -1.), (1., 0.), (2., .5)):
            e = CycleSteering(2., lr_voltage_offset_v=-1.)
            self.push(e, 0., left=3. * scale, right=scale)
            e.begin(100., 2., 100.)
            self.fill(e, left=3. * scale, right=scale)
            self.assertAlmostEqual(e.evaluate(100.5).compensated_difference, expected)

    def test_three_settings_are_validated_and_applied_as_one_update(self):
        e = CycleSteering(2., lr_voltage_offset_v=.1)
        self.start(e)
        e.configure(.1, 128., -1.)
        self.assertEqual((e.settings.threshold, e.settings.bias, e.settings.lr_voltage_offset_v), (.1, 256., .1))
        self.assertEqual((e.pending.threshold, e.pending.bias, e.pending.lr_voltage_offset_v), (.1, 128., -1.))
        pending = e.pending
        for value in (float('nan'), float('inf'), -float('inf'), True):
            with self.assertRaises(ValueError):
                e.configure(.2, 64., value)
            self.assertIs(e.pending, pending)
            with self.assertRaises(ValueError):
                CycleSteering(2., lr_voltage_offset_v=value)
        for threshold, bias in [(-1., 64.), (.2, 2048.)]:
            with self.assertRaises(ValueError):
                e.configure(threshold, bias, -.25)
            self.assertIs(e.pending, pending)
        self.fill(e)
        self.assertEqual(e.evaluate(100.5).bias, -128.)
        self.assertEqual(e.settings, pending)
        self.assertIsNone(e.pending)
        e.configure(.2, 64., .25)
        self.fill(e, .5, 1.)
        self.assertEqual(e.evaluate(101.).bias, 64.)
        self.assertEqual((e.settings.threshold, e.settings.bias, e.settings.lr_voltage_offset_v), (.2, 64., .25))

    def test_compensation_waits_through_quality_hold_without_rewriting_old_decision(self):
        e = CycleSteering(2.)
        self.start(e)
        self.fill(e)
        previous = e.evaluate(100.5)
        e.configure(.1, 256., -.75)
        for i in range(11, 22):
            t = i * .05
            self.push(e, t, status_cali=False, cali_id='')
            self.assertIsNone(e.evaluate(100. + t))
            self.assertEqual(e.settings.lr_voltage_offset_v, 0.)
            self.assertIs(e.last_decision, previous)
            self.assertEqual(previous.lr_voltage_offset_v, 0.)
        for i in range(22, 41):
            t = i * .05
            self.push(e, t)
            result = e.evaluate(100. + t)
            if i < 40:
                self.assertIsNone(result)
        self.assertEqual((e.epoch, result.cycle, result.lr_voltage_offset_v, result.bias), (100., 4, -.75, -256.))

    def test_out_of_range_pending_triplet_holds_previous_decision_and_can_be_replaced(self):
        e = CycleSteering(2.)
        self.start(e)
        self.fill(e)
        previous = e.evaluate(100.5)
        e.configure(.2, 128., -3.)
        pending = e.pending
        for start in (.5, 1.):
            self.fill(e, start, start + .5)
            self.assertIsNone(e.evaluate(100. + start + .5))
            self.assertIs(e.last_decision, previous)
            self.assertIs(e.pending, pending)
            self.assertEqual((e.settings.threshold, e.settings.bias, e.settings.lr_voltage_offset_v), (.1, 256., 0.))
            self.assertEqual(e.waiting_status, 'voltage_compensation_out_of_range')
        e.configure(.1, 64., -.75)
        self.fill(e, 1.5, 2.)
        result = e.evaluate(102.)
        self.assertEqual((e.epoch, result.cycle, result.bias), (100., 4, -64.))
        self.assertIsNone(e.pending)
        self.assertFalse(e.waiting_status)

    def test_active_compensation_recovers_automatically_when_voltage_is_sufficient(self):
        e = CycleSteering(2., lr_voltage_offset_v=-.5)
        self.push(e, 0., left=.2, right=.1)
        e.begin(100., 2., 100.)
        for i in range(1, 10):
            self.push(e, i * .05, left=.2, right=.1)
        self.push(e, .5, left=1., right=1.)
        self.assertIsNone(e.evaluate(100.5))
        self.assertEqual(e.cycle, 1)
        self.assertIsNone(e.last_decision)
        self.fill(e, .5, 1., left=1., right=1.)
        result = e.evaluate(101.)
        self.assertEqual((result.cycle, result.compensated_left_mean, result.compensated_right_mean), (2, .5, 1.5))
        self.assertEqual(result.bias, -256.)

    def test_zero_compensated_channel_is_valid_and_overflow_is_not_clamped(self):
        for offset, expected in ((-3., -2.), (1., 2.)):
            e = CycleSteering(2., lr_voltage_offset_v=offset)
            self.push(e, 0., left=3., right=1.)
            e.begin(100., 2., 100.)
            self.fill(e, left=3., right=1.)
            self.assertEqual(e.evaluate(100.5).compensated_difference, expected)
        e = CycleSteering(2., lr_voltage_offset_v=1e308)
        self.push(e, 0., left=1e308, right=1.)
        e.begin(100., 2., 100.)
        self.fill(e, left=1e308, right=1.)
        self.assertIsNone(e.evaluate(100.5))
        self.assertEqual(e.waiting_status, 'voltage_compensation_out_of_range')

    def test_compensation_hold_does_not_hide_timeout_or_saturation(self):
        for fault in ('timeout', 'saturation'):
            e = CycleSteering(2., lr_voltage_offset_v=-3.)
            self.start(e)
            self.fill(e)
            self.assertIsNone(e.evaluate(100.5))
            if fault == 'timeout':
                with self.assertRaisesRegex(SteeringFault, 'adc_timeout'):
                    e.evaluate(100.71)
            else:
                self.push(e, .55, adc_code=32767)
                with self.assertRaisesRegex(SteeringFault, 'saturated'):
                    e.evaluate(100.55)

    def test_missing_adc_and_timeouts(self):
        e = CycleSteering(2.)
        with self.assertRaisesRegex(SteeringFault,'no_data'):
            e.begin(100.,2.,100.)
        self.start(e)
        with self.assertRaisesRegex(SteeringFault,'adc_timeout'):
            e.evaluate(100.21)

    def test_invalid_frames_and_quality_are_rejected(self):
        cases=[NS(samples=[]), batch(0.,adc_code=32767), batch(0.,left=0.,right=0.),
               batch(0.,left=float('nan')),batch(0.,left=-1.)]
        for msg in cases:
            with self.subTest(msg=msg):
                e=CycleSteering(2.)
                self.assertFalse(e.push(msg,100.,0.))
                with self.assertRaises(SteeringFault): e.begin(100.,2.,100.)

    def test_transient_fault_cannot_be_hidden_by_later_valid_frame(self):
        e=CycleSteering(2.)
        self.start(e)
        self.push(e,.05,adc_code=32767)
        self.push(e,.1)
        with self.assertRaisesRegex(SteeringFault,'saturated'):
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

    def test_gain_change_holds_decision_until_a_full_aligned_cycle(self):
        e = CycleSteering(2.)
        self.start(e)
        self.fill(e)
        previous = e.evaluate(100.5)
        e.configure(.1, 128., 0.)
        self.push(e, .55, left=1., right=3., dac_volt=1.)
        for i in range(12, 30):
            t = i * .05
            self.push(e, t, left=1., right=3., dac_volt=1.)
            self.assertIsNone(e.evaluate(100. + t))
            self.assertIs(e.last_decision, previous)
            self.assertTrue(e.gain_warmup)
            self.assertIsNotNone(e.pending)
        self.push(e, 1.5, left=1., right=3., dac_volt=1.)
        result = e.evaluate(101.5)
        self.assertEqual((e.epoch, result.cycle, result.start, result.end), (100., 3, 101., 101.5))
        self.assertAlmostEqual(result.left_mean, 1.)
        self.assertAlmostEqual(result.right_mean, 3.)
        self.assertEqual(result.bias, -128.)
        self.assertFalse(e.gain_warmup)
        self.assertIsNone(e.pending)

    def test_mixed_gain_groups_refresh_timing_but_never_enter_means(self):
        e = CycleSteering(2.)
        self.start(e)
        self.fill(e)
        previous = e.evaluate(100.5)
        for t in (.55, .6, .65):
            mixed = batch(t, left=100.)
            mixed.samples[0].dac_volt = 1.
            self.assertTrue(e.push(mixed, 100. + t, t))
            self.assertIsNone(e.evaluate(100. + t))
            self.assertIs(e.last_decision, previous)
            self.assertFalse(e.rows)
        for i in range(14, 31):
            t = i * .05
            self.push(e, t, left=1., right=3., dac_volt=1.)
            result = e.evaluate(100. + t)
            if i < 30:
                self.assertIsNone(result)
        self.assertAlmostEqual(result.left_mean, 1.)
        self.assertAlmostEqual(result.right_mean, 3.)
        self.assertEqual(result.bias, -256.)
        # Entering tracking with mixed gain waits without inventing a mean.
        e.deactivate()
        mixed = batch(1.55)
        mixed.samples[0].dac_volt = 1.
        e.push(mixed, 101.55, 1.55)
        e.begin(101.55, 2., 101.55)
        self.assertIsNone(e.evaluate(101.55))
        self.assertTrue(e.gain_warmup)
        self.assertIsNone(e.last_decision)

    def test_continuous_gain_ramp_skips_cycles_without_resetting_phase(self):
        e = CycleSteering(2.)
        self.start(e)
        for i in range(1, 31):
            t = i * .05
            self.push(e, t, dac_volt=1. + i * .01)
            self.assertIsNone(e.evaluate(100. + t))
        self.assertEqual((e.epoch, e.cycle), (100., 3))
        self.assertIsNone(e.last_decision)
        self.fill(e, 1.5, 2., dac_volt=1.3)
        self.assertEqual(e.evaluate(102.).cycle, 4)

    def test_gain_change_at_crossing_resumes_at_the_next_full_cycle(self):
        for frequency in (1.5, 2.):
            e = CycleSteering(frequency)
            self.start(e)
            for i in range(1, 20):
                self.push(e, e.period * i / 20.)
            self.push(e, e.period, dac_volt=1.)
            self.assertIsNone(e.evaluate(100. + e.period))
            for i in range(21, 41):
                self.push(e, e.period * i / 20., dac_volt=1.)
            self.assertEqual(e.evaluate(100. + 2 * e.period).cycle, 2)

    def test_gain_warmup_does_not_hide_faults(self):
        for changes, reason in [({'adc_code': 32767}, 'saturated'),
                                ({'cali_id': 'new'}, 'gain_or_calibration_changed')]:
            with self.subTest(reason=reason):
                e = CycleSteering(2.)
                self.start(e)
                self.push(e, .05, dac_volt=1.)
                self.push(e, .1, dac_volt=1., **changes)
                self.push(e, .15, dac_volt=1.1)
                with self.assertRaisesRegex(SteeringFault, reason):
                    e.evaluate(100.15)
        e = CycleSteering(2.)
        self.start(e)
        self.push(e, .21, dac_volt=1.)
        with self.assertRaisesRegex(SteeringFault, 'adc_timeout'):
            e.evaluate(100.21)
        e = CycleSteering(2.)
        self.start(e)
        mixed = batch(.05)
        mixed.samples[0].dac_volt = 1.
        e.push(mixed, 100.05, .05)
        with self.assertRaisesRegex(SteeringFault, 'adc_timeout'):
            e.evaluate(100.26)

    def test_unusable_quality_holds_until_a_new_complete_valid_cycle(self):
        cases = [({'status_cali': False, 'cali_id': ''}, 'calibration_fallback'),
                 ({'status_dac_feedback': False, 'dac_volt': float('nan')}, 'gain_unknown'),
                 ({'status_cali': False, 'cali_id': '', 'status_dac_feedback': False,
                   'dac_volt': float('nan')}, 'calibration_fallback|gain_unknown')]
        for changes, reason in cases:
            with self.subTest(reason=reason):
                e = CycleSteering(2.)
                self.start(e)
                self.fill(e)
                previous = e.evaluate(100.5)
                e.configure(.1, 128., 0.)
                for i in range(11, 31):
                    t = i * .05
                    self.assertTrue(self.push(e, t, left=0., right=0., **changes))
                    self.assertIsNone(e.evaluate(100. + t))
                    self.assertEqual(e.waiting_status, reason)
                    self.assertIs(e.last_decision, previous)
                    self.assertFalse(e.rows)
                    self.assertIsNotNone(e.pending)
                for i in range(31, 51):
                    t = i * .05
                    self.push(e, t, left=1., right=3.)
                    result = e.evaluate(100. + t)
                    if i < 50:
                        self.assertIsNone(result)
                        self.assertEqual(e.waiting_status, 'data_warmup')
                self.assertEqual((e.epoch, result.cycle, result.bias), (100., 5, -128.))
                self.assertAlmostEqual(result.left_mean, 1.)
                self.assertAlmostEqual(result.right_mean, 3.)
                self.assertFalse(e.waiting_status)

    def test_entering_tracking_with_unusable_quality_waits_for_valid_data(self):
        for changes in ({'status_cali': False, 'cali_id': ''},
                        {'status_dac_feedback': False, 'dac_volt': float('nan')}):
            with self.subTest(changes=changes):
                e = CycleSteering(2.)
                self.push(e, 0., **changes)
                e.begin(100., 2., 100.)
                for i in range(1, 11):
                    t = i * .05
                    self.push(e, t, **changes)
                    self.assertIsNone(e.evaluate(100. + t))
                self.assertIsNone(e.last_decision)
                self.assertTrue(e.waiting_status)
                for i in range(11, 31):
                    t = i * .05
                    self.push(e, t)
                    result = e.evaluate(100. + t)
                self.assertEqual(result.cycle, 3)

    def test_quality_hold_still_checks_timestamps_and_latches_hard_faults(self):
        for quality in ({'status_cali': False}, {'status_dac_feedback': False}):
            with self.subTest(quality=quality):
                e = CycleSteering(2.)
                self.start(e)
                self.push(e, .05, **quality)
                with self.assertRaisesRegex(SteeringFault, 'adc_timeout'):
                    e.evaluate(100.26)
                e = CycleSteering(2.)
                self.start(e)
                self.push(e, .05, adc_code=32767, **quality)
                self.push(e, .1, **quality)
                self.push(e, .15)
                with self.assertRaisesRegex(SteeringFault, 'saturated'):
                    e.evaluate(100.15)
                e = CycleSteering(2.)
                self.start(e)
                self.assertFalse(e.push(batch(0., **quality), 100.3, .3))
                with self.assertRaisesRegex(SteeringFault, 'stale_sample'):
                    e.evaluate(100.3)

    def test_quality_recovery_does_not_hide_a_changed_valid_calibration(self):
        e = CycleSteering(2.)
        self.start(e)
        self.push(e, .05, status_cali=False, cali_id='')
        self.push(e, .1, cali_id='new')
        with self.assertRaisesRegex(SteeringFault, 'gain_or_calibration_changed'):
            e.evaluate(100.1)

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
                       {'threshold':2.01},{'threshold':float('nan')},{'bias':-1},{'bias':1025},{'left_bias_sign':0},
                       {'left_bias_sign':True},{'gap_s':0},{'max_age_s':0},
                       {'future_tolerance_s':-1},{'low_signal_v':0}):
            with self.subTest(kwargs=kwargs),self.assertRaises(ValueError):
                CycleSteering(**dict({'frequency':2.}, **kwargs))


if __name__=='__main__':
    unittest.main()
