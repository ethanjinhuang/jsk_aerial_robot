"""Independent physical-channel cases for steering, features and calibration."""
import importlib.util
from pathlib import Path
from types import SimpleNamespace as NS
import unittest

from robotic_fish_sonic.acoustic_core import adapt, Estimator
from robotic_fish_sonic.adc_calibration import AdcCalibration
from robotic_fish_sonic.channel_mapping import channel_mapping, head_left_right
from robotic_fish_sonic.cycle_steering import CycleSteering


def physical_batch(t, adc_volts, raw_volts=None, legacy_schema=False):
    samples = []
    for channel, voltage in enumerate(adc_volts):
        stamp = NS(to_sec=lambda: t)
        if legacy_schema:
            sample = NS(channel=channel, header=NS(stamp=stamp), calibrated_voltage=voltage,
                        calibration_applied=True, calibration_id='test', raw=100,
                        gain_control_voltage_valid=True, gain_control_voltage=.9)
        else:
            sample = NS(channel_id=channel, timestamp=stamp, volt_cali=voltage,
                        volt_raw=raw_volts[channel] if raw_volts is not None else voltage,
                        status_cali=True, cali_id='test', adc_code=100,
                        status_dac_feedback=True, dac_volt=.9)
        samples.append(sample)
    # Consumers must use channel IDs rather than the array's position.
    return NS(samples=[samples[1], samples[2], samples[0]])


class ChannelMappingTests(unittest.TestCase):
    def test_current_and_legacy_layouts_are_explicit(self):
        self.assertEqual(channel_mapping(), {'head': 2, 'left': 0, 'right': 1})
        self.assertEqual(head_left_right((3., 1., 2.)), (2., 3., 1.))
        self.assertEqual(head_left_right((2., 3., 1.), 'head_left_right'), (2., 3., 1.))
        for layout in ('', 'unknown', None, True, []):
            with self.subTest(layout=layout), self.assertRaises(ValueError):
                Estimator(channel_layout=layout)

    def test_directional_decisions_and_features_from_physical_ids(self):
        for adc, hlr, difference, bias, comparison in (
                ((3., 1., 2.), (2., 3., 1.), 1., -256., .5),
                ((1., 3., 2.), (2., 1., 3.), -1., 256., -.5),
                ((1., 1., 3.), (3., 1., 1.), 0., 0., 0.)):
            for legacy_schema in (False, True):
                with self.subTest(adc=adc, legacy_schema=legacy_schema):
                    engine, estimator = CycleSteering(2.), Estimator()
                    for i in range(11):
                        t = i * .05
                        message = physical_batch(t, adc, legacy_schema=legacy_schema)
                        self.assertEqual(adapt(message)['volts'], list(adc))
                        self.assertTrue(engine.push(message, 100. + t, t))
                        if i == 0:
                            engine.begin(100., 2., 100.)
                        result = estimator.push(message, t)
                    decision = engine.evaluate(100.5)
                    self.assertEqual((decision.head_mean, decision.left_mean, decision.right_mean), hlr)
                    self.assertEqual(decision.difference, difference)
                    self.assertEqual(decision.bias, bias)
                    self.assertAlmostEqual(result['comparison']['left_right_normalized_difference'], comparison)
                    head, left, right = hlr
                    self.assertAlmostEqual(result['features'][0], head / sum(hlr))
                    self.assertAlmostEqual(result['features'][1], left / sum(hlr))
                    self.assertEqual(result['hydrophone_channels'], {'head': 2, 'left': 0, 'right': 1})
                    self.assertTrue(result['comparison']['reliable'])

    def test_front_signal_cannot_rescue_missing_side_signals(self):
        engine = CycleSteering(2.)
        self.assertFalse(engine.push(physical_batch(0., (0., 0., 3.)), 100., 0.))
        self.assertEqual(engine.latest_error, 'low_signal')
        # Left alone is sufficient even when both old side-channel indices are zero.
        self.assertTrue(engine.push(physical_batch(.05, (3., 0., 0.)), 100.05, .05))

    def test_raw_cycle_means_keep_adc_order_without_affecting_direction(self):
        engine = CycleSteering(2.)
        for i in range(11):
            t = i * .05
            self.assertTrue(engine.push(physical_batch(t, (3., 1., 2.), (.25, .5, .75)), 100. + t, t))
            if i == 0:
                engine.begin(100., 2., 100.)
        decision = engine.evaluate(100.5)
        self.assertEqual(decision.raw_means, (.25, .5, .75))
        self.assertEqual((decision.head_mean, decision.left_mean, decision.right_mean), (2., 3., 1.))

    def test_legacy_wiring_produces_equivalent_features_and_predictions(self):
        from robotic_fish_sonic.acoustic_core import Model
        model = Model(Path(__file__).with_name('fixture_model.json'))
        for legacy_schema in (False, True):
            current = Estimator(model)
            previous = Estimator(model, channel_layout='head_left_right')
            for i in range(12):
                t = 10. + i * .05
                a = current.push(physical_batch(t, (3., 1., 2.), legacy_schema=legacy_schema), t)
                b = previous.push(physical_batch(t, (2., 3., 1.), legacy_schema=legacy_schema), t)
                self.assertEqual(a['features'], b['features'])
                self.assertEqual(a['probabilities'], b['probabilities'])
                self.assertEqual(a['comparison'], b['comparison'])

    def test_recalibration_uses_physical_adc_curves_before_direction_mapping(self):
        path = Path(__file__).resolve().parents[1]
        spec = importlib.util.spec_from_file_location('mapping_replay', path / 'scripts/replay_quadrants.py')
        replay = importlib.util.module_from_spec(spec)
        spec.loader.exec_module(replay)
        calibration = AdcCalibration.load(
            path / 'config/calibration/adc_independent_transfer_20261114_battery_common_5mvpp.json')
        raw = (.25, .5, .75)
        message = physical_batch(10., (0., 0., 0.), raw)
        original = [(s.channel_id, s.volt_raw, s.timestamp) for s in message.samples]
        replay.recalibrate(message, calibration)
        expected, _ = calibration.apply(raw)
        self.assertEqual(adapt(message)['volts'], list(expected))
        self.assertEqual([(s.channel_id, s.volt_raw, s.timestamp) for s in message.samples], original)
        result = Estimator().push(message, 10.)
        self.assertAlmostEqual(result['features'][0], expected[2] / sum(expected))
        self.assertAlmostEqual(result['features'][1], expected[0] / sum(expected))


if __name__ == '__main__':
    unittest.main()
