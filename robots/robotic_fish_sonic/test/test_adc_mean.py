"""Window boundaries, reset behavior and actual ADC publication integration."""
import ast
import math
from pathlib import Path
from types import SimpleNamespace as NS
import unittest

from robotic_fish_sonic.adc_mean import RawVoltageMean


class MeanTests(unittest.TestCase):
    def feed(self, mean, start=0., gain=.8, valid=True):
        return [mean.update(start + i / 20., float(i), valid, gain) for i in range(21)]

    def test_full_window_and_eviction(self):
        mean = RawVoltageMean(.2)
        results = self.feed(mean)
        self.assertTrue(all(not ready for _, ready in results[:-1]))
        self.assertEqual(results[-1], (10., True))
        value, ready = mean.update(1.125, 30., True, .8)
        # Only samples at .15 through 1.0 plus the new reading remain.
        self.assertAlmostEqual(value, (sum(range(3, 21)) + 30.) / 19)
        self.assertTrue(ready)

    def test_resets_on_gain_validity_gap_and_clock_changes(self):
        for stamp, valid, gain in [(1.05, True, .9), (1.05, False, 0.),
                                   (1.3, True, .8), (1., True, .8), (.5, True, .8)]:
            with self.subTest(stamp=stamp, valid=valid, gain=gain):
                mean = RawVoltageMean(.2)
                self.feed(mean)
                self.assertEqual(mean.update(stamp, 50., valid, gain), (50., False))

    def test_unknown_gain_and_invalid_input(self):
        mean = RawVoltageMean(.2)
        self.assertFalse(self.feed(mean, valid=False)[-1][1])
        self.assertEqual(mean.update(1.05, 7., True, .8), (7., False))
        for stamp, voltage in [(float('nan'), 1.), (2., float('inf'))]:
            value, ready = mean.update(stamp, voltage, True, .8)
            self.assertTrue(math.isnan(value))
            self.assertFalse(ready)
        self.assertEqual(mean.update(3., 4., True, .8), (4., False))

    def test_node_channels_raw_values_and_disconnect_reset(self):
        path = Path(__file__).resolve().parents[1] / 'scripts/adc_node.py'
        tree = ast.parse(path.read_text())
        ns = {}
        exec(compile(ast.Module(body=[n for n in tree.body if isinstance(n, ast.ClassDef)],
                                type_ignores=[]), str(path), 'exec'), ns)
        node = ns['AdcNode'].__new__(ns['AdcNode'])
        node.raw_means = {i: RawVoltageMean(.2) for i in range(3)}
        node.bus = None
        for tick in range(21):
            samples = [NS(channel_id=i, timestamp=NS(to_sec=lambda: tick / 20.),
                          volt_raw=float(i + 1), volt_cali=99.,
                          status_dac_feedback=True, dac_volt=.8) for i in range(3)]
            node._update_means(samples)
        self.assertEqual([s.volt_raw_mean_1s for s in samples], [1., 2., 3.])
        self.assertTrue(all(s.mean_1s_ready for s in samples))
        node._close_bus()
        node._update_means(samples)
        self.assertFalse(any(s.mean_1s_ready for s in samples))


if __name__ == '__main__':
    unittest.main()
