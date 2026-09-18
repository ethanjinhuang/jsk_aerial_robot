"""Safety and opt-in lateral window controller regressions (no ROS)."""
import sys
from pathlib import Path
import unittest

sys.path.insert(0, str(Path(__file__).resolve().parents[1] / 'src'))
from robotic_fish_io.agc_controller import GainController


class SafetyWindowTests(unittest.TestCase):
    def test_invalid_companion_does_not_mask_overrange(self):
        for bad in [float('nan'), -.000125]:
            for ch in range(3):
                c = GainController()
                v = [bad] * 3
                v[ch] = 4.095875
                self.assertEqual(c.observe(v, 1., 0.), .9)
                self.assertTrue(c.safety_active)

    def test_stale_never_increases_or_releases(self):
        for mode in ['manual', 'closed_loop']:
            c = GainController(mode=mode, manual_target_v=2., consecutive_samples=1)
            self.assertIsNone(c.observe([.1]*3, 1., 0., data_valid=False))
            self.assertEqual(c.observe([4.0]*3, 1., 1., data_valid=False), .9)
            self.assertIsNone(c.observe([.1]*3, .9, 2., data_valid=False))
            self.assertTrue(c.safety_active)

    def test_unreachable_threshold_rejected(self):
        with self.assertRaises(ValueError):
            GainController(safety_limit_v=4.096)

    def window(self):
        return GainController(mode='closed_loop', window_control=True,
                              window_high_fraction=.05, window_cooldown_s=2.)

    def fill(self, c, values, current=1., start=0., count=41):
        result = None
        for i in range(count):
            result = c.observe(values, current, start+i*.05)
        return result

    def test_head_does_not_set_normal_target(self):
        c = self.window()
        self.assertEqual(self.fill(c, [3.8, 1., .2]), 1.01)

    def test_stronger_lateral_mean_holds(self):
        c = self.window()
        self.assertIsNone(self.fill(c, [.1, 2.5, .1]))

    def test_frequent_high_lateral_decreases(self):
        c = self.window()
        self.assertEqual(self.fill(c, [.1, 3.2, .1]), .99)

    def test_isolated_peak_holds(self):
        c = self.window()
        self.fill(c, [.1]*3, count=40)
        self.assertIsNone(c.observe([.1, 3.2, .1], 1., 2.))
        self.assertEqual(c.last_action, 'window_peak_hold')

    def test_new_dac_requires_new_window(self):
        c = self.window()
        self.assertEqual(self.fill(c, [.1]*3), 1.01)
        self.assertIsNone(self.fill(c, [.1]*3, current=1.01, start=2.05, count=39))

    def test_gap_requires_new_window(self):
        c = self.window()
        self.fill(c, [.1]*3, count=40)
        self.assertIsNone(c.observe([.1]*3, 1., 3.))

    def test_head_safety_overrides_window(self):
        c = self.window()
        self.assertEqual(c.observe([4., .1, .1], 1., 0.), .9)

    def test_high_fraction_must_be_explicit(self):
        with self.assertRaises(ValueError):
            GainController(window_control=True)


if __name__ == '__main__':
    unittest.main()
