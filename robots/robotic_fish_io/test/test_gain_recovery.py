"""Opt-in fixed-mode recovery tests; no hardware or ROS node required."""
import sys
from pathlib import Path
import unittest
sys.path.insert(0, str(Path(__file__).resolve().parents[1] / 'src'))
from robotic_fish_io.agc_controller import GainController


class RecoveryTests(unittest.TestCase):
    def controller(self, **kw):
        args = dict(mode='fixed', fixed_target_v=1.0, fixed_auto_recovery=True,
                    fixed_recovery_wait_s=2., recovery_sample_timeout_s=.2)
        args.update(kw)
        c = GainController(**args)
        self.assertEqual(c.observe([4.0, 0.1, 0.1], 1., 0.), .9)
        c.observe([.1, .1, .1], .9, .05)
        return c

    def feed(self, c, start=0.1, stop=2.0, current=.9):
        for tick in range(round(start*100), round(stop*100)+1, 10):
            self.assertIsNone(c.observe([.1]*3, current, tick/100))

    def test_wait_then_slow_steps_and_target_cap(self):
        c = self.controller()
        self.feed(c)
        self.assertEqual(c.observe([.1]*3, .9, 2.11), .91)
        self.feed(c, 2.2, 2.5, .91)
        self.assertIsNone(c.observe([.1]*3, .91, 2.62))
        self.feed(c, 2.7, 3.1, .995)
        self.assertIsNone(c.observe([.1]*3, 1., 3.2))
        self.assertEqual(c.last_action, 'fixed_recovery_holding')

    def test_repeated_overrange_immediately_reduces(self):
        c = self.controller(); self.feed(c)
        self.assertEqual(c.observe([.1]*3, .9, 2.11), .91)
        self.assertEqual(c.observe([4.0, .1, .1], .91, 2.12), .81)
        self.assertIsNone(c.recovery_since_s)

    def test_last_step_does_not_exceed_target(self):
        c = self.controller(); self.feed(c)
        self.assertEqual(c.observe([.1]*3, .994, 2.11), 1.)

    def test_backward_clock_restarts_wait(self):
        c = self.controller(); self.feed(c)
        self.assertIsNone(c.observe([.1]*3, .9, 1.))
        self.assertEqual(c.recovery_since_s, 1.)

    def test_gap_restarts_wait(self):
        c = self.controller(); self.feed(c)
        self.assertIsNone(c.observe([.1]*3, .9, 10.))
        self.assertEqual(c.recovery_since_s, 10.)

    def test_nonfinite_and_negative_restart_wait(self):
        for value in [float('nan'), float('inf'), -.1]:
            c = self.controller(); self.feed(c)
            with self.assertRaises(ValueError): c.observe([value, .1, .1], .9, 2.05)
            self.assertIsNone(c.recovery_since_s)

    def test_high_but_not_overrange_resets_wait(self):
        c = self.controller(); self.feed(c)
        self.assertIsNone(c.observe([1.1, .1, .1], .9, 2.1))
        self.assertIsNone(c.recovery_since_s)
        self.assertIsNone(c.observe([.1]*3, .9, 2.2))

    def test_disabled_keeps_old_latch(self):
        c = self.controller(fixed_auto_recovery=False)
        self.feed(c, .1, 5.)
        self.assertEqual(c.last_action, 'fixed_limited')

    def test_off_and_mode_switch_clear_wait(self):
        c = self.controller(); self.feed(c)
        c.set_mode('off')
        self.assertIsNone(c.observe([.1]*3, .9, 2.1))
        c.set_mode('fixed')
        self.assertIsNone(c.observe([.1]*3, .9, 2.2))

    def test_error_reset_and_repeated_timestamp(self):
        c = self.controller(); self.feed(c)
        c.reset_counts()
        self.assertIsNone(c.observe([.1]*3, .9, 2.1))
        self.assertIsNone(c.observe([.1]*3, .9, 2.1))
        self.assertEqual(c.recovery_since_s, 2.1)

    def test_invalid_config(self):
        for opts in [dict(fixed_auto_recovery='true'), dict(fixed_recovery_wait_s=0),
                     dict(fixed_recovery_max_adc_v=3.3), dict(fixed_recovery_step_v=.2),
                     dict(fixed_recovery_interval_s=.001)]:
            with self.subTest(opts=opts), self.assertRaises(ValueError): self.controller(**opts)


if __name__ == '__main__': unittest.main()
