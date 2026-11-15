"""Manual joystick edges, bounded targets, and safety integration."""
import time
from pathlib import Path
import sys
from types import SimpleNamespace
import unittest
from unittest.mock import Mock, patch

sys.path.insert(0, str(Path(__file__).resolve().parent))
import test_node_safety as node_tests
from robotic_fish_sonic.manual_gain import ManualGainInput
from robotic_fish_sonic.agc_controller import GainController


def joy(horizontal=0, vertical=0):
    return SimpleNamespace(axes=[0.] * 9 + [horizontal, vertical])


class ManualGainTests(unittest.TestCase):
    def node(self, mode="manual"):
        n = node_tests.NodeSafetyTests().make_node(mode)
        n.controller.manual_target_v = 1.
        n.latest_sample_group_valid = True
        n.last_valid_adc_received_monotonic_s = time.monotonic()
        n._publish_config = lambda: None
        return n

    def press(self, n, horizontal=0, vertical=0):
        n._on_joy(joy())
        n._on_joy(joy(horizontal, vertical))

    def test_removed_fixed_mode_is_rejected(self):
        with self.assertRaises(ValueError):
            GainController(mode="fixed")
        c = GainController(mode="manual")
        with self.assertRaises(ValueError):
            c.set_mode("fixed")
        self.assertEqual(c.mode, "manual")

    def test_edges_steps_and_voltage_direction(self):
        n = self.node()
        self.press(n, horizontal=1)
        self.assertEqual(n.manual_input.step_v, .02)
        self.press(n, vertical=1)
        self.assertEqual(n.controller.manual_target_v, 1.02)
        n._on_joy(joy(vertical=1))
        self.assertEqual(n.controller.manual_target_v, 1.02)
        self.press(n, vertical=-1)
        self.assertEqual(n.controller.manual_target_v, 1.)
        self.press(n, horizontal=-1)
        self.assertEqual(n.manual_input.step_v, .01)
        self.assertEqual(n.calls, [])  # Only the manual timer executes normal commands.
        n._on_adc_samples(node_tests.NodeSafetyTests().message())
        self.assertEqual(n.calls, [])
        self.press(n, vertical=1)
        n._on_adc_samples(node_tests.NodeSafetyTests().message(stamp=99.95))
        self.assertEqual(n.calls, [])
        n._manual_callback(None)
        self.assertEqual(n.calls, [1.01])

    def test_step_and_voltage_bounds(self):
        n = self.node()
        for _ in range(20):
            self.press(n, horizontal=1)
        self.assertEqual(n.manual_input.step_v, .1)
        for _ in range(20):
            self.press(n, horizontal=-1)
        self.assertEqual(n.manual_input.step_v, .01)
        for target, direction in ((5., 1), (0., -1)):
            n.controller.manual_target_v = target
            self.press(n, vertical=direction)
            self.assertEqual(n.controller.manual_target_v, target)

    def test_nonmanual_modes_ignore_joystick(self):
        for mode in ("off", "closed_loop"):
            n = self.node(mode)
            self.press(n, horizontal=1, vertical=1)
            self.assertEqual(n.controller.manual_target_v, 1.)
            self.assertEqual(n.manual_input.step_v, .01)

    def test_invalid_axes_do_not_rearm_held_key(self):
        i = ManualGainInput()
        self.assertEqual(i.events(joy(vertical=1).axes), (0, 1))
        for axes in ([], [0.] * 10, joy(float("nan")).axes):
            with self.assertRaises(ValueError):
                i.events(axes)
        self.assertEqual(i.events(joy(vertical=1).axes), (0, 0))

    def test_manual_target_independent_of_adc_or_dac_initialization(self):
        for field, value in (("latest_sample_group_valid", False),
                             ("confirmed_dac_voltage_v", None), ("last_error", "failed")):
            n = self.node()
            setattr(n, field, value)
            self.press(n, vertical=1)
            self.assertEqual(n.controller.manual_target_v, 1.01)
        n = self.node()
        self.press(n, vertical=1)
        n.confirmed_dac_voltage_v = 1.
        n.latest_sample_group_valid = False
        n._manual_callback(None)
        self.assertEqual(n.calls, [1.01])

    def test_active_protection_does_not_queue_a_target(self):
        n = self.node()
        n.controller.safety_active = True
        self.press(n, vertical=1)
        self.assertEqual(n.controller.manual_target_v, 1.)

    def test_protection_discards_target_and_requires_new_press(self):
        n = self.node()
        n.controller.manual_target_v = 2.
        n._on_adc_samples(node_tests.NodeSafetyTests().message(values=(4.05, .1, .1)))
        self.assertEqual(n.confirmed_dac_voltage_v, .9)
        n._on_adc_samples(node_tests.NodeSafetyTests().message(stamp=99.95))
        self.assertTrue(n.controller.manual_limited)
        self.assertEqual(n.controller.manual_target_v, .9)
        self.assertIsNone(n.controller.observe([.1]*3, .9, time.monotonic()+10))
        self.press(n, vertical=-1)
        self.assertEqual(n.controller.manual_target_v, .89)
        self.assertFalse(n.controller.manual_limited)

    def test_manual_timer_and_safety_priority(self):
        c = GainController(mode="manual", manual_target_v=1.)
        self.assertIsNone(c.observe([.1]*3, 0., 1., data_valid=False))
        self.assertIsNone(c.observe([.1]*3, 0., 2.))
        self.assertEqual(c.manual_tick(0., 2.), .05)
        self.assertEqual(c.last_action, "manual_ramping_up")
        self.assertEqual(c.observe([4.05, .1, .1], .5, 2.01), .4)
        self.assertIsNone(c.manual_tick(.4, 3.))

    def test_manual_initializes_without_any_adc_then_ramps(self):
        n = self.node()
        n.confirmed_dac_voltage_v = None
        n.latest_sample_group_valid = False
        n._manual_callback(None)
        self.assertEqual(n.calls, [0.])
        n._manual_callback(None)
        self.assertEqual(n.calls, [0., .05])
        self.assertFalse(n._adc_is_valid())

    def test_bad_or_stale_adc_cannot_block_manual_ramp(self):
        fixtures = node_tests.NodeSafetyTests()
        for msg in (fixtures.message(values=(-.005, -.006, -.007)),
                    fixtures.message(values=(float('nan'), .1, .1)),
                    fixtures.message(stamp=1.)):
            with self.subTest(msg=msg):
                n = self.node()
                n.controller.manual_target_v = 1.1
                n._on_adc_samples(msg)
                self.assertFalse(n._adc_is_valid())
                self.assertEqual(n.last_error, '')
                self.assertTrue(n.adc_error)
                self.assertEqual(n.calls, [])
                n._manual_callback(None)
                self.assertEqual(n.calls, [1.05])

    def test_closed_loop_still_rejects_bad_or_missing_adc(self):
        n = self.node('closed_loop')
        n._manual_callback(None)
        self.assertEqual(n.calls, [])
        n._on_adc_samples(node_tests.NodeSafetyTests().message(values=(-.005, .1, .1)))
        n._manual_callback(None)
        self.assertEqual(n.calls, [])
        self.assertTrue(n.last_error)

    def test_timer_respects_interval_and_retries_failed_commands(self):
        n = self.node()
        n.controller.manual_target_v = 1.1
        with patch('time.monotonic', return_value=1.):
            n._set_dac_voltage = Mock(side_effect=OSError('DAC disconnected'))
            n._manual_callback(None)
            self.assertIn('DAC disconnected', n.last_error)
            self.assertEqual(n.confirmed_dac_voltage_v, 1.)
            n._set_dac_voltage = lambda voltage: n.calls.append(voltage) or voltage
            n._manual_callback(None)
            self.assertEqual(n.calls, [])
        with patch('time.monotonic', return_value=1.11):
            n._manual_callback(None)
            self.assertEqual(n.calls, [1.05])
            self.assertEqual(n.last_error, '')

    def test_shutdown_stops_timer_and_prevents_late_writes(self):
        n = self.node()
        n._shutdown()
        n.manual_timer.shutdown.assert_called_once()
        n.controller.manual_target_v = 2.
        n._manual_callback(None)
        self.assertEqual(n.calls, [])
        n._on_adc_samples(node_tests.NodeSafetyTests().message(values=(4.05, .1, .1)))
        self.assertEqual(n.calls, [])


if __name__ == "__main__":
    unittest.main()
