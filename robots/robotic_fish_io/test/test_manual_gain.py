"""Manual joystick edges, bounded targets, and safety integration."""
import time
from types import SimpleNamespace
import unittest

import test_node_safety as node_tests
from robotic_fish_io.manual_gain import ManualGainInput
from robotic_fish_io.agc_controller import GainController


def joy(horizontal=0, vertical=0):
    return SimpleNamespace(axes=[0.] * 9 + [horizontal, vertical])


class ManualGainTests(unittest.TestCase):
    def node(self, mode="manual"):
        n = node_tests.NodeSafetyTests().make_node(mode)
        n.controller.fixed_target_v = 1.
        n.latest_batch_valid = True
        n.last_adc_monotonic_s = time.monotonic()
        n._publish_config = lambda: None
        return n

    def press(self, n, horizontal=0, vertical=0):
        n._joy_callback(joy())
        n._joy_callback(joy(horizontal, vertical))

    def test_edges_steps_and_voltage_direction(self):
        n = self.node()
        self.press(n, horizontal=1)
        self.assertEqual(n.manual_input.step_v, .02)
        self.press(n, vertical=1)
        self.assertEqual(n.controller.fixed_target_v, 1.02)
        n._joy_callback(joy(vertical=1))
        self.assertEqual(n.controller.fixed_target_v, 1.02)
        self.press(n, vertical=-1)
        self.assertEqual(n.controller.fixed_target_v, 1.)
        self.press(n, horizontal=-1)
        self.assertEqual(n.manual_input.step_v, .01)
        self.assertEqual(n.calls, [])  # ADC callback alone executes hardware commands.
        n._adc_callback(node_tests.NodeSafetyTests().message())
        self.assertEqual(n.calls, [])
        self.press(n, vertical=1)
        n._adc_callback(node_tests.NodeSafetyTests().message(stamp=99.95))
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
            n.controller.fixed_target_v = target
            self.press(n, vertical=direction)
            self.assertEqual(n.controller.fixed_target_v, target)

    def test_nonmanual_modes_ignore_joystick(self):
        for mode in ("off", "fixed", "closed_loop"):
            n = self.node(mode)
            self.press(n, horizontal=1, vertical=1)
            self.assertEqual(n.controller.fixed_target_v, 1.)
            self.assertEqual(n.manual_input.step_v, .01)

    def test_invalid_axes_do_not_rearm_held_key(self):
        i = ManualGainInput()
        self.assertEqual(i.events(joy(vertical=1).axes), (0, 1))
        for axes in ([], [0.] * 10, joy(float("nan")).axes):
            with self.assertRaises(ValueError):
                i.events(axes)
        self.assertEqual(i.events(joy(vertical=1).axes), (0, 0))

    def test_blocked_input_does_not_queue_a_target(self):
        for field, value in (("latest_batch_valid", False),
                             ("current_dac_voltage", None), ("last_error", "failed")):
            n = self.node()
            setattr(n, field, value)
            self.press(n, vertical=1)
            self.assertEqual(n.controller.fixed_target_v, 1.)
        n = self.node()
        n.controller.safety_active = True
        self.press(n, vertical=1)
        self.assertEqual(n.controller.fixed_target_v, 1.)

    def test_protection_discards_target_and_requires_new_press(self):
        n = self.node()
        n.controller.fixed_target_v = 2.
        n.controller.fixed_auto_recovery = True
        n._adc_callback(node_tests.NodeSafetyTests().message(values=(4.05, .1, .1)))
        self.assertEqual(n.current_dac_voltage, .9)
        n._adc_callback(node_tests.NodeSafetyTests().message(stamp=99.95))
        self.assertTrue(n.controller.fixed_limited)
        self.assertEqual(n.controller.fixed_target_v, .9)
        self.assertIsNone(n.controller.observe([.1]*3, .9, time.monotonic()+10))
        self.press(n, vertical=-1)
        self.assertEqual(n.controller.fixed_target_v, .89)
        self.assertFalse(n.controller.fixed_limited)

    def test_manual_ramp_stale_and_safety_priority(self):
        c = GainController(mode="manual", fixed_target_v=1.)
        self.assertIsNone(c.observe([.1]*3, 0., 1., data_valid=False))
        self.assertEqual(c.observe([.1]*3, 0., 2.), .05)
        self.assertEqual(c.last_action, "manual_ramping_up")
        self.assertEqual(c.observe([4.05, .1, .1], .5, 2.01), .4)


if __name__ == "__main__":
    unittest.main()
