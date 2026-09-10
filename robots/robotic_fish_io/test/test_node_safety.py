"""Execute real node methods with isolated ROS/time/service doubles, not rostest."""
import ast
import math
from pathlib import Path
import sys
import threading
import time
from types import SimpleNamespace
import unittest

ROOT = Path(__file__).resolve().parents[1]
sys.path.insert(0, str(ROOT / 'src'))
from robotic_fish_io.agc_controller import GainController
from robotic_fish_io import dac_driver
from robotic_fish_io.adc_limits import MAX_RAW, MAX_VOLTAGE_V


class Stamp(float):
    def __sub__(self, other):
        return SimpleNamespace(to_sec=lambda: float(self)-float(other))

    @staticmethod
    def now():
        return Stamp(100)


def node_class(filename, name):
    tree = ast.parse((ROOT/'scripts'/filename).read_text())
    ns = dict(math=math, threading=threading, time=time, dac_driver=dac_driver,
              MAX_RAW=MAX_RAW, MAX_VOLTAGE_V=MAX_VOLTAGE_V,
              rospy=SimpleNamespace(Time=Stamp, loginfo=lambda *a: None,
                                    logerr_throttle=lambda *a: None,
                                    logwarn_throttle=lambda *a: None))
    exec(compile(ast.Module(body=[n for n in tree.body if isinstance(n, ast.ClassDef)],
                            type_ignores=[]), filename, 'exec'), ns)
    return ns[name]


class NodeSafetyTests(unittest.TestCase):
    def make_node(self, mode='fixed'):
        n = node_class('agc_node.py', 'GainControlNode').__new__(
            node_class('agc_node.py', 'GainControlNode'))
        n.controller = GainController(mode=mode, fixed_target_v=2., consecutive_samples=1)
        n.lock = threading.RLock()
        n.command_lock = threading.Lock()
        n.last_sample_stamp = Stamp(0)
        n.last_adc_monotonic_s = None
        n.latest_batch_valid = False
        n.adc_timeout = 1.
        n.current_dac_voltage = 1.
        n.adjustments = n.error_count = 0
        n.last_error = ''
        n._publish_state = lambda: None
        n.calls = []
        def apply(value):
            n.calls.append(value)
            return value
        n._set_dac_voltage = apply
        return n

    def message(self, stamp=99.9, values=(.1, .1, .1)):
        return SimpleNamespace(header=SimpleNamespace(stamp=Stamp(stamp)), samples=[
            SimpleNamespace(channel=i, raw=int(v*8000) if math.isfinite(v) else 0,
                            voltage=v, header=SimpleNamespace(stamp=Stamp(stamp-.01)),
                            gain_control_voltage=1., gain_control_voltage_valid=True)
            for i, v in enumerate(values)])

    def test_stale_fixed_and_closed_loop(self):
        for mode in ('fixed', 'closed_loop'):
            n = self.make_node(mode)
            n._adc_callback(self.message(stamp=1.))
            self.assertEqual(n.calls, [])
            self.assertFalse(n._adc_is_valid())

    def test_fresh_can_control(self):
        n = self.make_node()
        n._adc_callback(self.message())
        self.assertEqual(n.calls, [1.05])
        self.assertTrue(n._adc_is_valid())

    def test_old_nested_sample_rejected(self):
        n = self.make_node()
        msg = self.message()
        msg.samples[1].header.stamp = Stamp(1.)
        n._adc_callback(msg)
        self.assertEqual(n.calls, [])

    def test_repeated_and_future_rejected(self):
        n = self.make_node()
        n.last_sample_stamp = Stamp(99.9)
        for stamp in (99.9, 101.):
            n._adc_callback(self.message(stamp=stamp))
        self.assertEqual(n.calls, [])

    def test_missing_channel_does_not_mask_protection(self):
        n = self.make_node()
        msg = self.message(values=(4.05, .1, .1))
        msg.samples.pop()
        n._adc_callback(msg)
        self.assertEqual(n.calls, [.9])
        self.assertFalse(n._adc_is_valid())

    def test_raw_saturation_overrides_voltage(self):
        n = self.make_node()
        msg = self.message()
        msg.samples[1].raw = MAX_RAW
        n._adc_callback(msg)
        self.assertEqual(n.calls, [.9])

    def test_unknown_dac_and_invalid_data_no_initialization(self):
        n = self.make_node()
        n.current_dac_voltage = None
        n._adc_callback(self.message(stamp=1.))
        self.assertEqual(n.calls, [])

    def test_nonfinite_dac_invalidates_state(self):
        n = self.make_node()
        n._dac_state_callback(SimpleNamespace(data=float('nan')))
        self.assertIsNone(n.current_dac_voltage)

    def test_other_dac_channel_rejected_before_io(self):
        cls = node_class('dac_node.py', 'DacNode')
        n = cls.__new__(cls)
        n.default_channel = 1
        with self.assertRaises(ValueError):
            n._apply_voltage_locked(1., 2)


if __name__ == '__main__':
    unittest.main()
