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
            SimpleNamespace(channel_id=i, adc_code=int(v*8000) if math.isfinite(v) else 0,
                            volt_raw=v, timestamp=Stamp(stamp-.01),
                            dac_volt=1., status_dac_feedback=True)
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
        msg.samples[1].timestamp = Stamp(1.)
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
        msg.samples[1].adc_code = MAX_RAW
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


class AdcSchemaTests(unittest.TestCase):
    def make_node(self):
        cls = node_class('adc_node.py', 'AdcNode')
        node = cls.__new__(cls)
        fields = [line.split()[1] for line in
                  (ROOT / 'msg/AdcSample.msg').read_text().splitlines()
                  if line.strip() and not line.startswith('#')]
        # Slots make obsolete message-field writes fail, as generated ROS messages do.
        message_type = type('Sample', (), {'__slots__': fields})
        cls._sample_channel.__globals__['AdcSample'] = message_type
        return node, message_type

    def test_timestamp_is_read_completion_and_dac_is_start_snapshot(self):
        n, _ = self.make_node()
        n.bus = None
        n.address = 0x48
        n.data_rate = 128
        n.timeout = .1
        n.poll_interval = .001
        n.frame_id = 'ads1115'
        n.sample_sequence = 7
        n.gain_control_state = (True, .8)
        n._ros_time_from_anchor = lambda anchor, mono, value: value
        def read(*args, **kwargs):
            n.gain_control_state = (True, .9)
            return SimpleNamespace(channel=1, raw=16000, voltage=2.,
                                   started_ns=100, completed_ns=300)
        n._sample_channel.__globals__['adc_driver'] = SimpleNamespace(read_timed_channel=read)
        msg = n._sample_channel(1)
        self.assertEqual(msg.timestamp, 300)
        self.assertEqual(msg.serial_num, 7)
        self.assertEqual(msg.channel_id, 1)
        self.assertEqual(msg.adc_code, 16000)
        self.assertEqual(msg.dac_volt, .8)
        self.assertTrue(msg.status_dac_feedback)
        n.gain_control_state = (False, 0.)
        msg = n._sample_channel(1)
        self.assertTrue(math.isnan(msg.dac_volt))
        self.assertFalse(msg.status_dac_feedback)

    def test_calibration_difference_and_whole_group_fallback_then_recovery(self):
        from robotic_fish_io.adc_calibration import AdcCalibration
        n, message_type = self.make_node()
        n.calibration_enabled = True
        n.calibration = AdcCalibration.load(ROOT / 'config/calibration/'
            'adc_independent_transfer_20260901_171408_calbri05_raw.json')
        samples = [message_type() for _ in range(3)]
        for sample in samples:
            sample.volt_raw = 1.
        n._apply_calibration(samples)
        for sample in samples:
            self.assertTrue(sample.status_cali)
            self.assertAlmostEqual(sample.diff_cali, sample.volt_cali - sample.volt_raw)
            self.assertEqual(sample.cali_id, n.calibration.calibration_id)
        samples[1].volt_raw = 3.7
        n._apply_calibration(samples)
        for sample in samples:
            self.assertFalse(sample.status_cali)
            self.assertEqual(sample.volt_cali, sample.volt_raw)
            self.assertTrue(math.isnan(sample.diff_cali))
            self.assertEqual(sample.cali_id, '')
        samples[1].volt_raw = 1.
        n._apply_calibration(samples)
        self.assertTrue(all(sample.status_cali for sample in samples))


if __name__ == '__main__':
    unittest.main()
