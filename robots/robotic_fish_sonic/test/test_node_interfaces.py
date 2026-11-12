"""Constructor/parameter and actual ADC publication regressions; no hardware."""
import importlib.util
import json
from pathlib import Path
import sys
import unittest
from unittest.mock import Mock, patch
from contextlib import ExitStack

import genpy
import rospy
# Load the catkin package before pure-driver fixtures prepend the source tree.
# This retains the generated msg/srv package paths in isolated nose processes.
import robotic_fish_sonic.msg

ROOT = Path(__file__).resolve().parents[1]
sys.path.insert(0, str(ROOT / 'src'))
sys.path.insert(0, str(ROOT / 'test'))

from robotic_fish_sonic.parameters import get_compatible_param
from test_drivers import FakeAdcBus


def load_node(name):
    spec = importlib.util.spec_from_file_location('interface_test_' + name,
                                                 ROOT / 'scripts' / (name + '.py'))
    module = importlib.util.module_from_spec(spec)
    spec.loader.exec_module(module)
    return module


class NodeInterfaceTests(unittest.TestCase):
    def setUp(self):
        self.parameters = {'~adc/calibration/enabled': False}
        self.published = {}
        self.stack = ExitStack()
        self.addCleanup(self.stack.close)
        self.stack.enter_context(patch.object(rospy, 'has_param',
                                              side_effect=lambda key: key in self.parameters))
        self.stack.enter_context(patch.object(rospy, 'get_param',
                                              side_effect=lambda key, default=None: self.parameters.get(key, default)))
        self.stack.enter_context(patch.object(rospy, 'Publisher', side_effect=self.publisher))
        self.stack.enter_context(patch.object(rospy.Time, 'now', return_value=genpy.Time(100)))
        for name in ('Subscriber', 'Service', 'ServiceProxy', 'Timer', 'on_shutdown',
                     'loginfo', 'logwarn', 'logerr', 'logerr_throttle', 'logwarn_throttle', 'sleep'):
            self.stack.enter_context(patch.object(rospy, name))

    def publisher(self, topic, message_type, **kwargs):
        messages = self.published.setdefault(topic, [])
        result = Mock()
        result.publish.side_effect = messages.append
        result.message_type = message_type
        result.options = kwargs
        return result

    def config(self, device):
        records = self.published['/robotic_fish/gain_control/config']
        return json.loads(next(m.config_json for m in reversed(records) if m.device == device))

    def adc(self):
        return load_node('adc_node').AdcNode()

    def dac(self):
        module = load_node('dac_node')
        with patch.object(module.DacNode, '_open_locked'):
            return module.DacNode()

    def gain(self):
        return load_node('agc_node').GainControlNode()

    def test_canonical_parameter_precedence_and_explicit_false_zero_empty(self):
        for value in (False, 0, ''):
            self.parameters.update(new=value, old=123)
            self.assertEqual(get_compatible_param('new', 9, 'old'), value)
        del self.parameters['new']
        self.assertEqual(get_compatible_param('new', 9, 'old'), 123)
        del self.parameters['old']
        self.assertEqual(get_compatible_param('new', 9, 'old'), 9)

    def test_adc_legacy_and_canonical_parameters_preserve_effective_snapshot(self):
        legacy = {'~adc/bus': 2, '~adc/address': 0x49, '~adc/channels': [0, 1, 2],
                  '~adc/data_rate': 250, '~adc/channel_rate': 10., '~adc/conversion_timeout': .2,
                  '~adc/ready_poll_interval': .002, '~adc/reconnect_interval': 2.,
                  '~adc/frame_id': 'test_adc', '~adc/gain_control_topic': '/test/dac'}
        canonical = dict(zip(
            ('~adc/i2c_bus_id', '~adc/i2c_address', '~adc/channel_ids',
             '~adc/conversion_rate_sps', '~adc/sample_group_rate_hz', '~adc/conversion_timeout_s',
             '~adc/ready_poll_interval_s', '~adc/reconnect_interval_s', '~adc/device_id',
             '~adc/dac_state_topic'), legacy.values()))
        self.parameters.update(legacy)
        old_node = self.adc()
        old_config = self.config('adc')
        for key in legacy:
            del self.parameters[key]
        self.parameters.update(canonical)
        new_node = self.adc()
        self.assertEqual(self.config('adc'), old_config)
        self.assertEqual(old_node.sample_group_rate_hz, new_node.sample_group_rate_hz)
        self.assertEqual(old_config['settings']['channel_rate'], 10.)
        self.assertNotIn('sample', old_config['topics'])
        self.assertNotIn('/robotic_fish/adc/sample', self.published)

    def test_dac_legacy_and_canonical_parameters_preserve_effective_snapshot(self):
        legacy = {'~dac/port': '/test/serial', '~dac/baud': 38400, '~dac/channel': 2,
                  '~dac/timeout': .2, '~dac/safe_voltage': .1,
                  '~dac/command_timeout': 3., '~dac/reconnect_interval': 2.}
        canonical = dict(zip(
            ('~dac/serial_port', '~dac/serial_baud_rate', '~dac/channel_id',
             '~dac/serial_timeout_s', '~dac/safe_voltage_v', '~dac/command_timeout_s',
             '~dac/reconnect_interval_s'), legacy.values()))
        self.parameters.update(legacy)
        self.dac()
        old_config = self.config('dac')
        for key in legacy:
            del self.parameters[key]
        self.parameters.update(canonical)
        self.dac()
        self.assertEqual(self.config('dac'), old_config)
        self.assertEqual(old_config['settings']['safe_voltage'], .1)

    def test_gain_legacy_launch_overrides_and_canonical_override(self):
        self.parameters.update({'~gain_control/mode': 'manual',
                                '~gain_control/manual_voltage': .2,
                                '~gain_control/adc_safety_limit_v': 4.,
                                '~gain_control/adc_timeout': .7})
        node = self.gain()
        self.assertEqual(node.controller.manual_target_v, .2)
        self.assertEqual(node.adc_timeout_s, .7)
        self.assertEqual(self.config('gain_control')['settings']['safety_limit_v'], 4.)
        self.parameters['~gain_control/manual_target_voltage_v'] = 0.
        self.parameters['~gain_control/adc_timeout_s'] = .8
        node = self.gain()
        self.assertEqual(node.controller.manual_target_v, 0.)
        self.assertEqual(node.adc_timeout_s, .8)
        self.assertEqual(self.config('gain_control')['settings']['manual_target_v'], 0.)

    def test_adc_run_publishes_one_complete_group_and_keeps_wire_fields(self):
        self.parameters['~adc/publish_array'] = False  # Obsolete flag cannot silence the only output.
        node = self.adc()
        node.bus = FakeAdcBus(raw_by_channel={0: 8000, 1: 16000, 2: 24000})
        node.dac_voltage_state = (True, .8)
        with patch.object(rospy, 'is_shutdown', side_effect=[False, True]):
            node.run()
        messages = self.published['/robotic_fish/adc/samples']
        self.assertEqual(len(messages), 1)
        self.assertEqual([s.channel_id for s in messages[0].samples], [0, 1, 2])
        self.assertEqual([s.serial_num for s in messages[0].samples], [0, 1, 2])
        self.assertEqual([s.volt_raw for s in messages[0].samples], [1., 2., 3.])
        self.assertTrue(all(s.status_dac_feedback and s.dac_volt == .8 for s in messages[0].samples))
        self.assertTrue(all(not s.status_cali and s.volt_cali == s.volt_raw for s in messages[0].samples))
        self.assertNotIn('/robotic_fish/adc/sample', self.published)

    def test_partial_conversion_failure_does_not_publish_a_group(self):
        node = self.adc()
        node.bus = FakeAdcBus()
        first = Mock()
        with patch.object(node, '_sample_channel', side_effect=[first, OSError('test I2C failure')]), \
                patch.object(rospy, 'is_shutdown', side_effect=[False, True]):
            node.run()
        self.assertEqual(self.published['/robotic_fish/adc/samples'], [])
        self.assertEqual(node.error_count, 1)
        self.assertIsNone(node.bus)


if __name__ == '__main__':
    unittest.main()
