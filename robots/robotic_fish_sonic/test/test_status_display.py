"""Snapshot readability, uncolored transport, and terminal color fallback."""
import importlib.util
import io
from pathlib import Path
import re
from types import SimpleNamespace as NS
import unittest
from unittest.mock import Mock, patch

from robotic_fish_sonic.status_display import format_status, write_snapshot


def snapshot(**changes):
    data = dict(mode='tracking', status='active', reason='tracking_button', stamp=123.45,
                direction='left', bias=128., bias_limit=1024., raw=(.5, .7, .4),
                calibrated=(.51, .72, .42), means=(.5, .7, .4), quality='valid',
                sample_age=.02, receipt_age=.015, cycle=3, historical=False,
                window=2/3, count=13, delta=.3, difference=6/11, lr_voltage_offset_v=0.,
                compensated_means=(.5, .7, .4), compensated_delta=.3, pending_lr_voltage_offset_v=None,
                compensated_difference=6/11, threshold=.1, judged='left')
    data.update(changes)
    return data


class Stream(io.StringIO):
    def __init__(self, tty=False):
        super().__init__()
        self.tty = tty

    def isatty(self):
        return self.tty


class DisplayTests(unittest.TestCase):
    def test_aligned_columns_precision_and_previous_cycle_values(self):
        plain = format_status(snapshot())
        self.assertEqual(set(map(len, plain.splitlines())), {78})
        self.assertRegex(plain, r'Calibrated / fallback\s*\|\s*0.51000\s*\|\s*0.72000\s*\|\s*0.42000')
        self.assertRegex(plain, r'Previous cycle mean\s*\|\s*0.50000\s*\|\s*0.70000\s*\|\s*0.40000')
        self.assertIn('(+128.0/1024.0)', plain)
        self.assertNotIn('\033[', plain)

    def test_compensation_is_visible_without_replacing_raw_differences(self):
        plain = format_status(snapshot(lr_voltage_offset_v=-.165, compensated_difference=-.05454545,
                                        compensated_means=(.5, .535, .565), compensated_delta=-.03,
                                        pending_lr_voltage_offset_v=-.1,
                                        direction='straight', judged='straight'))
        self.assertEqual(set(map(len, plain.splitlines())), {78})
        self.assertRegex(plain, r'L-R \[V\]\s*:\s*\+0.30000')
        self.assertRegex(plain, r'Raw diff \[%\]\s*:\s*\+54.55')
        self.assertRegex(plain, r'LR offset \[V\]\s*:\s*-0.16500')
        self.assertRegex(plain, r'Pending LR \[V\]\s*:\s*-0.10000')
        self.assertRegex(plain, r'Compensated mean\s*\|\s*0.50000\s*\|\s*0.53500\s*\|\s*0.56500')
        self.assertRegex(plain, r'Comp. L-R \[V\]\s*:\s*-0.03000')
        self.assertRegex(plain, r'Diff / T \[%\]\s*:\s*-5.45 / 10.00')

    def test_long_quality_and_reason_wrap_inside_table(self):
        plain = format_status(snapshot(reason='fault_' * 40,
                                       quality='calibration_fallback|gain_unknown|' * 8))
        self.assertEqual(set(map(len, plain.splitlines())), {78})
        self.assertIn('calibration_fallback', plain)

    def test_unavailable_values_and_historical_direction_are_explicit(self):
        plain = format_status(snapshot(raw=None, calibrated=None, means=None, quality='no_data',
                                       direction='warmup', delta=None, difference=None,
                                       compensated_difference=None, compensated_means=None, compensated_delta=None))
        self.assertIn('no completed tracking cycle', plain)
        self.assertRegex(plain, r'Raw diff \[%\]\s*:\s*--')
        self.assertRegex(plain, r'Diff / T \[%\]\s*:\s*-- / 10.00')
        self.assertRegex(plain, r'Previous cycle mean\s*\|\s*--\s*\|\s*--\s*\|\s*--')
        held = format_status(snapshot(direction='stopped', historical=True, bias=0.))
        self.assertIn('HISTORICAL', held)
        self.assertRegex(held, r'Direction\s*:\s*STOPPED')
        self.assertRegex(held, r'Last decision\s*:\s*LEFT')

    def test_color_changes_no_visible_text_or_alignment(self):
        for direction in ('left', 'right', 'straight', 'warmup', 'holding', 'stopped'):
            plain = format_status(snapshot(direction=direction,
                                           status='gain_warmup' if direction == 'holding' else 'active'))
            stream = Stream()
            write_snapshot(plain, 'always', stream, {})
            colored = stream.getvalue()
            self.assertIn('\033[', colored)
            self.assertEqual(re.sub(r'\033\[[0-9;]*m', '', colored), '\n' + plain + '\n')
            if direction == 'holding':
                self.assertIn('\033[93mHOLDING\033[0m', colored)
                self.assertIn('\033[93mgain_warmup\033[0m', colored)
                recovery = Stream()
                write_snapshot(format_status(snapshot(direction='holding', status='data_warmup')),
                               'always', recovery, {})
                self.assertIn('\033[93mdata_warmup\033[0m', recovery.getvalue())
        self.assertIn('\033[91mSTOPPED\033[0m', colored)

    def test_color_auto_tty_no_color_and_dumb_terminal(self):
        for tty, env, colored in [(True, {}, True), (False, {}, False),
                                   (True, {'TERM': 'dumb'}, False), (True, {'NO_COLOR': ''}, False)]:
            with self.subTest(tty=tty, env=env):
                stream = Stream(tty)
                write_snapshot(format_status(snapshot()), 'auto', stream, env)
                self.assertEqual('\033[' in stream.getvalue(), colored)
        stream = Stream(True)
        write_snapshot(format_status(snapshot()), 'never', stream, {})
        self.assertNotIn('\033[', stream.getvalue())
        with self.assertRaises(ValueError):
            write_snapshot('text', 'invalid', stream, {})

    def test_display_node_subscribes_and_prints_without_hardware(self):
        fake_ros = NS(get_param=lambda name, default: default, Subscriber=Mock(),
                      on_shutdown=Mock(), loginfo=Mock())
        path = Path(__file__).resolve().parents[1] / 'scripts/status_console.py'
        spec = importlib.util.spec_from_file_location('tested_status_console', path)
        module = importlib.util.module_from_spec(spec)
        with patch.dict('sys.modules', {'rospy': fake_ros, 'std_msgs.msg': NS(String=object)}):
            spec.loader.exec_module(module)
            node = module.StatusConsole()
        plain = format_status(snapshot())
        stream = Stream()
        with patch('sys.stdout', stream):
            node.callback(NS(data=plain))
        self.assertEqual(stream.getvalue(), '\n' + plain + '\n')
        self.assertEqual(fake_ros.Subscriber.call_args.args[0], '/robotic_fish/sonic/status_snapshot')


if __name__ == '__main__':
    unittest.main()
