#!/usr/bin/env python3
"""Exercise embedded old/new message definitions and offline tools.

Synthetic bags only; no ROS master or hardware. Run separately from unit tests.
"""
import hashlib
import json
from pathlib import Path
import subprocess
import sys
import tempfile
import unittest

import genpy
import genpy.dynamic
import rosbag


ROOT = Path(__file__).resolve().parents[1]


class BagCompatibilityTests(unittest.TestCase):
    def run_tool(self, name, bag, output, *extra):
        result = subprocess.run(
            [sys.executable, str(ROOT / 'scripts' / name), str(bag),
             '--output', str(output), *extra], capture_output=True, text=True)
        self.assertEqual(result.returncode, 0, result.stdout + result.stderr)
        return json.loads((output / 'summary.json').read_text())

    def check_schema(self, package, field_count):
        sample_definition = (ROOT / 'msg/AdcSample.msg').read_text()
        if field_count == 12:
            sample_definition = '\n'.join(
                line for line in sample_definition.splitlines()
                if not line.startswith(('float64 volt_raw_mean_1s', 'bool mean_1s_ready')))
        definition = ('{0}/AdcSample[] samples\n\n'.format(package) + '=' * 80 +
                      '\nMSG: {0}/AdcSample\n'.format(package) + sample_definition + '\n')
        types = genpy.dynamic.generate_dynamic(package + '/AdcSampleArray', definition)
        sample_type = types[package + '/AdcSample']
        array_type = types[package + '/AdcSampleArray']
        self.assertEqual(len(sample_type.__slots__), field_count)
        with tempfile.TemporaryDirectory(prefix='fish-sonic-bag-test-') as directory:
            directory = Path(directory)
            path = directory / 'synthetic.bag'
            with rosbag.Bag(str(path), 'w') as bag:
                for plateau in range(4):
                    for tick in range(80):
                        message = array_type()
                        stamp = 100 + plateau * 4 + tick * .05
                        for channel, ratio in enumerate((1., .9, 1.2)):
                            sample = sample_type()
                            sample.serial_num = plateau * 80 + tick
                            sample.timestamp = genpy.Time.from_sec(stamp + channel * .005)
                            sample.device = 'SYNTHETIC_COMPATIBILITY_TEST'
                            sample.channel_id = channel
                            sample.volt_raw = (plateau + 1) * .1 * ratio
                            sample.volt_cali = sample.volt_raw
                            sample.adc_code = int(sample.volt_raw * 8000)
                            sample.diff_cali = 0.
                            sample.status_cali = True
                            sample.cali_id = 'SYNTHETIC_COMPATIBILITY_TEST'
                            sample.dac_volt = .9 + plateau * .1
                            sample.status_dac_feedback = True
                            message.samples.append(sample)
                        bag.write('/robotic_fish/adc/samples', message, message.samples[-1].timestamp)
            before = hashlib.sha256(path.read_bytes()).hexdigest()
            replay = self.run_tool('replay_quadrants.py', path, directory / 'replay')[0]
            self.assertEqual(replay['counts']['groups'], 320)
            self.assertEqual(replay['counts']['comparison_updates'], 320)
            self.assertGreater(replay['counts']['reliable_comparisons'], 0)
            self.assertEqual(replay['counts']['updated'], 0)
            calibration_dir = directory / 'calibration'
            audit = self.run_tool('calibrate_from_bag.py', path, calibration_dir, '--common-input')
            self.assertEqual(audit['groups'], 320)
            self.assertEqual(audit['invalid_groups'], 0)
            self.assertEqual(audit['accepted_plateaus'], 4)
            corrected = directory / 'corrected'
            self.run_tool('replay_quadrants.py', path, corrected, '--calibration',
                          str(calibration_dir / 'candidate_calibration.json'))
            rows = [json.loads(line) for line in (corrected / 'synthetic.jsonl').read_text().splitlines()]
            self.assertEqual(len(rows), 320)
            for row in rows:
                self.assertIsNone(row['probabilities'])
                comparison = row['comparison']
                self.assertAlmostEqual(comparison['left_right_normalized_difference'], 0.)
                self.assertAlmostEqual(comparison['head_sides_normalized_difference'], 0.)
            self.assertEqual(hashlib.sha256(path.read_bytes()).hexdigest(), before)

    def test_old_package_12_fields(self):
        self.check_schema('robotic_fish_io', 12)

    def test_old_package_14_fields(self):
        self.check_schema('robotic_fish_io', 14)

    def test_merged_package_14_fields(self):
        self.check_schema('robotic_fish_sonic', 14)


if __name__ == '__main__':
    unittest.main(verbosity=2)
