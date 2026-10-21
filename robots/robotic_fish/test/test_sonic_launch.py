"""Resolve real ROS launch includes and arguments without starting any nodes."""
import os
from pathlib import Path
import unittest
from unittest.mock import patch

import roslaunch.config
import roslaunch.substitution_args
import roslaunch.xmlloader


ROOT = Path(__file__).resolve().parents[1]
REPOSITORY = ROOT.parents[1]


class SonicLaunchTests(unittest.TestCase):
    def load(self, filename, *arguments):
        paths = [str(ROOT), str(ROOT.parent/'robotic_fish_sonic'),
                 str(REPOSITORY/'aerial_robot_nerve/spinal'),
                 os.environ.get('ROS_PACKAGE_PATH', '')]
        config = roslaunch.config.ROSLaunchConfig()
        with patch.dict(os.environ, {'ROS_PACKAGE_PATH': ':'.join(paths)}), \
                patch.object(roslaunch.substitution_args, '_rospack', None):
            roslaunch.xmlloader.XmlLoader().load(str(ROOT/'launch'/filename), config,
                                                argv=list(arguments), verbose=False)
        return config

    def test_pc_launch_has_only_joystick(self):
        config = self.load('joy.launch')
        self.assertEqual([(node.package,node.type,node.name) for node in config.nodes],
                         [('joy','joy_node','joy_node')])
        self.assertEqual(config.params['/joy_node/dev'].value, '/dev/input/js0')
        self.assertEqual(config.params['/joy_node/autorepeat_rate'].value, 20.)
        self.assertEqual(config.params['/joy_node/coalesce_interval'].value, .025)

    def test_pc_joystick_options(self):
        config = self.load('joy.launch', 'joy_dev:=/dev/input/js1', 'joy_autorepeat_rate:=30')
        self.assertEqual(config.params['/joy_node/dev'].value, '/dev/input/js1')
        self.assertEqual(config.params['/joy_node/autorepeat_rate'].value, 30.)

    def test_original_entry_keeps_joystick_and_motion(self):
        config = self.load('sonic_teleop.launch')
        self.assertCountEqual([node.type for node in config.nodes], ['joy_node','sonic_teleop.py'])
        self.assertFalse(config.params['/sonic_teleop_node/acoustic_steering/enabled'].value)
        self.assertEqual(config.params['/sonic_teleop_node/acoustic_steering/future_tolerance_s'].value, .05)

    def test_motion_only_entry_has_no_io_or_joystick(self):
        config = self.load('sonic_teleop.launch', 'enable_joy:=false',
                           'acoustic_steering:=true', 'acoustic_future_tolerance_s:=0.1')
        self.assertEqual([node.type for node in config.nodes], ['sonic_teleop.py'])
        self.assertTrue(config.params['/sonic_teleop_node/acoustic_steering/enabled'].value)
        self.assertEqual(config.params['/sonic_teleop_node/acoustic_steering/future_tolerance_s'].value, .1)

    def test_vim4_entry_has_one_motion_bridge_and_sensor_stack(self):
        config = self.load('sonic_vim4.launch')
        self.assertCountEqual([node.type for node in config.nodes],
                             ['serial_node','adc_node.py','dac_node.py','agc_node.py',
                              'status_console.py','sonic_teleop.py'])
        self.assertTrue(all(not node.machine_name for node in config.nodes))
        self.assertEqual(config.params['/rosserial_server/port'].value, '/dev/robotic_fish_spinal')
        self.assertEqual(config.params['/gain_control/gain_control/mode'].value, 'manual')
        self.assertEqual(config.params['/gain_control/gain_control/manual_voltage'].value, 0.)
        self.assertTrue(config.params['/sonic_teleop_node/acoustic_steering/enabled'].value)

    def test_vim4_entry_forwards_control_gain_and_display_options(self):
        values = {'auto_frequency': 2., 'acoustic_threshold': .15, 'acoustic_bias': 128.,
                  'acoustic_lr_offset': .08, 'acoustic_max_age_s': .7,
                  'acoustic_future_tolerance_s': .025, 'left_bias_sign': -1,
                  'joy_timeout_s': .6, 'update_rate': 60., 'manual_gain_voltage': 1.7}
        config = self.load('sonic_vim4.launch', 'status_color:=never',
                           *[key+':='+str(value) for key,value in values.items()])
        for argument, parameter in [('auto_frequency','auto_frequency'),
                ('acoustic_threshold','acoustic_steering/threshold'),
                ('acoustic_bias','acoustic_steering/bias'),
                ('acoustic_lr_offset','acoustic_steering/lr_offset'),
                ('acoustic_max_age_s','acoustic_steering/max_age_s'),
                ('acoustic_future_tolerance_s','acoustic_steering/future_tolerance_s'),
                ('left_bias_sign','acoustic_steering/left_bias_sign'),
                ('joy_timeout_s','joy_timeout_s'), ('update_rate','update_rate')]:
            self.assertEqual(config.params['/sonic_teleop_node/'+parameter].value, values[argument])
        self.assertEqual(config.params['/gain_control/gain_control/manual_voltage'].value, 1.7)
        self.assertEqual(config.params['/sonic_status_console/color'].value, 'never')

    def test_vim4_optional_features_can_be_disabled_without_starting_joystick(self):
        config = self.load('sonic_vim4.launch', 'enable_status_console:=false',
                           'acoustic_steering:=false', 'gain_mode:=off')
        self.assertEqual(len(config.nodes), 5)
        self.assertNotIn('joy_node', [node.type for node in config.nodes])
        self.assertNotIn('status_console.py', [node.type for node in config.nodes])
        self.assertFalse(config.params['/sonic_teleop_node/acoustic_steering/enabled'].value)
        self.assertEqual(config.params['/gain_control/gain_control/mode'].value, 'off')


if __name__ == '__main__':
    unittest.main()
