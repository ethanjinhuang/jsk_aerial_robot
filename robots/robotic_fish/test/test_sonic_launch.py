"""Resolve real ROS launch includes and arguments without starting any nodes."""
import os
from pathlib import Path
import importlib.util
import shlex
import tempfile
import unittest
from unittest.mock import patch

import yaml
import roslaunch.config
import roslaunch.substitution_args
import roslaunch.xmlloader
# Keep catkin's generated package paths before pure-motion fixtures add src/.
import robotic_fish_sonic.msg


ROOT = Path(__file__).resolve().parents[1]
REPOSITORY = ROOT.parents[1]


class SonicLaunchTests(unittest.TestCase):
    def load(self, filename, *arguments, package='robotic_fish'):
        paths = [str(ROOT), str(ROOT.parent/'robotic_fish_sonic'),
                 str(REPOSITORY/'aerial_robot_nerve/spinal'),
                 os.environ.get('ROS_PACKAGE_PATH', '')]
        config = roslaunch.config.ROSLaunchConfig()
        with patch.dict(os.environ, {'ROS_PACKAGE_PATH': ':'.join(paths)}), \
                patch.object(roslaunch.substitution_args, '_rospack', None):
            roslaunch.xmlloader.XmlLoader().load(str(ROOT.parent/package/'launch'/filename), config,
                                                argv=list(arguments), verbose=False)
        return config

    def gain_values(self, config):
        # Exercise the actual node's canonical/legacy lookup, without constructing
        # a ROS node, creating services or opening hardware.
        import rospy
        spec = importlib.util.spec_from_file_location('launch_gain_lookup',
            ROOT.parent/'robotic_fish_sonic/scripts/agc_node.py')
        module = importlib.util.module_from_spec(spec)
        spec.loader.exec_module(module)
        node = module.GainControlNode
        key = lambda name: '/gain_control/' + name.lstrip('~')
        with patch.object(rospy, 'has_param', side_effect=lambda name: key(name) in config.params), \
                patch.object(rospy, 'get_param', side_effect=lambda name, default=None:
                    config.params[key(name)].value if key(name) in config.params else default):
            return dict(mode=node._configured_mode(), **{
                name: node._param(name, None) for name in
                ('manual_voltage','target_min_v','target_max_v','step_v','adc_safety_limit_v')})

    def test_pc_launch_has_only_joystick(self):
        config = self.load('joy.launch')
        self.assertEqual([(node.package,node.type,node.name) for node in config.nodes],
                         [('joy','joy_node','joy_node')])
        self.assertEqual(config.params['/joy_node/dev'].value, '/dev/input/js0')
        self.assertEqual(config.params['/joy_node/autorepeat_rate'].value, 20.)
        self.assertEqual(config.params['/joy_node/coalesce_interval'].value, .025)

    def test_recording_includes_continuous_state_configuration_events_and_diagnostics(self):
        config = self.load('record.launch')
        self.assertEqual(len(config.nodes), 1)
        topics = config.nodes[0].args.split()
        for topic in ('/robotic_fish/adc/samples', '/robotic_fish/acoustic/steering_state',
                      '/robotic_fish/sonic/motion_state', '/robotic_fish/sonic/config',
                      '/robotic_fish/acoustic/steering_events', '/robotic_fish/sonic/status_snapshot',
                      '/robotic_fish/gain_control/config', '/servo/target_states', '/diagnostics', '/rosout'):
            self.assertIn(topic, topics)

    def test_recording_diagnostics_can_be_disabled(self):
        config = self.load('record.launch', 'record_diagnostics:=false')
        self.assertNotIn('/diagnostics', config.nodes[0].args.split())

    def test_pc_joystick_options(self):
        config = self.load('joy.launch', 'joy_dev:=/dev/input/js1', 'joy_autorepeat_rate:=30')
        self.assertEqual(config.params['/joy_node/dev'].value, '/dev/input/js1')
        self.assertEqual(config.params['/joy_node/autorepeat_rate'].value, 30.)

    def test_original_entry_keeps_joystick_and_motion(self):
        config = self.load('sonic_teleop.launch')
        self.assertCountEqual([node.type for node in config.nodes], ['joy_node','sonic_teleop.py'])
        self.assertFalse(config.params['/sonic_teleop_node/acoustic_steering/enabled'].value)
        self.assertEqual(config.params['/sonic_teleop_node/acoustic_steering/left_bias_sign'].value, -1)
        self.assertEqual(config.params['/sonic_teleop_node/acoustic_steering/future_tolerance_s'].value, .05)

    def test_motion_only_entry_has_no_io_or_joystick(self):
        config = self.load('sonic_teleop.launch', 'enable_joy:=false',
                           'acoustic_steering:=true', 'acoustic_future_tolerance_s:=0.1')
        self.assertEqual([node.type for node in config.nodes], ['sonic_teleop.py'])
        self.assertTrue(config.params['/sonic_teleop_node/acoustic_steering/enabled'].value)
        self.assertEqual(config.params['/sonic_teleop_node/acoustic_steering/future_tolerance_s'].value, .1)
        self.assertEqual(config.params['/sonic_teleop_node/acoustic_steering/left_bias_sign'].value, -1)

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
        self.assertEqual(config.params['/sonic_teleop_node/acoustic_steering/left_bias_sign'].value, -1)

    def test_vim4_entry_forwards_control_gain_and_display_options(self):
        values = {'auto_frequency': 2., 'acoustic_threshold': .15, 'acoustic_bias': 128.,
                  'acoustic_lr_voltage_offset_v': -.01, 'acoustic_max_age_s': .7,
                  'acoustic_future_tolerance_s': .025, 'left_bias_sign': 1,
                  'joy_timeout_s': .6, 'joy_timeout_action': 'stop',
                  'update_rate': 60., 'manual_gain_voltage': 1.7}
        config = self.load('sonic_vim4.launch', 'status_color:=never',
                           *[key+':='+str(value) for key,value in values.items()])
        for argument, parameter in [('auto_frequency','auto_frequency'),
                ('acoustic_threshold','acoustic_steering/threshold'),
                ('acoustic_bias','acoustic_steering/bias'),
                ('acoustic_lr_voltage_offset_v','acoustic_steering/lr_voltage_offset_v'),
                ('acoustic_max_age_s','acoustic_steering/max_age_s'),
                ('acoustic_future_tolerance_s','acoustic_steering/future_tolerance_s'),
                ('left_bias_sign','acoustic_steering/left_bias_sign'),
                ('joy_timeout_s','joy_timeout_s'), ('joy_timeout_action','joy_timeout_action'),
                ('update_rate','update_rate')]:
            self.assertEqual(config.params['/sonic_teleop_node/'+parameter].value, values[argument])
        self.assertEqual(self.gain_values(config)['manual_voltage'], 1.7)
        self.assertEqual(config.params['/sonic_status_console/color'].value, 'never')

    def test_vim4_optional_features_can_be_disabled_without_starting_joystick(self):
        config = self.load('sonic_vim4.launch', 'enable_status_console:=false',
                           'acoustic_steering:=false', 'gain_mode:=off')
        self.assertEqual(len(config.nodes), 5)
        self.assertNotIn('joy_node', [node.type for node in config.nodes])
        self.assertNotIn('status_console.py', [node.type for node in config.nodes])
        self.assertFalse(config.params['/sonic_teleop_node/acoustic_steering/enabled'].value)
        self.assertEqual(config.params['/gain_control/gain_control/mode'].value, 'off')

    def test_default_motion_config_preserves_user_range_and_scene_defaults(self):
        config = self.load('sonic_vim4.launch')
        expected = dict(servo_center=2048., servo_range=512., auto_frequency=1.5,
                        auto_default_amplitude=512., auto_amplitude_rate=64.,
                        joy_dead_zone=.1, joy_timeout_s=.5, joy_timeout_action='continue', update_rate=50.)
        for key, value in expected.items():
            self.assertEqual(config.params['/sonic_teleop_node/'+key].value, value)
        self.assertEqual(self.gain_values(config)['mode'], 'manual')
        self.assertEqual(self.gain_values(self.load('sensor_io.launch', package='robotic_fish_sonic'))['mode'], 'off')
        self.assertFalse(self.load('sonic_teleop.launch').params['/sonic_teleop_node/acoustic_steering/enabled'].value)

    def test_motion_yaml_is_loaded_in_node_private_namespace(self):
        settings = yaml.safe_load((ROOT/'config/motion.yaml').read_text())
        settings.update(servo_center=2000., servo_range=300., auto_default_amplitude=100.,
                        auto_frequency=2.4, auto_amplitude_rate=10., joy_timeout_s=.7, joy_timeout_action='stop',
                        steering_axis=1, status_local_echo=True)
        settings['acoustic_steering'].update(enabled=True, threshold=.25, bias=120.)
        with tempfile.TemporaryDirectory() as directory:
            path = Path(directory)/'motion.yaml'
            path.write_text(yaml.safe_dump(settings))
            config = self.load('sonic_teleop.launch', 'motion_config:='+str(path))
            for name in ('servo_center','servo_range','auto_default_amplitude','auto_frequency',
                         'auto_amplitude_rate','joy_timeout_s','joy_timeout_action','steering_axis','status_local_echo'):
                self.assertEqual(config.params['/sonic_teleop_node/'+name].value, settings[name])
                self.assertNotIn('/'+name, config.params)
            self.assertTrue(config.params['/sonic_teleop_node/acoustic_steering/enabled'].value)
            overridden = self.load('sonic_vim4.launch', 'motion_config:='+str(path),
                'auto_frequency:=1.8', 'acoustic_steering:=false', 'acoustic_bias:=0',
                'acoustic_lr_voltage_offset_v:=0', 'left_bias_sign:=1')
            self.assertEqual(overridden.params['/sonic_teleop_node/auto_frequency'].value, 1.8)
            self.assertFalse(overridden.params['/sonic_teleop_node/acoustic_steering/enabled'].value)
            self.assertEqual(overridden.params['/sonic_teleop_node/acoustic_steering/bias'].value, 0.)
            self.assertEqual(overridden.params['/sonic_teleop_node/acoustic_steering/lr_voltage_offset_v'].value, 0.)
            self.assertEqual(overridden.params['/sonic_teleop_node/acoustic_steering/left_bias_sign'].value, 1)

    def test_custom_legacy_io_yaml_is_not_overridden_by_launch_defaults(self):
        settings = yaml.safe_load((ROOT.parent/'robotic_fish_sonic/config/io.yaml').read_text())
        settings['gain_control'].update(mode='closed_loop',manual_voltage=.4,
            target_min_v=1.8,target_max_v=2.2,step_v=.02,adc_safety_limit_v=3.5)
        with tempfile.TemporaryDirectory() as directory:
            path = Path(directory)/'io.yaml'
            path.write_text(yaml.safe_dump(settings))
            for filename in ('sensor_io.launch','sonic.launch'):
                config = self.load(filename, 'config:='+str(path), package='robotic_fish_sonic')
                self.assertEqual(self.gain_values(config), dict(mode='closed_loop',manual_voltage=.4,
                    target_min_v=1.8,target_max_v=2.2,step_v=.02,adc_safety_limit_v=3.5))
            experiment = self.load('sonic_vim4.launch', 'io_config:='+str(path))
            self.assertEqual(self.gain_values(experiment)['mode'], 'manual')
            self.assertEqual(self.gain_values(experiment)['manual_voltage'], .4)

    def test_explicit_io_overrides_win_canonical_and_legacy_yaml(self):
        settings = yaml.safe_load((ROOT.parent/'robotic_fish_sonic/config/io.yaml').read_text())
        settings['gain_control'].update(mode='closed_loop',manual_target_voltage_v=.4,
            target_min_voltage_v=1.8,target_max_voltage_v=2.2,voltage_step_v=.02,
            adc_safety_trigger_voltage_v=3.5)
        with tempfile.TemporaryDirectory() as directory:
            path = Path(directory)/'canonical.yaml'
            path.write_text(yaml.safe_dump(settings))
            for package,filename,arg in [('robotic_fish_sonic','sensor_io.launch','config'),
                                         ('robotic_fish_sonic','sonic.launch','config'),
                                         ('robotic_fish','sonic_vim4.launch','io_config')]:
                config = self.load(filename, arg+':='+str(path),'gain_mode:=off',
                    'manual_gain_voltage:=0','agc_target_min_v:=2.0','agc_target_max_v:=2.8',
                    'agc_step_v:=0.03','adc_safety_limit_v:=3.8',package=package)
                self.assertEqual(self.gain_values(config), dict(mode='off',manual_voltage=0.,
                    target_min_v=2.,target_max_v=2.8,step_v=.03,adc_safety_limit_v=3.8))

    def test_deprecated_agc_alias_preserves_yaml_and_explicit_false(self):
        settings = yaml.safe_load((ROOT.parent/'robotic_fish_sonic/config/io.yaml').read_text())
        settings['agc'] = dict(enabled=True)
        with tempfile.TemporaryDirectory() as directory:
            path = Path(directory)/'io.yaml'
            path.write_text(yaml.safe_dump(settings))
            inherited = self.load('sensor_io.launch','config:='+str(path),package='robotic_fish_sonic')
            self.assertEqual(self.gain_values(inherited)['mode'], 'closed_loop')
            disabled = self.load('sensor_io.launch','config:='+str(path),'enable_agc:=false',package='robotic_fish_sonic')
            self.assertEqual(self.gain_values(disabled)['mode'], 'off')
            conflicting = self.load('sensor_io.launch','config:='+str(path),'gain_mode:=manual',package='robotic_fish_sonic')
            with self.assertRaises(ValueError):
                self.gain_values(conflicting)

    def test_io_config_alias_and_canonical_argument_priority(self):
        with tempfile.TemporaryDirectory() as directory:
            settings = yaml.safe_load((ROOT.parent/'robotic_fish_sonic/config/io.yaml').read_text())
            paths=[]
            for index in (1,2):
                settings['adc']['bus']=index
                path=Path(directory)/('io'+str(index)+'.yaml')
                path.write_text(yaml.safe_dump(settings))
                paths.append(str(path))
            alias=self.load('sonic_vim4.launch','config:='+paths[0])
            self.assertEqual(alias.params['/adc/adc/bus'].value,1)
            canonical=self.load('sonic_vim4.launch','config:='+paths[0],'io_config:='+paths[1])
            self.assertEqual(canonical.params['/adc/adc/bus'].value,2)

    def test_sensor_only_experiment_keeps_protection_and_optional_recording(self):
        config = self.load('sonic_vim4.launch','enable_motion:=false','enable_bridge:=false',
                           'enable_status_console:=false','gain_mode:=off','record:=true',
                           'bag_prefix:=/tmp/fish recordings/run','record_diagnostics:=false')
        self.assertCountEqual([node.type for node in config.nodes],
                              ['adc_node.py','dac_node.py','agc_node.py','record'])
        self.assertEqual(len({node.name for node in config.nodes}),len(config.nodes))
        self.assertEqual(self.gain_values(config)['mode'],'off')
        self.assertEqual(self.gain_values(config)['adc_safety_limit_v'],4.)
        recorder=next(node for node in config.nodes if node.type=='record')
        args=shlex.split(recorder.args)
        self.assertEqual(args[args.index('-o')+1],'/tmp/fish recordings/run')
        self.assertNotIn('/diagnostics',args)
        self.assertIn('/robotic_fish/gain_control/config',args)

    def test_recording_adds_one_recorder_to_the_default_experiment(self):
        config=self.load('sonic_vim4.launch','record:=true')
        self.assertEqual(len(config.nodes),7)
        self.assertEqual(sum(node.type=='record' for node in config.nodes),1)
        self.assertEqual(len({node.name for node in config.nodes}),len(config.nodes))
        self.assertNotIn('joy_node',[node.type for node in config.nodes])

    def test_removed_io_entry_migrates_to_sensor_bridge_and_display_only(self):
        config=self.load('sonic_vim4.launch','enable_motion:=false','gain_mode:=off')
        self.assertCountEqual([node.type for node in config.nodes],
            ['serial_node','adc_node.py','dac_node.py','agc_node.py','status_console.py'])
        self.assertEqual(self.gain_values(config)['mode'],'off')
        self.assertEqual(config.params['/rosserial_server/port'].value,'/dev/robotic_fish_spinal')
        self.assertEqual(len({node.name for node in config.nodes}),5)

    def test_optional_analysis_can_consume_external_adc_without_gain_controller(self):
        config=self.load('sonic.launch','enable_adc:=false','enable_dac:=false',package='robotic_fish_sonic')
        self.assertEqual([node.type for node in config.nodes],['quadrant_node.py'])


if __name__ == '__main__':
    unittest.main()
