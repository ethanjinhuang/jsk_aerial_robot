"""Deterministic trajectory, joystick, watchdog and ROS adapter tests; no hardware."""
import importlib.util
import json
import math
from pathlib import Path
import sys
import threading
from types import SimpleNamespace as NS
import unittest
from unittest.mock import Mock, patch

ROOT = Path(__file__).resolve().parents[1]
sys.path.insert(0, str(ROOT / 'src'))
sys.path.insert(0, str(ROOT.parent / 'robotic_fish_sonic' / 'src'))
from robotic_fish.sonic_motion import SonicController, SonicMotion


def joy(steering=0., pressed=(), decrease=1., increase=1., button_count=8):
    buttons = [int(i in pressed) for i in range(button_count)]
    return [0., 0., steering, decrease, increase], buttons


class MotionTests(unittest.TestCase):
    def test_quarter_periods_and_manual_bias(self):
        motion = SonicMotion()
        for t, target in [(0., 2048.), (.125, 2560.), (.25, 2048.), (.375, 1536.), (.5, 2048.)]:
            self.assertAlmostEqual(motion.sample(t, 0., 512., 2.)[0], target)
        for axis in [-1., -.5, 0., .5, 1.]:
            self.assertEqual(motion.sample(0., -axis * 1024., 0., 2.)[0], 2048. - axis * 1024.)

    def test_bias_reduces_envelope_symmetrically(self):
        motion = SonicMotion()
        for bias in [-900., 900.]:
            self.assertEqual(motion.sample(0., bias, 512., 1.)[1], 124.)
            for tick in range(101):
                target, amplitude = motion.sample(tick / 100., bias, 512., 1.)
                self.assertTrue(1024 <= target <= 3072)
                self.assertAlmostEqual(amplitude, 124.)
        self.assertEqual(motion.sample(.25, 5000., 512., 1.), (3072., 0.))
        self.assertEqual(motion.sample(.25, -5000., 512., 1.), (1024., 0.))

    def test_invalid_geometry_and_waveform(self):
        for kwargs in [{'center': float('nan')}, {'servo_range': 0}, {'servo_range': -1},
                       {'center': 32767, 'servo_range': 1}, {'center': -32768, 'servo_range': 1}]:
            with self.subTest(kwargs=kwargs), self.assertRaises(ValueError):
                SonicMotion(**kwargs)
        for args in [(-1., 0., 1., 1.), (0., float('nan'), 1., 1.),
                     (0., 0., -1., 1.), (0., 0., 1., 0.), (0., 0., 1., float('inf')),
                     (1e308, 0., 1., 1e308)]:
            with self.subTest(args=args), self.assertRaises(ValueError):
                SonicMotion().sample(*args)


class ControllerTests(unittest.TestCase):
    def feed(self, controller, now, **kwargs):
        return controller.handle_joy(*joy(**kwargs), now)

    def test_no_commands_until_first_input(self):
        c = SonicController()
        self.assertIsNone(c.tick(0.))
        self.assertIsNone(c.tick(20.))
        self.assertEqual(self.feed(c, 20.).mode, 'manual')
        self.assertEqual(c.tick(20.02).target, 2048.)

    def test_manual_position_follows_joystick_directly(self):
        c = SonicController()
        for now, axis, target in [(0., -1., 3072.), (.001, 1., 1024.),
                                  (.002, -.25, 2304.), (.003, .05, 2048.)]:
            state = self.feed(c, now, steering=axis)
            self.assertEqual(state.target, target)
            self.assertEqual(state.applied_bias, state.requested_bias)
            self.assertEqual(c.tick(now + .0001).target, target)

    def test_mode_switches_use_current_joystick_position(self):
        c = SonicController(joy_timeout_s=10.)
        self.assertEqual(self.feed(c, 0., steering=-.5).target, 2560.)
        state = self.feed(c, .1, steering=1., pressed=(3,))
        self.assertEqual((state.mode, state.applied_bias, state.target), ('auto', -1024., 1024.))
        self.assertEqual(c.tick(.2).applied_bias, -1024.)
        state = self.feed(c, .21, steering=1., pressed=(0,))
        self.assertEqual((state.mode, state.target), ('manual', 1024.))
        self.feed(c, .22, pressed=(1,))
        self.feed(c, .23)
        self.feed(c, .24, pressed=(0,))
        self.assertEqual(self.feed(c, .25, steering=-1.).target, 3072.)

    def test_auto_center_follows_joystick_without_resetting_phase(self):
        c = SonicController(joy_timeout_s=10.)
        state = self.feed(c, 0., steering=-.5, pressed=(3,))
        self.assertEqual((state.applied_bias, state.target), (512., 2560.))
        for now, axis, target in [(.125, .25, 2304.), (.25, 1., 1024.),
                                  (.251, -1., 3072.), (.375, 0., 1536.)]:
            state = self.feed(c, now, steering=axis)
            self.assertEqual(state.applied_bias, state.requested_bias)
            self.assertAlmostEqual(state.target, target)

    def test_dead_zone_preserves_previous_mapping(self):
        c = SonicController()
        self.assertEqual(self.feed(c, 0., steering=.05).requested_bias, 0.)
        self.assertAlmostEqual(self.feed(c, .1, steering=.1).requested_bias, -102.4)

    def test_acoustic_bias_only_controls_tracking_and_never_rearms_stop(self):
        c = SonicController(tracking_enabled=True)
        self.assertEqual(self.feed(c, 0., steering=-1.).target, 3072.)
        self.assertEqual(self.feed(c, .1, steering=-1., pressed=(2,)).target, 2048.)
        c.set_tracking_bias(256.)
        self.assertEqual(self.feed(c, .2, steering=1.).applied_bias, 256.)
        self.assertEqual(c.auto_epoch, .1)
        for value in (float('nan'), 1025.):
            with self.assertRaises(ValueError): c.set_tracking_bias(value)
        self.feed(c, .3, pressed=(1,))
        with self.assertRaises(ValueError): c.set_tracking_bias(256.)
        self.feed(c, .4)
        self.assertEqual(self.feed(c, .45, pressed=(0,),steering=0.).mode,'manual')
        self.assertEqual(self.feed(c,.46,steering=1.).target,1024.)

    def test_circle_selects_tracking_without_toggling_or_restarting_phase(self):
        c = SonicController(tracking_enabled=True)
        self.assertEqual(self.feed(c, 0., pressed=(2,)).mode, 'tracking')
        c.set_tracking_bias(128.)
        self.feed(c, .05)
        state = self.feed(c, .1, pressed=(2,))
        self.assertEqual((state.mode, state.applied_bias, c.auto_epoch), ('tracking', 128., 0.))
        self.assertEqual(self.feed(c, .15, pressed=(2,)).mode, 'tracking')
        state = self.feed(c, .2, pressed=(3,), steering=1.)
        self.assertEqual((state.mode, state.applied_bias), ('auto', -1024.))
        with self.assertRaises(ValueError): c.set_tracking_bias(128.)
        self.assertEqual(self.feed(c, .25, pressed=(0,)).mode, 'manual')
        state = self.feed(c, .3, pressed=(2,))
        self.assertEqual((state.mode, state.applied_bias, c.auto_epoch), ('tracking', 0., .3))

    def test_cross_stops_tracking_and_circle_recovery_requires_release(self):
        c = SonicController(tracking_enabled=True)
        self.feed(c, 0., pressed=(2,))
        state = self.feed(c, .1, pressed=(0, 1, 2, 3))
        self.assertEqual((state.mode, state.reason, state.target), ('stopped', 'stop_button', 2048.))
        self.assertEqual(self.feed(c, .2, pressed=(2,)).mode, 'stopped')
        self.feed(c, .3)
        state = self.feed(c, .4, pressed=(2,))
        self.assertEqual((state.mode, state.applied_bias), ('tracking', 0.))
        state = c.tick(.901)
        self.assertEqual((state.mode, state.reason), ('stopped', 'joy_timeout'))

    def test_circle_requires_enabled_tracking_and_conflicting_keys_stop(self):
        c = SonicController()
        self.assertEqual(self.feed(c, 0., pressed=(2,)).reason, 'tracking_unavailable')
        for pressed in ((0, 2), (2, 3)):
            c = SonicController(tracking_enabled=True)
            self.assertEqual(self.feed(c, 0., pressed=pressed).reason, 'mode_conflict')

    def test_auto_phase_edges_and_zero_amplitude_restart(self):
        c = SonicController(amplitude=0., joy_timeout_s=10.)
        self.assertEqual(self.feed(c, 0., pressed=(3,)).target, 2048.)
        self.assertEqual(c.amplitude, 512.)
        self.assertAlmostEqual(self.feed(c, .125, pressed=(3,)).target, 2560.)
        self.feed(c, .2)
        self.assertEqual(self.feed(c, .25, pressed=(3,)).target, 2048.)

    def test_auto_bias_envelope_at_position_boundary(self):
        c = SonicController(joy_timeout_s=10.)
        self.feed(c, 0., pressed=(3,), steering=-1.)
        state = c.tick(.2)
        self.assertEqual(state.applied_bias, 1024.)
        self.assertEqual(state.effective_amplitude, 0.)
        self.assertEqual(state.target, 3072.)

    def test_amplitude_direction_and_bounds(self):
        c = SonicController(joy_timeout_s=10.)
        self.feed(c, 0.)
        state = self.feed(c, .25, pressed=(6,), increase=-1.)
        self.assertEqual(state.requested_amplitude, 528.)
        state = self.feed(c, .5, pressed=(7,), decrease=-1.)
        self.assertEqual(state.requested_amplitude, 512.)
        c = SonicController(amplitude=1023., joy_timeout_s=10.)
        self.feed(c, 0.)
        self.assertEqual(self.feed(c, .25, pressed=(6,), increase=-1.).requested_amplitude, 1024.)
        c = SonicController(amplitude=1., joy_timeout_s=10.)
        self.feed(c, 0.)
        self.assertEqual(self.feed(c, .25, pressed=(7,), decrease=-1.).requested_amplitude, 0.)

    def test_stop_overrides_short_or_invalid_message(self):
        c = SonicController()
        self.feed(c, 0., pressed=(3,))
        state = c.handle_joy([], [0, 1], .1)
        self.assertEqual((state.mode, state.reason, state.target), ('stopped', 'stop_button', 2048.))
        self.assertEqual(self.feed(c, .2, pressed=(3,)).mode, 'stopped')

    def test_stop_is_latched_and_requires_neutral_release_then_new_press(self):
        c = SonicController()
        self.feed(c, 0., pressed=(3,))
        self.feed(c, .1, pressed=(1, 3))
        self.assertEqual(self.feed(c, .2, pressed=(3,)).mode, 'stopped')
        self.assertEqual(self.feed(c, .25, steering=.5).mode, 'stopped')
        self.assertEqual(self.feed(c, .3, steering=.5, pressed=(0,)).mode, 'stopped')
        self.assertEqual(self.feed(c, .35).mode, 'stopped')
        self.assertEqual(self.feed(c, .4, pressed=(0,)).mode, 'manual')
        self.assertEqual(c.tick(.41).target, 2048.)

    def test_auto_recovery_clears_old_bias(self):
        c = SonicController()
        self.feed(c, 0., steering=-1., pressed=(3,))
        self.assertGreater(c.tick(.1).applied_bias, 0.)
        self.feed(c, .11, pressed=(1,))
        self.feed(c, .2)
        state = self.feed(c, .3, pressed=(3,))
        self.assertEqual((state.mode, state.requested_bias, state.applied_bias, state.target),
                         ('auto', 0., 0., 2048.))

    def test_timeout_and_late_joy_cannot_restart_held_auto(self):
        c = SonicController()
        self.feed(c, 0., pressed=(3,))
        state = c.tick(.501)
        self.assertEqual((state.mode, state.reason, state.target), ('stopped', 'joy_timeout', 2048.))
        self.assertEqual(self.feed(c, .6, pressed=(3,)).mode, 'stopped')
        self.feed(c, .7)
        self.assertEqual(self.feed(c, .8, pressed=(3,)).mode, 'auto')
        c = SonicController()
        self.feed(c, 0., pressed=(3,))
        self.assertEqual(self.feed(c, 1., pressed=(3,)).mode, 'stopped')

    def test_manual_mode_also_times_out(self):
        c = SonicController()
        self.feed(c, 0., steering=-1.)
        self.assertGreater(c.tick(.1).target, 2048.)
        state = c.tick(.6)
        self.assertEqual((state.mode, state.reason, state.target), ('stopped', 'joy_timeout', 2048.))

    def test_phase_overflow_returns_a_stopped_command(self):
        c = SonicController(frequency=1e307, joy_timeout_s=100.)
        self.feed(c, 0., pressed=(3,))
        state = c.tick(10.)
        self.assertEqual((state.mode, state.reason, state.target), ('stopped', 'motion_error', 2048.))

    def test_invalid_input_stops_and_requires_recovery(self):
        bad = [([], []), (None, None), joy(steering=float('nan')),
               joy(increase=float('inf')), joy(steering=1.01)]
        axes, buttons = joy(); buttons[6] = 2; bad.append((axes, buttons))
        for axes, buttons in bad:
            with self.subTest(axes=axes, buttons=buttons):
                c = SonicController()
                self.feed(c, 0., pressed=(3,))
                state = c.handle_joy(axes, buttons, .1)
                self.assertEqual((state.mode, state.reason), ('stopped', 'invalid_joy'))
                self.assertEqual(self.feed(c, .2, pressed=(3,)).mode, 'stopped')
                self.feed(c, .3)
                self.assertEqual(self.feed(c, .4, pressed=(0,)).mode, 'manual')

    def test_invalid_messages_do_not_rearm_recovery(self):
        c = SonicController()
        self.feed(c, 0., pressed=(1,))
        c.handle_joy([], [], .1)
        self.assertEqual(self.feed(c, .2, pressed=(3,)).mode, 'stopped')

    def test_amplitude_button_cannot_arm_recovery(self):
        c = SonicController()
        self.feed(c, 0., pressed=(1,))
        self.feed(c, .1, pressed=(6,))
        self.assertEqual(self.feed(c, .2, pressed=(3,)).mode, 'stopped')

    def test_simultaneous_mode_requests_stop(self):
        c = SonicController()
        self.assertEqual(self.feed(c, 0., pressed=(0, 3)).reason, 'mode_conflict')

    def test_clock_faults_stop_and_clear_bias(self):
        for time_value, reason in [(float('nan'), 'invalid_clock'), (-1., 'invalid_clock'), (.5, 'clock_rewind')]:
            with self.subTest(time=time_value):
                c = SonicController()
                self.feed(c, 1., pressed=(3,), steering=-1.)
                state = c.tick(time_value)
                self.assertEqual((state.mode, state.reason, state.target), ('stopped', reason, 2048.))

    def test_mapping_and_configuration_validation(self):
        c = SonicController(steering_axis=0, amplitude_decrease_axis=1, amplitude_increase_axis=2,
                            stop_button=2, auto_button=4, manual_button=5, tracking_button=3, amplitude_buttons=[6])
        buttons = [0] * 7; buttons[4] = 1
        self.assertEqual(c.handle_joy([-.5, 1., 1.], buttons, 0.).requested_bias, 512.)
        bad = [{'frequency': 0}, {'frequency': float('nan')}, {'default_amplitude': 1025},
               {'amplitude': -1}, {'amplitude_rate': -1},
               {'joy_timeout_s': 0}, {'joy_dead_zone': 1}, {'joy_dead_zone': -.1},
               {'stop_button': -1}, {'manual_button': True}, {'manual_button': 3},
               {'tracking_button': 1}, {'tracking_enabled': 'true'},
               {'steering_axis': 3}, {'amplitude_buttons': []}, {'amplitude_buttons': [1]},
               {'amplitude_buttons': [6, 6]}]
        for kwargs in bad:
            with self.subTest(kwargs=kwargs), self.assertRaises(ValueError):
                SonicController(**kwargs)


class RosAdapterTests(unittest.TestCase):
    def make_node(self, params=None):
        published = []
        fake_ros = NS(get_param=lambda k, default=None: (params or {}).get(k, default),
                      Publisher=Mock(return_value=NS(publish=published.append)),
                      Subscriber=Mock(return_value=NS(unregister=lambda: None)), on_shutdown=Mock(),
                      loginfo=Mock(), logwarn=Mock(), logdebug=Mock(), ROSException=RuntimeError,
                      Service=Mock(side_effect=lambda *args, **kwargs: NS(shutdown=Mock())))
        fake_ros.published = published
        modules = {'rospy': fake_ros, 'sensor_msgs.msg': NS(Joy=object),
                   'std_msgs.msg': NS(String=lambda data:NS(data=data)),
                   'spinal.msg': NS(ServoControlCmd=lambda: NS(index=[], cmd=[])),
                   'robotic_fish_sonic.msg': NS(AdcSampleArray=object,
                       SonicMotionState=lambda: NS(header=NS()),
                       RuntimeConfig=lambda: NS(), AcousticSteeringEvent=lambda: NS(header=NS()),
                       AcousticSteeringState=lambda: NS(header=NS())),
                   'robotic_fish_sonic.srv': NS(SetAcousticSteering=object,
                       SetAcousticSteeringResponse=lambda success,message:NS(success=success,message=message))}
        spec = importlib.util.spec_from_file_location('tested_sonic_teleop', ROOT / 'scripts/sonic_teleop.py')
        module = importlib.util.module_from_spec(spec)
        with patch.dict(sys.modules, modules):
            spec.loader.exec_module(module)
        module._mock_modules = modules
        clock = [0.]
        fake_ros.Time = NS(now=lambda:NS(to_sec=lambda value=clock[0]:value),from_sec=lambda t:t)
        module.time = NS(monotonic=lambda: clock[0])
        module.threading = NS(RLock=threading.RLock, Event=threading.Event,
                              Thread=Mock(return_value=NS(start=lambda: None, join=lambda **k: None,
                                                          is_alive=lambda: False)),
                              current_thread=lambda: None)
        return module, fake_ros, clock, published

    def test_real_adapter_publish_stop_and_shutdown(self):
        module, ros, clock, published = self.make_node()
        node = module.Teleop()
        node._controlLoop(); self.assertFalse(any(hasattr(m, 'cmd') for m in published))
        node._joyCallback(NS(axes=joy(pressed=(3,))[0], buttons=joy(pressed=(3,))[1]))
        clock[0] = .125; node._controlLoop()
        command = [m for m in published if hasattr(m, 'cmd')][-1]
        self.assertEqual((command.index, command.cmd), ([0], [2560]))
        clock[0] = .6; node._controlLoop()
        self.assertEqual([m for m in published if hasattr(m, 'cmd')][-1].cmd, [2048])
        self.assertEqual(node.controller.reason, 'joy_timeout')
        node._shutdown()
        self.assertEqual([m for m in published if hasattr(m, 'cmd')][-1].cmd, [2048])
        self.assertEqual(node.controller.reason, 'shutdown')
        self.assertEqual(ros.Publisher.call_count, 5)

    def test_invalid_config_fails_before_publisher(self):
        for params in [{'~update_rate': 0}, {'~update_rate': 4}, {'~update_rate': True},
                       {'~servo_cmd': []}, {'~servo_cmd': [float('nan')]}, {'~servo_cmd': [5000]},
                       *[{'~status_button': value} for value in (-1, True, 1.5, 0, 1, 2, 3, 6)],
                       {'~acoustic_steering/max_age_s': 0},
                       {'~acoustic_steering/max_age_s': True},
                       {'~acoustic_steering/future_tolerance_s': False},
                       {'~status_local_echo': 'true'},
                       {'~acoustic_steering/future_tolerance_s': -1}]:
            with self.subTest(params=params):
                module, ros, _, _ = self.make_node(params)
                with self.assertRaises(ValueError):
                    module.Teleop()
                ros.Publisher.assert_not_called()

    def test_idle_shutdown_does_not_activate_the_servo(self):
        module, _, _, published = self.make_node()
        node = module.Teleop()
        node._shutdown()
        self.assertFalse(any(hasattr(m, 'cmd') for m in published))

    def test_shutdown_centers_before_optional_recording_failure(self):
        module, ros, _, published = self.make_node({'~acoustic_steering/enabled': True})
        with patch.dict(sys.modules, module._mock_modules):
            node = module.Teleop()
        self.send_status_joy(node, (3,), steering=-.25)
        self.assertTrue(node._setAcousticSteering(NS(threshold=.15, bias=128., lr_voltage_offset_v=.08)).success)
        node.event_pub = NS(publish=Mock(side_effect=ros.ROSException('event transport closed')))
        node._shutdown()
        self.assertEqual(node.controller.reason, 'shutdown')
        self.assertEqual([m for m in published if hasattr(m, 'cmd')][-1].cmd, [2048])

    def adc(self, t):
        return NS(samples=[NS(channel_id=i,timestamp=NS(to_sec=lambda:t),volt_cali=value,
                              volt_raw=value*.5,
                              status_cali=True,cali_id='test',adc_code=100,
                              status_dac_feedback=True,dac_volt=.9)
                           for i,value in enumerate((1.,2.,1.))])

    def test_acoustic_decision_and_atomic_runtime_parameters(self):
        module,ros,clock,published=self.make_node({'~acoustic_steering/enabled':True})
        with patch.dict(sys.modules,module._mock_modules):
            node=module.Teleop()
        node._adcCallback(self.adc(0.))
        node._joyCallback(NS(axes=joy(pressed=(2,))[0],buttons=joy(pressed=(2,))[1]))
        node._controlLoop()
        self.assertEqual(node.controller.applied_bias,0.)
        for i in range(1,11):
            clock[0]=i*.05
            node._adcCallback(self.adc(clock[0]))
            node._joyCallback(NS(axes=joy(steering=-1.)[0],buttons=joy()[1]))
            node._controlLoop()
            if i<10:self.assertEqual(node.controller.applied_bias,0.)
        self.assertEqual((node.controller.mode,node.controller.applied_bias),('tracking',-256.))
        self.assertTrue(node._setAcousticSteering(NS(threshold=.7,bias=128.,lr_voltage_offset_v=0.)).success)
        self.assertFalse(node._setAcousticSteering(NS(threshold=.2,bias=1025.,lr_voltage_offset_v=0.)).success)
        self.assertEqual(node.acoustic.settings.bias,256.)
        for i in range(11,21):
            clock[0]=i*.05
            node._adcCallback(self.adc(clock[0]))
            node._joyCallback(NS(axes=joy()[0],buttons=joy()[1]))
            node._controlLoop()
        self.assertEqual((node.controller.applied_bias,node.acoustic.settings.bias),(0.,128.))
        clock[0]=1.21
        node._joyCallback(NS(axes=joy()[0],buttons=joy()[1]))
        node._controlLoop()
        self.assertEqual((node.controller.mode,node.controller.reason),('stopped','acoustic_adc_timeout'))
        self.assertTrue(any(getattr(msg,'cmd',[])==[2048] for msg in published))
        node._shutdown()
        node.settings_service.shutdown.assert_called_once()

    def test_mean_percentage_boundary_matches_motion_share_service_and_recording(self):
        for left, right, threshold, direction, bias in (
                (1.075, .925, .15, 'left', -256.),
                (.925, 1.075, .15, 'right', 256.),
                (1.074, .926, .15, 'straight', 0.),
                (1., 1., 0., 'straight', 0.)):
            with self.subTest(left=left, right=right, threshold=threshold):
                module, ros, clock, published = self.make_node({
                    '~acoustic_steering/enabled': True, '~acoustic_steering/threshold': threshold})
                with patch.dict(sys.modules, module._mock_modules):
                    node = module.Teleop()
                for i in range(11):
                    clock[0] = i * .05
                    frame = self.adc(clock[0])
                    frame.samples[1].volt_cali = left
                    frame.samples[2].volt_cali = right
                    node._adcCallback(frame)
                    self.send_status_joy(node, (2,) if i == 0 else ())
                self.assertEqual(node.controller.applied_bias, bias)
                state = [m for m in published if hasattr(m, 'decision_updated')][-1]
                self.assertEqual((state.direction, state.decision_direction), (direction, direction))
                self.assertEqual(state.difference_reference, 'channel_mean')
                self.assertEqual(state.left_bias_sign, -1)
                self.assertEqual(state.command_bias, bias)
                motion = [m for m in published if hasattr(m, 'target_position')][-1]
                self.assertAlmostEqual(motion.target_position, 2048. + bias)
                response = node._setAcousticSteering(NS(threshold=threshold, bias=256., lr_voltage_offset_v=0.))
                self.assertEqual(response.direction, direction)
                self.assertEqual(response.difference_reference, 'channel_mean')
                self.send_status_joy(node, (8,))
                self.assertRegex(self.status_reports(ros)[-1], r'Direction\s*:\s*' + direction.upper())

    def test_three_field_service_changes_judgment_and_snapshot_at_next_cycle(self):
        module, ros, clock, published = self.make_node({'~acoustic_steering/enabled': True,
                                                       '~acoustic_steering/lr_voltage_offset_v': .1})
        with patch.dict(sys.modules, module._mock_modules):
            node = module.Teleop()
        node._adcCallback(self.adc(0.))
        self.send_status_joy(node, (2,))
        for i in range(1, 11):
            clock[0] = i * .05
            node._adcCallback(self.adc(clock[0]))
            self.send_status_joy(node)
        previous = node.acoustic.last_decision
        self.assertEqual(ros.Service.call_count, 1)
        self.assertTrue(node._setAcousticSteering(NS(threshold=.1, bias=128., lr_voltage_offset_v=-1.5)).success)
        pending = node.acoustic.pending
        self.assertFalse(node._setAcousticSteering(NS(threshold=.8, bias=64., lr_voltage_offset_v=float('nan'))).success)
        self.assertIs(node.acoustic.pending, pending)
        self.send_status_joy(node, (8,))
        self.assertIn('+0.10000', self.status_reports(ros)[-1])
        self.assertIs(node.acoustic.last_decision, previous)
        for i in range(11, 21):
            clock[0] = i * .05
            node._adcCallback(self.adc(clock[0]))
            self.send_status_joy(node)
        self.send_status_joy(node, (8,))
        report = self.status_reports(ros)[-1]
        self.assertRegex(report, r'LR offset \[V\]\s*:\s*-1.50000')
        self.assertRegex(report, r'Raw cycle mean\s*\|\s*0.50000\s*\|\s*1.00000\s*\|\s*0.50000')
        self.assertRegex(report, r'Calibrated cycle mean\s*\|\s*1.00000\s*\|\s*2.00000\s*\|\s*1.00000')
        self.assertRegex(report, r'Diff before \[%\]\s*:\s*\(2.00000-1.00000\)/AVG = \+66.67%')
        self.assertNotIn('Compensated mean', report)
        self.assertRegex(report, r'Diff after/T \[%\]\s*:\s*\(2.00000-1.00000\+\(-1.50000\)\)/AVG = -33.33% / 10.00%')
        self.assertRegex(report, r'Direction\s*:\s*RIGHT')
        self.assertEqual(node.controller.applied_bias, 128.)
        state = [m for m in published if hasattr(m, 'normalized_difference')][-1]
        self.assertAlmostEqual(state.normalized_difference, 2 / 3)
        self.assertAlmostEqual(state.compensated_difference, -1 / 3)
        self.assertEqual(state.lr_voltage_offset_v, -1.5)
        self.assertEqual((state.compensated_left_mean, state.compensated_right_mean), (1.25, 1.75))
        self.assertEqual(state.compensated_voltage_difference, -.5)
        self.assertEqual(node.controller.auto_epoch, 0.)
        self.send_status_joy(node, (1, 8))
        self.assertEqual(node.controller.mode, 'stopped')
        node._shutdown()
        self.assertFalse(node._setAcousticSteering(NS(threshold=.1, bias=128., lr_voltage_offset_v=0.)).success)

    def test_acoustic_missing_data_cannot_start_tracking_but_manual_works(self):
        module,_,clock,_=self.make_node({'~acoustic_steering/enabled':True})
        with patch.dict(sys.modules,module._mock_modules):
            node=module.Teleop()
        node._joyCallback(NS(axes=joy(steering=-1.)[0],buttons=joy()[1]))
        node._controlLoop()
        self.assertEqual(node.controller.tick(0.).target,3072.)
        node._joyCallback(NS(axes=joy(pressed=(2,))[0],buttons=joy(pressed=(2,))[1]))
        node._controlLoop()
        self.assertEqual(node.controller.reason,'acoustic_no_data')
        clock[0]=.1
        node._joyCallback(NS(axes=joy(pressed=(2,))[0],buttons=joy(pressed=(2,))[1]))
        node._controlLoop()
        self.assertEqual(node.controller.mode,'stopped')

    def test_gain_warmup_keeps_motion_and_share_marks_held_decision(self):
        module, ros, clock, published = self.make_node({'~acoustic_steering/enabled': True})
        with patch.dict(sys.modules, module._mock_modules):
            node = module.Teleop()
        node._adcCallback(self.adc(0.))
        self.send_status_joy(node, (2,))
        for i in range(1, 11):
            clock[0] = i * .05
            node._adcCallback(self.adc(clock[0]))
            self.send_status_joy(node)
        epoch = node.controller.auto_epoch
        for i in range(11, 31):
            clock[0] = i * .05
            frame = self.adc(clock[0])
            for s in frame.samples:
                s.dac_volt = 1.
                s.volt_cali = (1., 1., 3.)[s.channel_id]
            if i == 11:
                frame.samples[0].dac_volt = .9  # Change part way through a group.
            node._adcCallback(frame)
            self.send_status_joy(node, (8,) if i == 12 else ())
            self.assertEqual((node.controller.mode, node.controller.auto_epoch), ('tracking', epoch))
            if i < 30:
                self.assertEqual(node._acoustic_status, 'gain_warmup')
                self.assertEqual(node.controller.applied_bias, -256.)
                expected, _ = node.controller.motion.sample(clock[0] - epoch, -256., 512., 2.)
                self.assertEqual([m for m in published if hasattr(m, 'cmd')][-1].cmd, [int(expected)])
        report = self.status_reports(ros)[-1]
        self.assertIn('tracking / gain_warmup', report)
        self.assertRegex(report, r'Direction\s*:\s*HOLDING')
        self.assertIn('HISTORICAL', report)
        self.assertIn('(-256.0/1024.0)', report)
        self.assertEqual((node._acoustic_status, node.controller.applied_bias), ('active', 256.))
        self.assertEqual(node.acoustic.last_decision.cycle, 3)

    def test_cross_and_joy_timeout_still_stop_during_quality_or_gain_wait(self):
        for quality, reason in ((q, r) for q in ('gain', 'calibration', 'unknown')
                                for r in ('stop_button', 'joy_timeout')):
            with self.subTest(reason=reason, quality=quality):
                module, _, clock, published = self.make_node({'~acoustic_steering/enabled': True})
                with patch.dict(sys.modules, module._mock_modules):
                    node = module.Teleop()
                node._adcCallback(self.adc(0.))
                self.send_status_joy(node, (2,))
                for i in range(1, 13):
                    clock[0] = i * .05
                    frame = self.adc(clock[0])
                    for s in frame.samples:
                        s.dac_volt = 1. + i * .01
                        s.status_cali = quality != 'calibration'
                        s.status_dac_feedback = quality != 'unknown'
                    node._adcCallback(frame)
                    if reason == 'stop_button':
                        self.send_status_joy(node, (1,) if i == 3 else ())
                    else:
                        node._controlLoop()
                self.assertEqual((node.controller.mode, node.controller.reason), ('stopped', reason))
                self.assertEqual([m for m in published if hasattr(m, 'cmd')][-1].cmd, [2048])

    def test_unusable_quality_keeps_tracking_bias_and_resumes_automatically(self):
        for quality, expected in [('calibration', 'calibration_fallback'), ('unknown', 'gain_unknown')]:
            with self.subTest(quality=quality):
                module, ros, clock, _ = self.make_node({'~acoustic_steering/enabled': True})
                with patch.dict(sys.modules, module._mock_modules):
                    node = module.Teleop()
                node._adcCallback(self.adc(0.))
                self.send_status_joy(node, (2,))
                for i in range(1, 51):
                    clock[0] = i * .05
                    frame = self.adc(clock[0])
                    if 11 <= i <= 30:
                        for s in frame.samples:
                            if quality == 'calibration':
                                s.status_cali, s.cali_id = False, ''
                            else:
                                s.status_dac_feedback, s.dac_volt = False, float('nan')
                    elif i >= 31:
                        for s in frame.samples:
                            s.volt_cali = (1., 1., 3.)[s.channel_id]
                    node._adcCallback(frame)
                    self.send_status_joy(node, (8,) if i in (12, 32) else ())
                    self.assertEqual((node.controller.mode, node.controller.auto_epoch), ('tracking', 0.))
                    if 11 <= i <= 30:
                        self.assertEqual(node._acoustic_status, expected)
                    if 11 <= i < 50:
                        self.assertEqual(node.controller.applied_bias, -256.)
                for report in self.status_reports(ros):
                    self.assertRegex(report, r'Direction\s*:\s*HOLDING')
                    self.assertIn('HISTORICAL', report)
                self.assertIn('data_warmup', self.status_reports(ros)[-1])
                self.assertEqual((node._acoustic_status, node.controller.applied_bias), ('active', 256.))

    def test_tracking_can_start_with_quality_hold_and_zero_bias(self):
        for quality in ('calibration', 'unknown'):
            module, ros, _, _ = self.make_node({'~acoustic_steering/enabled': True})
            with patch.dict(sys.modules, module._mock_modules):
                node = module.Teleop()
            frame = self.adc(0.)
            for s in frame.samples:
                s.status_cali = quality != 'calibration'
                s.status_dac_feedback = quality != 'unknown'
            node._adcCallback(frame)
            self.send_status_joy(node, (2, 8))
            self.assertEqual((node.controller.mode, node.controller.applied_bias), ('tracking', 0.))
            self.assertRegex(self.status_reports(ros)[-1], r'Direction\s*:\s*HOLDING')
            self.assertIn('no completed tracking cycle', self.status_reports(ros)[-1])

    def test_triangle_ordinary_auto_ignores_acoustic_faults_and_uses_joystick(self):
        module,_,clock,_=self.make_node({'~acoustic_steering/enabled':True})
        with patch.dict(sys.modules,module._mock_modules):
            node=module.Teleop()
        node._joyCallback(NS(axes=joy(pressed=(3,),steering=-.25)[0],buttons=joy(pressed=(3,))[1]))
        node._controlLoop()
        self.assertEqual((node.controller.mode,node.controller.applied_bias),('auto',256.))
        clock[0]=.1
        node._adcCallback(NS(samples=[]))
        node._controlLoop()
        self.assertEqual((node.controller.mode,node.controller.reason),('auto','auto_button'))

    def send_status_joy(self, node, pressed=(), **kwargs):
        axes, buttons = joy(pressed=pressed, button_count=14, **kwargs)
        node._joyCallback(NS(axes=axes, buttons=buttons))
        node._controlLoop()

    def status_reports(self, ros):
        return [message.data for message in ros.published if hasattr(message, 'data')]

    def test_share_reports_actual_cycle_means_once_per_press_without_changing_gait(self):
        module, ros, clock, published = self.make_node({'~acoustic_steering/enabled': True})
        with patch.dict(sys.modules, module._mock_modules):
            node = module.Teleop()
        module.write_snapshot = Mock()
        node._adcCallback(self.adc(0.))
        self.send_status_joy(node, (2,))
        for i in range(1, 11):
            clock[0] = i * .05
            node._adcCallback(self.adc(clock[0]))
            self.send_status_joy(node)
        epoch, cycle = node.controller.auto_epoch, node.acoustic.cycle
        clock[0] = .52
        newest = self.adc(.52)
        newest.samples[0].volt_cali = 4.
        node._adcCallback(newest)
        self.send_status_joy(node, (8,))
        report = self.status_reports(ros)[-1]
        self.assertRegex(report, r'Calibrated cycle mean\s*\|\s*1.00000\s*\|\s*2.00000\s*\|\s*1.00000')
        self.assertRegex(report, r'Diff after/T \[%\]\s*:\s*\(2.00000-1.00000\+\(\+0.00000\)\)/AVG = \+66.67% / 10.00%')
        self.assertRegex(report, r'Direction\s*:\s*LEFT')
        self.assertIn('(-256.0/1024.0)', report)
        self.assertNotIn('\033[', report)
        module.write_snapshot.assert_not_called()
        self.send_status_joy(node, (8,))
        self.assertEqual(len(self.status_reports(ros)), 1)
        self.assertEqual((node.controller.auto_epoch, node.acoustic.cycle), (epoch, cycle))
        self.send_status_joy(node)
        self.send_status_joy(node, (8,))
        self.assertEqual(len(self.status_reports(ros)), 2)
        self.send_status_joy(node)
        self.send_status_joy(node, (1, 8))
        report = self.status_reports(ros)[-1]
        self.assertIn('HISTORICAL', report)
        self.assertRegex(report, r'Direction\s*:\s*STOPPED')
        self.assertRegex(report, r'Reason\s*:\s*stop_button')
        self.assertIn('(+0.0/1024.0)', report)
        self.assertEqual([message for message in published if hasattr(message, 'cmd')][-1].cmd, [2048])

    def test_share_works_with_tracking_disabled_and_marks_missing_or_stale_data(self):
        module, ros, clock, _ = self.make_node()
        node = module.Teleop()
        self.send_status_joy(node, (8,))
        self.assertIn('no_data', self.status_reports(ros)[-1])
        self.assertRegex(self.status_reports(ros)[-1], r'Direction\s*:\s*DISABLED')
        self.assertIn('(+0.0/1024.0)', self.status_reports(ros)[-1])
        clock[0] = .1
        frame = self.adc(.1)
        frame.samples.reverse()
        node._adcCallback(frame)
        self.send_status_joy(node)
        self.send_status_joy(node, (8,))
        report = self.status_reports(ros)[-1]
        self.assertRegex(report, r'Raw instant\s*\|\s*0.50000\s*\|\s*1.00000\s*\|\s*0.50000')
        self.assertRegex(report, r'ADC quality\s*:\s*valid')
        self.assertIn('no completed tracking cycle', report)
        clock[0] = .4
        self.send_status_joy(node)
        self.send_status_joy(node, (8,))
        self.assertRegex(self.status_reports(ros)[-1], r'ADC quality\s*:\s*stale_sample')
        self.assertEqual(node.controller.mode, 'manual')
        node._shutdown()
        self.assertIsNone(node.acoustic)

    def test_share_marks_warmup_and_invalid_calibration_without_fabricating_means(self):
        module, ros, clock, _ = self.make_node({'~acoustic_steering/enabled': True})
        with patch.dict(sys.modules, module._mock_modules):
            node = module.Teleop()
        node._adcCallback(self.adc(0.))
        self.send_status_joy(node, (2, 8))
        self.assertRegex(self.status_reports(ros)[-1], r'Direction\s*:\s*WARMUP')
        self.assertIn('no completed tracking cycle', self.status_reports(ros)[-1])
        clock[0] = .05
        bad = self.adc(.05)
        bad.samples[1].status_cali = False
        node._adcCallback(bad)
        self.send_status_joy(node)
        self.send_status_joy(node, (8,))
        report = self.status_reports(ros)[-1]
        self.assertRegex(report, r'ADC quality\s*:\s*calibration_fallback')
        self.assertRegex(report, r'Direction\s*:\s*HOLDING')

    def test_share_can_be_remapped_and_handles_incomplete_adc_groups(self):
        module, ros, _, _ = self.make_node({'~status_button': 10, '~status_local_echo': True})
        node = module.Teleop()
        module.write_snapshot = Mock()
        self.send_status_joy(node, (8,))
        self.assertEqual(self.status_reports(ros), [])
        node._adcCallback(NS(samples=[]))
        self.send_status_joy(node, (10,))
        self.assertIn('expected one channel each', self.status_reports(ros)[-1])
        module.write_snapshot.assert_called_once_with(self.status_reports(ros)[-1])
        self.assertEqual(node.controller.mode, 'manual')

    def test_continuous_idle_state_and_latched_effective_configuration_do_not_send_commands(self):
        module, ros, clock, published = self.make_node({'~servo_center': 2000., '~auto_amplitude': 400.})
        node = module.Teleop()
        for t in (0., .02, .04):
            clock[0] = t
            node._controlLoop()
        states = [m for m in published if hasattr(m, 'command_published')]
        self.assertEqual(len(states), 3)
        self.assertEqual(states[-1].motion_mode, 'idle')
        self.assertFalse(states[-1].command_published)
        self.assertTrue(math.isnan(states[-1].target_position))
        self.assertTrue(math.isnan(states[-1].phase_rad))
        self.assertTrue(math.isnan(states[-1].valid_joy_age_s))
        self.assertEqual(states[-1].adc_quality, 'no_data')
        self.assertAlmostEqual(states[-1].control_interval_s, .02)
        self.assertFalse(any(hasattr(m, 'cmd') for m in published))
        configs = [m for m in published if hasattr(m, 'config_json')]
        self.assertEqual(len(configs), 1)
        values = json.loads(configs[0].config_json)
        self.assertEqual(values['schema_version'], 4)
        self.assertEqual(values['settings']['acoustic_steering']['difference_reference'], 'channel_mean')
        self.assertEqual(values['settings']['acoustic_steering']['voltage_offset_reference'], 'left_minus_right')
        self.assertTrue(values['settings']['acoustic_steering']['threshold_inclusive'])
        self.assertEqual(values['settings']['servo_center'], 2000.)
        self.assertEqual(values['settings']['auto_amplitude'], 400.)
        self.assertEqual(values['settings']['initial_servo_cmd'], 2000.)
        self.assertEqual(values['settings']['acoustic_steering']['max_age_s'], .2)
        self.assertIsNone(values['pending_request'])
        self.assertTrue(any(call.args[0] == '/robotic_fish/sonic/config' and call.kwargs.get('latch')
                            for call in ros.Publisher.call_args_list))

    def test_motion_telemetry_matches_servo_output_phase_and_amplitude_clipping(self):
        module, _, clock, published = self.make_node()
        node = module.Teleop()
        self.send_status_joy(node, (3,), steering=-.25)
        clock[0] = .125
        node._controlLoop()
        state = [m for m in published if hasattr(m, 'command_published')][-1]
        self.assertEqual((state.requested_bias, state.applied_bias), (256., 256.))
        self.assertEqual((state.requested_amplitude, state.effective_amplitude), (512., 512.))
        self.assertAlmostEqual(state.phase_rad, math.pi / 2)
        self.assertEqual(state.target_position, 2816.)
        self.assertEqual(state.target_command, [m for m in published if hasattr(m, 'cmd')][-1].cmd[0])
        self.send_status_joy(node, steering=-1.)
        state = [m for m in published if hasattr(m, 'command_published')][-1]
        self.assertEqual((state.effective_amplitude, state.target_command), (0., 3072))
        self.send_status_joy(node, (1,))
        state = [m for m in published if hasattr(m, 'command_published')][-1]
        self.assertEqual((state.reason, state.target_command), ('stop_button', 2048))
        self.assertTrue(math.isnan(state.phase_rad))

    def test_adc_telemetry_distinguishes_source_clock_receipt_age_and_gap(self):
        module, _, clock, published = self.make_node()
        node = module.Teleop()
        clock[0] = 1.
        node._adcCallback(self.adc(1.067))
        node._controlLoop()
        state = [m for m in published if hasattr(m, 'command_published')][-1]
        self.assertEqual(state.adc_quality, 'future_sample')
        self.assertAlmostEqual(state.adc_source_age_at_receipt_s, -.067)
        self.assertAlmostEqual(state.adc_source_age_s, -.067)
        self.assertEqual(state.adc_receipt_age_s, 0.)
        clock[0] = 1.1
        node._controlLoop()
        state = [m for m in published if hasattr(m, 'command_published')][-1]
        self.assertEqual(state.adc_quality, 'valid')
        self.assertAlmostEqual(state.adc_source_age_s, .033)
        self.assertAlmostEqual(state.adc_source_age_at_receipt_s, -.067)
        clock[0] = 1.3
        node._adcCallback(self.adc(1.3))
        node._controlLoop()
        state = [m for m in published if hasattr(m, 'command_published')][-1]
        self.assertAlmostEqual(state.adc_receipt_interval_s, .3)
        clock[0] = 1.6
        node._controlLoop()
        state = [m for m in published if hasattr(m, 'command_published')][-1]
        self.assertEqual(state.adc_quality, 'stale_sample')
        node._adcCallback(NS(samples=[]))
        node._controlLoop()
        state = [m for m in published if hasattr(m, 'command_published')][-1]
        self.assertFalse(state.adc_structurally_valid)
        self.assertIn('invalid_input', state.adc_quality)
        self.assertTrue(math.isnan(state.adc_source_age_s))

    def test_invalid_joy_records_arrival_without_refreshing_watchdog_age(self):
        module, _, clock, published = self.make_node()
        node = module.Teleop()
        self.send_status_joy(node, (3,))
        clock[0] = .1
        node._joyCallback(NS(axes=[], buttons=[]))
        node._controlLoop()
        state = [m for m in published if hasattr(m, 'command_published')][-1]
        self.assertEqual(state.reason, 'invalid_joy')
        self.assertTrue(state.input_error)
        self.assertEqual(state.joy_receipt_age_s, 0.)
        self.assertAlmostEqual(state.valid_joy_age_s, .1)

    def test_structured_cycle_means_and_historical_decision_during_quality_hold(self):
        module, _, clock, published = self.make_node({'~acoustic_steering/enabled': True})
        with patch.dict(sys.modules, module._mock_modules):
            node = module.Teleop()
        node._adcCallback(self.adc(0.))
        self.send_status_joy(node, (2,))
        warmup = [m for m in published if hasattr(m, 'decision_updated')][-1]
        self.assertFalse(warmup.has_decision)
        self.assertEqual(warmup.direction, 'warmup')
        self.assertTrue(math.isnan(warmup.head_mean))
        for i in range(1, 11):
            clock[0] = i * .05
            node._adcCallback(self.adc(clock[0]))
            self.send_status_joy(node)
        active = [m for m in published if hasattr(m, 'decision_updated')][-1]
        self.assertTrue(active.has_decision and active.decision_current)
        self.assertEqual((active.head_mean, active.voltage_difference), (1., 1.))
        self.assertEqual((active.direction, active.decision_direction), ('left', 'left'))
        clock[0] = .55
        frame = self.adc(.55)
        frame.samples[0].status_cali = False
        node._adcCallback(frame)
        self.send_status_joy(node)
        held = [m for m in published if hasattr(m, 'decision_updated')][-1]
        self.assertTrue(held.has_decision)
        self.assertFalse(held.decision_current)
        self.assertEqual((held.direction, held.decision_direction), ('holding', 'left'))
        self.assertEqual(held.head_mean, 1.)
        motion = [m for m in published if hasattr(m, 'command_published')][-1]
        self.assertEqual(motion.adc_quality_flags, 4)
        self.assertEqual(motion.acoustic_status, 'calibration_fallback')

    def test_parameter_events_link_requests_supersession_rejection_and_actual_application(self):
        module, _, clock, published = self.make_node({'~acoustic_steering/enabled': True})
        with patch.dict(sys.modules, module._mock_modules):
            node = module.Teleop()
        node._adcCallback(self.adc(0.))
        self.send_status_joy(node, (2,))
        self.assertTrue(node._setAcousticSteering(NS(threshold=.2, bias=128., lr_voltage_offset_v=.08)).success)
        self.assertFalse(node._setAcousticSteering(NS(threshold=float('nan'), bias=64., lr_voltage_offset_v=0.)).success)
        self.assertTrue(node._setAcousticSteering(NS(threshold=1.4, bias=64., lr_voltage_offset_v=.1)).success)
        events = [m for m in published if hasattr(m, 'event')]
        self.assertEqual([(m.request_id, m.event) for m in events],
                         [(1, 'accepted'), (2, 'rejected'), (1, 'superseded'), (3, 'accepted')])
        self.assertTrue(math.isnan(events[1].threshold))
        self.assertTrue(all(m.difference_reference == 'channel_mean' for m in events))
        pending = json.loads([m for m in published if hasattr(m, 'config_json')][-1].config_json)
        self.assertEqual(pending['pending_request']['request_id'], 3)
        self.assertEqual(pending['settings']['acoustic_steering']['threshold'], .1)
        for i in range(1, 11):
            clock[0] = i * .05
            node._adcCallback(self.adc(clock[0]))
            self.send_status_joy(node)
        applied = [m for m in published if hasattr(m, 'event')][-1]
        self.assertEqual((applied.request_id, applied.event, applied.cycle_index), (3, 'applied', 1))
        config = json.loads([m for m in published if hasattr(m, 'config_json')][-1].config_json)
        self.assertIsNone(config['pending_request'])
        self.assertEqual(config['applied_request_id'], 3)
        self.assertEqual(config['settings']['acoustic_steering']['threshold'], 1.4)
        self.assertEqual(config['settings']['initial_servo_cmd'], 2048.)
        self.assertEqual(node.controller.applied_bias, 0.)
        # Repeating the active values is still a distinct request and application.
        self.assertTrue(node._setAcousticSteering(NS(threshold=1.4, bias=64., lr_voltage_offset_v=.1)).success)
        for i in range(11, 21):
            clock[0] = i * .05
            node._adcCallback(self.adc(clock[0]))
            self.send_status_joy(node)
        repeated = [m for m in published if hasattr(m, 'event')][-1]
        self.assertEqual((repeated.request_id, repeated.event, repeated.cycle_index), (4, 'applied', 2))
        self.assertTrue(node._setAcousticSteering(NS(threshold=.1, bias=32., lr_voltage_offset_v=0.)).success)
        node._shutdown()
        cancelled = [m for m in published if hasattr(m, 'event')][-1]
        self.assertEqual((cancelled.request_id, cancelled.event), (5, 'cancelled'))
        self.assertFalse([m for m in published if hasattr(m, 'decision_updated')][-1].parameters_pending)
        self.assertEqual([m for m in published if hasattr(m, 'command_published')][-1].reason, 'shutdown')

    def test_service_feedback_reports_active_pending_state_and_rejection_without_motion(self):
        module, _, clock, published = self.make_node({'~acoustic_steering/enabled': True})
        with patch.dict(sys.modules, module._mock_modules):
            node = module.Teleop()
        # A configuration call before Joy remains read-only with respect to motion.
        accepted = node._setAcousticSteering(NS(threshold=.15, bias=128., lr_voltage_offset_v=.08))
        self.assertEqual((accepted.success, accepted.request_id, accepted.motion_mode), (True, 1, 'manual'))
        self.assertEqual((accepted.adc_quality, accepted.direction, accepted.command_bias),
                         ('no_data', 'not_tracking', 0.))
        self.assertEqual((accepted.active_threshold, accepted.active_bias, accepted.active_lr_voltage_offset_v),
                         (.1, 256., 0.))
        self.assertTrue(accepted.parameters_pending)
        self.assertEqual((accepted.pending_request_id, accepted.pending_threshold,
                          accepted.pending_bias, accepted.pending_lr_voltage_offset_v), (1, .15, 128., .08))
        self.assertFalse(any(hasattr(m, 'cmd') for m in published))
        node._adcCallback(self.adc(0.))
        self.send_status_joy(node, (2,))
        for i in range(1, 11):
            clock[0] = i * .05
            node._adcCallback(self.adc(clock[0]))
            self.send_status_joy(node)
        decision, epoch = node.acoustic.last_decision, node.controller.auto_epoch
        commands = len([m for m in published if hasattr(m, 'cmd')])
        rejected = node._setAcousticSteering(NS(threshold=-1., bias=64., lr_voltage_offset_v=0.))
        self.assertFalse(rejected.success)
        self.assertEqual((rejected.motion_mode, rejected.status, rejected.direction, rejected.adc_quality),
                         ('tracking', 'active', 'left', 'valid'))
        self.assertEqual((rejected.command_bias, rejected.active_threshold,
                          rejected.active_bias, rejected.active_lr_voltage_offset_v), (-128., .15, 128., .08))
        self.assertFalse(rejected.parameters_pending)
        self.assertEqual(rejected.pending_request_id, 0)
        self.assertTrue(all(math.isnan(value) for value in
                            (rejected.pending_threshold, rejected.pending_bias, rejected.pending_lr_voltage_offset_v)))
        accepted = node._setAcousticSteering(NS(threshold=.3, bias=64., lr_voltage_offset_v=.1))
        rejected = node._setAcousticSteering(NS(threshold=.4, bias=1025., lr_voltage_offset_v=.2))
        self.assertEqual((rejected.request_id, rejected.pending_request_id), (4, accepted.request_id))
        self.assertEqual((rejected.pending_threshold, rejected.pending_bias, rejected.pending_lr_voltage_offset_v),
                         (.3, 64., .1))
        self.assertEqual(len([m for m in published if hasattr(m, 'cmd')]), commands)
        self.assertIs(node.acoustic.last_decision, decision)
        self.assertEqual(node.controller.auto_epoch, epoch)

    def test_voltage_compensation_hold_preserves_gait_pending_request_and_feedback(self):
        module, ros, clock, published = self.make_node({'~acoustic_steering/enabled': True})
        with patch.dict(sys.modules, module._mock_modules):
            node = module.Teleop()
        node._adcCallback(self.adc(0.))
        self.send_status_joy(node, (2,))
        for i in range(1, 11):
            clock[0] = i * .05
            node._adcCallback(self.adc(clock[0]))
            self.send_status_joy(node)
        previous = node.acoustic.last_decision
        request = node._setAcousticSteering(NS(threshold=.2, bias=64., lr_voltage_offset_v=-6.))
        self.assertTrue(request.success and request.parameters_pending)
        for i in range(11, 31):
            clock[0] = i * .05
            node._adcCallback(self.adc(clock[0]))
            self.send_status_joy(node, (8,) if i == 21 else ())
            self.assertEqual((node.controller.mode, node.controller.auto_epoch, node.controller.applied_bias),
                             ('tracking', 0., -256.))
            if i >= 20:
                self.assertEqual(node._acoustic_status, 'voltage_compensation_out_of_range')
                self.assertIs(node.acoustic.last_decision, previous)
        report = self.status_reports(ros)[-1]
        self.assertIn('voltage_compensation_out_of_range', report)
        self.assertIn('HISTORICAL', report)
        self.assertRegex(report, r'Pending LR \[V\]\s*:\s*-6.00000')
        self.assertFalse(any(m.event == 'applied' for m in published if hasattr(m, 'event')))
        replacement = node._setAcousticSteering(NS(threshold=.1, bias=128., lr_voltage_offset_v=-1.5))
        self.assertEqual((replacement.direction, replacement.adc_quality), ('holding', 'valid'))
        for i in range(31, 41):
            clock[0] = i * .05
            node._adcCallback(self.adc(clock[0]))
            self.send_status_joy(node)
        self.assertEqual((node._acoustic_status, node.controller.applied_bias), ('active', 128.))
        self.assertEqual(node.acoustic.last_decision.cycle, 4)
        events = [m for m in published if hasattr(m, 'event')]
        self.assertEqual([(m.request_id, m.event) for m in events],
                         [(1, 'accepted'), (1, 'superseded'), (2, 'accepted'), (2, 'applied')])

    def test_cross_and_joy_timeout_still_stop_during_voltage_compensation_hold(self):
        for reason in ('stop_button', 'joy_timeout'):
            module, _, clock, published = self.make_node({'~acoustic_steering/enabled': True})
            with patch.dict(sys.modules, module._mock_modules):
                node = module.Teleop()
            node._adcCallback(self.adc(0.))
            self.send_status_joy(node, (2,))
            node._setAcousticSteering(NS(threshold=.1, bias=128., lr_voltage_offset_v=-6.))
            for i in range(1, 32):
                clock[0] = i * .05
                node._adcCallback(self.adc(clock[0]))
                if reason == 'stop_button':
                    self.send_status_joy(node, (1,) if i == 21 else ())
                elif i <= 20:
                    self.send_status_joy(node)
                else:
                    node._controlLoop()
            self.assertEqual((node.controller.mode, node.controller.reason), ('stopped', reason))
            self.assertEqual([m for m in published if hasattr(m, 'cmd')][-1].cmd, [2048])


if __name__ == '__main__':
    unittest.main()
