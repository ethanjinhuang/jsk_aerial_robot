"""Deterministic trajectory, joystick, watchdog and ROS adapter tests; no hardware."""
import importlib.util
import math
from pathlib import Path
import sys
import threading
from types import SimpleNamespace as NS
import unittest
from unittest.mock import Mock, patch

ROOT = Path(__file__).resolve().parents[1]
sys.path.insert(0, str(ROOT / 'src'))
from robotic_fish.sonic_motion import SonicController, SonicMotion


def joy(steering=0., pressed=(), decrease=1., increase=1.):
    buttons = [int(i in pressed) for i in range(8)]
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
                            stop_button=2, auto_button=4, manual_button=5, amplitude_buttons=[6])
        buttons = [0] * 7; buttons[4] = 1
        self.assertEqual(c.handle_joy([-.5, 1., 1.], buttons, 0.).requested_bias, 512.)
        bad = [{'frequency': 0}, {'frequency': float('nan')}, {'default_amplitude': 1025},
               {'amplitude': -1}, {'amplitude_rate': -1},
               {'joy_timeout_s': 0}, {'joy_dead_zone': 1}, {'joy_dead_zone': -.1},
               {'stop_button': -1}, {'manual_button': True}, {'manual_button': 3},
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
                      loginfo=Mock(), logwarn=Mock(), loginfo_throttle=Mock(), ROSException=RuntimeError)
        modules = {'rospy': fake_ros, 'sensor_msgs.msg': NS(Joy=object),
                   'spinal.msg': NS(ServoControlCmd=lambda: NS(index=[], cmd=[]))}
        spec = importlib.util.spec_from_file_location('tested_sonic_teleop', ROOT / 'scripts/sonic_teleop.py')
        module = importlib.util.module_from_spec(spec)
        with patch.dict(sys.modules, modules):
            spec.loader.exec_module(module)
        clock = [0.]
        module.time = NS(monotonic=lambda: clock[0])
        module.threading = NS(RLock=threading.RLock, Event=threading.Event,
                              Thread=Mock(return_value=NS(start=lambda: None, join=lambda **k: None,
                                                          is_alive=lambda: False)),
                              current_thread=lambda: None)
        return module, fake_ros, clock, published

    def test_real_adapter_publish_stop_and_shutdown(self):
        module, ros, clock, published = self.make_node()
        node = module.Teleop()
        node._controlLoop(); self.assertEqual(published, [])
        node._joyCallback(NS(axes=joy(pressed=(3,))[0], buttons=joy(pressed=(3,))[1]))
        clock[0] = .125; node._controlLoop()
        self.assertEqual((published[-1].index, published[-1].cmd), ([0], [2560]))
        clock[0] = .6; node._controlLoop()
        self.assertEqual(published[-1].cmd, [2048])
        self.assertEqual(node.controller.reason, 'joy_timeout')
        node._shutdown()
        self.assertEqual(published[-1].cmd, [2048])
        self.assertEqual(node.controller.reason, 'shutdown')
        self.assertEqual(ros.Publisher.call_count, 1)

    def test_invalid_config_fails_before_publisher(self):
        for params in [{'~update_rate': 0}, {'~update_rate': 4}, {'~update_rate': True},
                       {'~servo_cmd': []}, {'~servo_cmd': [float('nan')]}, {'~servo_cmd': [5000]}]:
            with self.subTest(params=params):
                module, ros, _, _ = self.make_node(params)
                with self.assertRaises(ValueError):
                    module.Teleop()
                ros.Publisher.assert_not_called()

    def test_idle_shutdown_does_not_activate_the_servo(self):
        module, _, _, published = self.make_node()
        node = module.Teleop()
        node._shutdown()
        self.assertEqual(published, [])


if __name__ == '__main__':
    unittest.main()
