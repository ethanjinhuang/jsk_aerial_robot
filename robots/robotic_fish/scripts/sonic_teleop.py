#!/usr/bin/env python3
"""ROS adapter for single-servo motion; no acoustic control in this node yet."""
import math
import threading
import time

import rospy
from sensor_msgs.msg import Joy
from spinal.msg import ServoControlCmd

from robotic_fish.sonic_motion import SonicController


class Teleop:
    def __init__(self):
        parameters = {
            'center': rospy.get_param('~servo_center', 2048.),
            'servo_range': rospy.get_param('~servo_range', 1024.),
            'frequency': rospy.get_param('~auto_frequency', 2.),
            'default_amplitude': rospy.get_param('~auto_default_amplitude', 512.),
            'amplitude': rospy.get_param('~auto_amplitude', None),
            'amplitude_rate': rospy.get_param('~auto_amplitude_rate', 64.),
            'joy_timeout_s': rospy.get_param('~joy_timeout_s', .5),
            'joy_dead_zone': rospy.get_param('~joy_dead_zone', .1),
        }
        for name, default in [('steering_axis', 2), ('amplitude_decrease_axis', 3),
                              ('amplitude_increase_axis', 4), ('stop_button', 1),
                              ('auto_button', 3), ('manual_button', 0), ('amplitude_buttons', [6, 7])]:
            parameters[name] = rospy.get_param('~' + name, default)
        self.controller = SonicController(**parameters)
        self.update_rate = rospy.get_param('~update_rate', 50.)
        if isinstance(self.update_rate, bool):
            raise ValueError('update_rate must be a positive finite number')
        self.update_rate = float(self.update_rate)
        if not math.isfinite(self.update_rate) or self.update_rate <= 2. * self.controller.frequency:
            raise ValueError('update_rate must be finite and exceed twice auto_frequency')
        # Retain the old parameter as an initial buffer; no command is sent until input.
        self.servo_cmd = rospy.get_param('~servo_cmd', [self.controller.motion.center])
        if not isinstance(self.servo_cmd, (list, tuple)) or not self.servo_cmd:
            raise ValueError('servo_cmd must contain an initial position')
        if isinstance(self.servo_cmd[0], bool):
            raise ValueError('initial servo_cmd must be a finite position')
        initial = float(self.servo_cmd[0])
        if not math.isfinite(initial) or not self.controller.motion.lower <= initial <= self.controller.motion.upper:
            raise ValueError('initial servo_cmd must fit the servo envelope')
        self.servo_cmd = [initial]
        self.lock = threading.RLock()
        self._stopping = threading.Event()
        self._last_transition = None
        self.servo_cmd_pub = rospy.Publisher('/servo/target_states', ServoControlCmd, queue_size=1)
        self.joy_sub = rospy.Subscriber('/joy', Joy, self._joyCallback, queue_size=1)
        self._thread = threading.Thread(target=self._run, name='sonic-motion', daemon=True)
        rospy.on_shutdown(self._shutdown)
        rospy.loginfo('Sonic motion: %.1f Hz wall clock, direct joystick mapping, Joy timeout %.3f s',
                      self.update_rate, self.controller.joy_timeout_s)
        self._thread.start()

    def _joyCallback(self, msg):
        with self.lock:
            if not self._stopping.is_set():
                self.controller.handle_joy(msg.axes, msg.buttons, time.monotonic())

    def publish_cmd(self, state):
        self.servo_cmd[0] = state.target
        msg = ServoControlCmd()
        msg.index = [0]
        msg.cmd = [int(state.target)]
        self.servo_cmd_pub.publish(msg)

    def _controlLoop(self):
        with self.lock:
            state = self.controller.tick(time.monotonic())
            if state is None:
                return
            self.publish_cmd(state)
            transition = (state.mode, state.reason)
            if transition != self._last_transition:
                logger = rospy.logwarn if state.mode == 'stopped' else rospy.loginfo
                logger('Sonic mode=%s reason=%s %s', state.mode, state.reason, self.controller.input_error)
                self._last_transition = transition
            rospy.loginfo_throttle(1., 'Sonic %s: bias target/applied=%.1f/%.1f, '
                                   'amplitude requested/effective=%.1f/%.1f, target=%.1f',
                                   state.mode, state.requested_bias, state.applied_bias,
                                   state.requested_amplitude, state.effective_amplitude, state.target)

    def _run(self):
        period = 1. / self.update_rate
        deadline = time.monotonic()
        while not self._stopping.is_set() and not rospy.is_shutdown():
            self._controlLoop()
            deadline += period
            now = time.monotonic()
            if deadline < now:
                deadline = now + period
            self._stopping.wait(max(0., deadline - now))

    def _shutdown(self):
        self._stopping.set()
        self.joy_sub.unregister()
        if self._thread.is_alive() and threading.current_thread() is not self._thread:
            self._thread.join(timeout=1.)
        with self.lock:
            if self.controller.tick(time.monotonic()) is not None:
                try:
                    self.publish_cmd(self.controller.stop('shutdown'))
                except rospy.ROSException as exc:
                    rospy.logwarn('Could not send sonic shutdown center: %s', exc)


if __name__ == '__main__':
    rospy.init_node('sonic_teleop')
    try:
        teleop_node = Teleop()
        rospy.spin()
    except (ValueError, TypeError, OverflowError) as exc:
        rospy.logfatal('Invalid sonic motion configuration: %s', exc)
        raise SystemExit(1)
