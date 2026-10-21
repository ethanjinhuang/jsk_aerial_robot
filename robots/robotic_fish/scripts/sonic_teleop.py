#!/usr/bin/env python3
"""Single-servo ROS adapter, with optional complete-cycle acoustic steering."""
import math
import threading
import time

import rospy
from sensor_msgs.msg import Joy
from spinal.msg import ServoControlCmd
from std_msgs.msg import String

from robotic_fish.sonic_motion import SonicController
from robotic_fish_sonic.acoustic_core import adapt, Status
from robotic_fish_sonic.msg import AdcSampleArray
from robotic_fish_sonic.status_display import format_status, write_snapshot


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
            'tracking_enabled': rospy.get_param('~acoustic_steering/enabled', False),
        }
        for name, default in [('steering_axis', 2), ('amplitude_decrease_axis', 3),
                              ('amplitude_increase_axis', 4), ('stop_button', 1),
                              ('auto_button', 3), ('manual_button', 0), ('tracking_button', 2),
                              ('amplitude_buttons', [6, 7])]:
            parameters[name] = rospy.get_param('~' + name, default)
        self.controller = SonicController(**parameters)
        self.status_local_echo = rospy.get_param('~status_local_echo', False)
        if not isinstance(self.status_local_echo, bool):
            raise ValueError('status_local_echo must be boolean')
        self.status_button = rospy.get_param('~status_button', 8)
        if (isinstance(self.status_button, bool) or not isinstance(self.status_button, int)
                or self.status_button < 0):
            raise ValueError('status_button must be a nonnegative integer')
        if self.status_button in (self.controller.stop_button, self.controller.auto_button,
                                  self.controller.manual_button, self.controller.tracking_button,
                                  *self.controller.amplitude_buttons):
            raise ValueError('status_button must not overlap motion buttons')
        max_age = rospy.get_param('~acoustic_steering/max_age_s', .2)
        future_tolerance = rospy.get_param('~acoustic_steering/future_tolerance_s', .05)
        if isinstance(max_age, bool) or isinstance(future_tolerance, bool):
            raise ValueError('ADC timing limits must be numbers, not booleans')
        self._status_max_age = float(max_age)
        self._status_future_tolerance = float(future_tolerance)
        if (not math.isfinite(self._status_max_age) or self._status_max_age <= 0
                or not math.isfinite(self._status_future_tolerance) or self._status_future_tolerance < 0):
            raise ValueError('ADC age limit must be positive and future tolerance nonnegative')
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
        self._status_button_prev = self._status_requested = False
        self._latest_adc = self._latest_adc_receipt = None
        self.acoustic = None
        self._acoustic_token = None
        self._acoustic_status = 'disabled'
        if self.controller.tracking_enabled:
            from robotic_fish_sonic.cycle_steering import CycleSteering, SteeringFault
            from robotic_fish_sonic.msg import AcousticSteeringState
            from robotic_fish_sonic.srv import SetAcousticSteering, SetAcousticSteeringResponse
            self._SteeringFault = SteeringFault
            self._AcousticState = AcousticSteeringState
            self._SteeringResponse = SetAcousticSteeringResponse
            self.acoustic = CycleSteering(
                frequency=self.controller.frequency, max_bias=self.controller.motion.servo_range,
                threshold=rospy.get_param('~acoustic_steering/threshold', .1),
                bias=rospy.get_param('~acoustic_steering/bias', 256.),
                lr_offset=rospy.get_param('~acoustic_steering/lr_offset', 0.),
                left_bias_sign=rospy.get_param('~acoustic_steering/left_bias_sign', 1),
                gap_s=rospy.get_param('~acoustic_steering/gap_s', .2),
                max_age_s=self._status_max_age,
                future_tolerance_s=self._status_future_tolerance,
                low_signal_v=rospy.get_param('~acoustic_steering/low_signal_v', .001))
            self.acoustic_pub = rospy.Publisher('/robotic_fish/acoustic/steering_state',
                                               AcousticSteeringState, queue_size=1)
            self.settings_service = rospy.Service('~set_acoustic_steering', SetAcousticSteering,
                                                   self._setAcousticSteering)
        # Keep instant readings available to Share even when tracking is disabled.
        self.adc_sub = rospy.Subscriber('/robotic_fish/adc/samples', AdcSampleArray,
                                        self._adcCallback, queue_size=100)
        self.status_pub = rospy.Publisher('/robotic_fish/sonic/status_snapshot', String,
                                          queue_size=10, latch=False)
        self.servo_cmd_pub = rospy.Publisher('/servo/target_states', ServoControlCmd, queue_size=1)
        self.joy_sub = rospy.Subscriber('/joy', Joy, self._joyCallback, queue_size=1)
        self._thread = threading.Thread(target=self._run, name='sonic-motion', daemon=True)
        rospy.on_shutdown(self._shutdown)
        rospy.loginfo('Sonic motion: %.1f Hz wall clock, direct joystick mapping, Joy timeout %.3f s',
                      self.update_rate, self.controller.joy_timeout_s)
        self._thread.start()

    def _adcCallback(self, msg):
        with self.lock:
            if not self._stopping.is_set():
                self._latest_adc, self._latest_adc_receipt = msg, time.monotonic()
                if self.acoustic is not None:
                    self.acoustic.push(msg, self._latest_adc_receipt, rospy.Time.now().to_sec())

    def _setAcousticSteering(self, request):
        with self.lock:
            if self._stopping.is_set():
                return self._SteeringResponse(False, 'motion node is shutting down')
            try:
                self.acoustic.configure(request.threshold, request.bias, request.lr_offset)
            except ValueError as exc:
                return self._SteeringResponse(False, str(exc))
            return self._SteeringResponse(True, 'accepted; applies at next valid full-cycle decision')

    def _joyCallback(self, msg):
        with self.lock:
            if not self._stopping.is_set():
                self.controller.handle_joy(msg.axes, msg.buttons, time.monotonic())
                try:
                    pressed = len(msg.buttons) > self.status_button and msg.buttons[self.status_button] == 1
                except TypeError:
                    pressed = False
                if pressed and not self._status_button_prev:
                    self._status_requested = True
                self._status_button_prev = pressed

    def _formatStatus(self, state, now):
        """Describe the actual held decision; never run a new steering decision."""
        data = dict(mode=state.mode, status=self._acoustic_status, reason=state.reason,
                    stamp=rospy.Time.now().to_sec(), raw=None, calibrated=None, means=None,
                    quality='no_data', sample_age=None, receipt_age=None, cycle=0,
                    historical=False, window=1. / self.controller.frequency, count=0,
                    delta=None, difference=None, compensated_difference=None, lr_offset=0.,
                    threshold=None, judged='unavailable',
                    bias=state.applied_bias, bias_limit=self.controller.motion.servo_range)
        if self._latest_adc is not None:
            samples = sorted(self._latest_adc.samples, key=lambda s: s.channel_id)
            if [s.channel_id for s in samples] != [0, 1, 2]:
                data['quality'] = 'invalid_input (expected channels 0,1,2)'
            else:
                data['raw'] = [getattr(sample, 'volt_raw', float('nan')) for sample in samples]
                data['calibrated'] = [sample.volt_cali for sample in samples]
                data['receipt_age'] = now - self._latest_adc_receipt
                try:
                    group = adapt(self._latest_adc)
                    data['sample_age'] = data['stamp'] - group['t']
                    data['quality'] = '|'.join(flag.name.lower() for flag in Status if flag & group['status']) or 'valid'
                    if data['sample_age'] > self._status_max_age or data['receipt_age'] > self._status_max_age:
                        data['quality'] = 'stale_sample'
                    elif data['sample_age'] < -self._status_future_tolerance:
                        data['quality'] = 'future_sample'
                except (ValueError, TypeError, AttributeError, OverflowError) as exc:
                    data['quality'] = 'invalid_input (%s)' % exc
        decision = None if self.acoustic is None else self.acoustic.last_decision
        active = state.mode == 'tracking' and self._acoustic_status == 'active'
        if self.acoustic is not None:
            data['threshold'] = self.acoustic.settings.threshold
            data['lr_offset'] = self.acoustic.settings.lr_offset
        if decision is not None:
            data.update(judged='left' if decision.compensated_difference > data['threshold'] else
                        'right' if decision.compensated_difference < -data['threshold'] else 'straight',
                        means=(decision.head_mean, decision.left_mean, decision.right_mean),
                        cycle=decision.cycle, historical=not active,
                        window=decision.end - decision.start, count=decision.sample_count,
                        delta=decision.left_mean - decision.right_mean, difference=decision.difference,
                        lr_offset=decision.lr_offset, compensated_difference=decision.compensated_difference)
        data['direction'] = data['judged'] if active else (
                            'holding' if state.mode == 'tracking' and self.acoustic.waiting_status else
                            'warmup' if state.mode == 'tracking' else
                            'stopped' if state.mode == 'stopped' else
                            'disabled' if self.acoustic is None else 'not_tracking')
        return format_status(data)

    def publish_cmd(self, state):
        self.servo_cmd[0] = state.target
        msg = ServoControlCmd()
        msg.index = [0]
        msg.cmd = [int(state.target)]
        self.servo_cmd_pub.publish(msg)

    def _acousticStep(self, state, now):
        updated = False
        if state is None or state.mode != 'tracking':
            self.acoustic.deactivate()
            self._acoustic_token = None
            self._acoustic_status = 'idle' if state is None else state.reason if state.mode == 'stopped' else state.mode
        else:
            token = (self.controller.auto_epoch, self.controller.frequency)
            try:
                if token != self._acoustic_token:
                    self.acoustic.begin(*token, now)
                    self._acoustic_token = token
                    self._acoustic_status = 'warmup'
                decision = self.acoustic.evaluate(now)
                if decision is not None:
                    self.controller.set_tracking_bias(decision.bias)
                    updated = True
                    self._acoustic_status = 'active'
                    rospy.logdebug('Acoustic cycle=%d, L/R=%.5f/%.5f, difference=%.3f, bias=%.1f',
                                  decision.cycle, decision.left_mean, decision.right_mean,
                                  decision.difference, decision.bias)
                elif self.acoustic.waiting_status:
                    self._acoustic_status = self.acoustic.waiting_status
            except self._SteeringFault as exc:
                self._acoustic_status = 'acoustic_' + str(exc)
                self.controller.stop(self._acoustic_status)
                self.acoustic.deactivate()
                self._acoustic_token = None
            state = self.controller.tick(now)
        self._publishAcousticState(state, now, updated)
        return state

    def _publishAcousticState(self, state, now, updated):
        msg = self._AcousticState()
        msg.header.stamp = rospy.Time.now()
        msg.header.frame_id = 'fish_base_link'
        msg.enabled = True
        msg.motion_mode = 'idle' if state is None else state.mode
        msg.status, msg.decision_updated = self._acoustic_status, updated
        msg.parameters_pending = self.acoustic.pending is not None
        msg.frequency, msg.window_s = self.acoustic.frequency, self.acoustic.period
        msg.threshold, msg.turn_bias = self.acoustic.settings.threshold, self.acoustic.settings.bias
        msg.lr_offset = self.acoustic.settings.lr_offset
        msg.left_bias_sign = self.acoustic.left_bias_sign
        msg.command_bias = 0. if state is None else state.applied_bias
        msg.left_mean = msg.right_mean = msg.normalized_difference = float('nan')
        msg.compensated_difference = float('nan')
        if self.acoustic.latest_stamp is not None:
            msg.sample_stamp = rospy.Time.from_sec(self.acoustic.latest_stamp)
        decision = self.acoustic.last_decision
        if decision is not None:
            # Convert monotonic boundaries using the ADC/ROS clock mapping.
            offset = self.acoustic.epoch_clock_offset
            msg.window_start = rospy.Time.from_sec(max(0., decision.start - offset))
            msg.window_end = rospy.Time.from_sec(max(0., decision.end - offset))
            msg.cycle_index, msg.sample_count = decision.cycle, decision.sample_count
            msg.left_mean, msg.right_mean = decision.left_mean, decision.right_mean
            msg.normalized_difference = decision.difference
            msg.compensated_difference = decision.compensated_difference
        self.acoustic_pub.publish(msg)

    def _controlLoop(self):
        report = None
        with self.lock:
            now = time.monotonic()
            state = self.controller.tick(now)
            if self.acoustic is not None:
                state = self._acousticStep(state, now)
            if state is None:
                return
            self.publish_cmd(state)
            transition = (state.mode, state.reason)
            if transition != self._last_transition:
                logger = rospy.logwarn if state.mode == 'stopped' else rospy.loginfo
                logger('Sonic mode=%s reason=%s %s', state.mode, state.reason, self.controller.input_error)
                self._last_transition = transition
            if self._status_requested:
                self._status_requested = False
                report = self._formatStatus(state, now)
        if report is not None:
            self.status_pub.publish(String(data=report))
            if self.status_local_echo:
                write_snapshot(report)

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
        self.adc_sub.unregister()
        if self.acoustic is not None:
            self.settings_service.shutdown()
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
