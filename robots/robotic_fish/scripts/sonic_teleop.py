#!/usr/bin/env python3
"""Single-servo ROS adapter, with optional complete-cycle acoustic steering."""
import json
import math
import socket
import threading
import time

import rospy
from sensor_msgs.msg import Joy
from spinal.msg import ServoControlCmd
from std_msgs.msg import String

from robotic_fish.sonic_motion import SonicController
from robotic_fish_sonic.acoustic_core import adapt, Status
from robotic_fish_sonic.cycle_steering import DIFFERENCE_REFERENCE, SteeringSettings, steering_direction
from robotic_fish_sonic.msg import AdcSampleArray, SonicMotionState, RuntimeConfig, AcousticSteeringEvent
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
        self._initial_servo_cmd = initial
        self.lock = threading.RLock()
        self._stopping = threading.Event()
        self._last_transition = None
        self._status_button_prev = self._status_requested = False
        self._latest_adc = self._latest_adc_receipt = None
        self._latest_adc_ros = self._latest_adc_group = None
        self._adc_error = ''
        self._adc_receipt_interval = float('nan')
        self._latest_joy_receipt = self._previous_control = None
        self._initial_amplitude = self.controller.amplitude
        self._started_monotonic = time.monotonic()
        self._started_ros = rospy.Time.now().to_sec()
        self._config_payload = None
        self._request_id = self._applied_request_id = 0
        self._pending_request = None
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
                lr_voltage_offset_v=rospy.get_param('~acoustic_steering/lr_voltage_offset_v', 0.),
                left_bias_sign=rospy.get_param('~acoustic_steering/left_bias_sign', 1),
                gap_s=rospy.get_param('~acoustic_steering/gap_s', .2),
                max_age_s=self._status_max_age,
                future_tolerance_s=self._status_future_tolerance,
                low_signal_v=rospy.get_param('~acoustic_steering/low_signal_v', .001))
            self.acoustic_pub = rospy.Publisher('/robotic_fish/acoustic/steering_state',
                                               AcousticSteeringState, queue_size=1)
        self.motion_pub = rospy.Publisher('/robotic_fish/sonic/motion_state', SonicMotionState, queue_size=10)
        self.config_pub = rospy.Publisher('/robotic_fish/sonic/config', RuntimeConfig, queue_size=1, latch=True)
        self.event_pub = rospy.Publisher('/robotic_fish/acoustic/steering_events',
                                         AcousticSteeringEvent, queue_size=50)
        self._publishConfig()
        if self.acoustic is not None:
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
        rospy.loginfo('Sonic host=%s; ADC max age=%.3f s, future tolerance=%.3f s; Joy uses local receipt time',
                      socket.gethostname(), self._status_max_age, self._status_future_tolerance)
        self._thread.start()

    def _adcCallback(self, msg):
        with self.lock:
            if not self._stopping.is_set():
                previous = self._latest_adc_receipt
                self._latest_adc, self._latest_adc_receipt = msg, time.monotonic()
                self._latest_adc_ros = rospy.Time.now()
                self._adc_receipt_interval = (float('nan') if previous is None else
                                              self._latest_adc_receipt - previous)
                try:
                    self._latest_adc_group = adapt(msg)
                    self._adc_error = ''
                except (ValueError, TypeError, AttributeError, OverflowError) as exc:
                    self._latest_adc_group = None
                    self._adc_error = 'invalid_input (%s)' % exc
                if self.acoustic is not None:
                    self.acoustic.push(msg, self._latest_adc_receipt, self._latest_adc_ros.to_sec())

    def _publishConfig(self):
        """Latched effective values, including pending settings for late recorders."""
        c = self.controller
        settings = dict(servo_center=c.motion.center, servo_range=c.motion.servo_range,
                        servo_index=0, auto_frequency=c.frequency,
                        auto_default_amplitude=c.default_amplitude, auto_amplitude=self._initial_amplitude,
                        auto_amplitude_rate=c.amplitude_rate, joy_timeout_s=c.joy_timeout_s,
                        joy_dead_zone=c.dead_zone, update_rate=self.update_rate,
                        steering_axis=c.steering_axis, amplitude_decrease_axis=c.decrease_axis,
                        amplitude_increase_axis=c.increase_axis, amplitude_buttons=c.amplitude_buttons,
                        manual_button=c.manual_button, auto_button=c.auto_button,
                        tracking_button=c.tracking_button, stop_button=c.stop_button,
                        status_button=self.status_button, status_local_echo=self.status_local_echo,
                        initial_servo_cmd=self._initial_servo_cmd,
                        acoustic_steering=dict(enabled=self.acoustic is not None,
                            difference_reference=DIFFERENCE_REFERENCE, threshold_inclusive=True,
                            max_age_s=self._status_max_age, future_tolerance_s=self._status_future_tolerance))
        if self.acoustic is not None:
            a = self.acoustic
            settings['acoustic_steering'].update(vars(a.settings), left_bias_sign=a.left_bias_sign,
                gap_s=a.gap_s, low_signal_v=a.low_signal_v, max_bias=a.max_bias)
        payload = json.dumps(dict(schema_version=3, host=socket.gethostname(),
            started_ros_s=self._started_ros, started_monotonic_s=self._started_monotonic,
            settings=settings, applied_request_id=self._applied_request_id,
            pending_request=None if self._pending_request is None else dict(
                request_id=self._pending_request[0], **vars(self._pending_request[1]))),
            sort_keys=True, allow_nan=False)
        if payload != self._config_payload:
            msg = RuntimeConfig()
            msg.timestamp, msg.device, msg.config_json = rospy.Time.now(), 'sonic_motion', payload
            self.config_pub.publish(msg)
            self._config_payload = payload

    def _publishEvent(self, request_id, event, values, detail, cycle=0):
        msg = AcousticSteeringEvent()
        msg.header.stamp = rospy.Time.now()
        msg.header.frame_id = 'fish_base_link'
        msg.request_id, msg.event, msg.detail = request_id, event, detail
        msg.difference_reference = DIFFERENCE_REFERENCE
        msg.threshold, msg.turn_bias, msg.lr_voltage_offset_v = values.threshold, values.bias, values.lr_voltage_offset_v
        msg.cycle_index = cycle
        self.event_pub.publish(msg)

    def _setAcousticSteering(self, request):
        with self.lock:
            self._request_id += 1
            request_id = self._request_id
            # ROS service fields are float64; retain even a rejected NaN/Inf request.
            values = SteeringSettings(request.threshold, request.bias, request.lr_voltage_offset_v)
            if self._stopping.is_set():
                self._publishEvent(request_id, 'rejected', values, 'motion node is shutting down')
                return self._steeringResponse(False, 'motion node is shutting down', request_id)
            try:
                self.acoustic.configure(request.threshold, request.bias, request.lr_voltage_offset_v)
            except ValueError as exc:
                self._publishEvent(request_id, 'rejected', values, str(exc))
                return self._steeringResponse(False, str(exc), request_id)
            if self._pending_request is not None:
                old_id, old_values = self._pending_request
                self._publishEvent(old_id, 'superseded', old_values, 'replaced by request %d' % request_id)
            self._pending_request = (request_id, self.acoustic.pending)
            self._publishEvent(request_id, 'accepted', values, 'applies at next valid full-cycle decision')
            self._publishConfig()
            return self._steeringResponse(True, 'accepted; applies at next valid full-cycle decision', request_id)

    def _steeringResponse(self, success, message, request_id):
        """Report the current controller state without advancing it or commanding motion."""
        response = self._SteeringResponse(success=success, message=message)
        response.request_id = request_id
        response.difference_reference = DIFFERENCE_REFERENCE
        response.motion_mode = self.controller.mode
        response.status, response.reason = self._acoustic_status, self.controller.reason
        response.direction = self._directions(self.controller)[2]
        response.adc_quality = self._adcStatus(time.monotonic(), rospy.Time.now().to_sec())['quality']
        response.command_bias = self.controller.applied_bias
        active, pending = self.acoustic.settings, self.acoustic.pending
        response.active_threshold, response.active_bias, response.active_lr_voltage_offset_v = (
            active.threshold, active.bias, active.lr_voltage_offset_v)
        response.parameters_pending = pending is not None
        response.pending_request_id = 0 if self._pending_request is None else self._pending_request[0]
        response.pending_threshold = float('nan') if pending is None else pending.threshold
        response.pending_bias = float('nan') if pending is None else pending.bias
        response.pending_lr_voltage_offset_v = float('nan') if pending is None else pending.lr_voltage_offset_v
        return response

    def _joyCallback(self, msg):
        with self.lock:
            if not self._stopping.is_set():
                self._latest_joy_receipt = time.monotonic()
                self.controller.handle_joy(msg.axes, msg.buttons, self._latest_joy_receipt)
                try:
                    pressed = len(msg.buttons) > self.status_button and msg.buttons[self.status_button] == 1
                except TypeError:
                    pressed = False
                if pressed and not self._status_button_prev:
                    self._status_requested = True
                self._status_button_prev = pressed

    def _adcStatus(self, now, ros_now):
        """Describe the latest received frame, independently of held decisions."""
        result = dict(quality='no_data', flags=0, source_age=float('nan'),
                      source_age_at_receipt=float('nan'), receipt_age=float('nan'))
        if self._latest_adc is None:
            return result
        result['receipt_age'] = now - self._latest_adc_receipt
        group = self._latest_adc_group
        if group is None:
            result.update(quality=self._adc_error, flags=int(Status.INVALID_INPUT))
            return result
        result.update(source_age=ros_now - group['t'],
                      source_age_at_receipt=self._latest_adc_ros.to_sec() - group['t'],
                      flags=int(group['status']),
                      quality='|'.join(flag.name.lower() for flag in Status if flag & group['status']) or 'valid')
        if result['source_age'] > self._status_max_age or result['receipt_age'] > self._status_max_age:
            result['quality'] = 'stale_sample'
        elif result['source_age'] < -self._status_future_tolerance:
            result['quality'] = 'future_sample'
        return result

    def _directions(self, state):
        decision = None if self.acoustic is None else self.acoustic.last_decision
        current = (decision is not None and state is not None and
                   state.mode == 'tracking' and self._acoustic_status == 'active')
        judged = ('unavailable' if decision is None else
                  {1: 'left', -1: 'right', 0: 'straight'}[steering_direction(
                      decision.compensated_difference, self.acoustic.settings.threshold)])
        direction = (judged if current else 'idle' if state is None else
                     'holding' if state.mode == 'tracking' and self.acoustic.waiting_status else
                     'warmup' if state.mode == 'tracking' else 'stopped' if state.mode == 'stopped' else
                     'disabled' if self.acoustic is None else 'not_tracking')
        return current, judged, direction

    def _publishMotionState(self, state, now, stamp):
        msg = SonicMotionState()
        msg.header.stamp, msg.header.frame_id = stamp, 'fish_base_link'
        msg.monotonic_s = now
        msg.control_interval_s = float('nan') if self._previous_control is None else now - self._previous_control
        self._previous_control = now
        msg.command_published = state is not None
        msg.motion_mode = 'idle' if state is None else state.mode
        msg.reason = self.controller.reason if state is None else state.reason
        msg.input_error = self.controller.input_error
        msg.target_position = float('nan') if state is None else state.target
        msg.target_command = 0 if state is None else int(state.target)
        msg.requested_bias = 0. if state is None else state.requested_bias
        msg.applied_bias = 0. if state is None else state.applied_bias
        msg.requested_amplitude = self.controller.amplitude if state is None else state.requested_amplitude
        msg.effective_amplitude = 0. if state is None else state.effective_amplitude
        msg.frequency = self.controller.frequency
        msg.gait_elapsed_s = msg.phase_rad = float('nan')
        if state is not None and state.mode in ('auto', 'tracking'):
            msg.gait_elapsed_s = now - self.controller.auto_epoch
            msg.phase_rad = (2. * math.pi * msg.frequency * msg.gait_elapsed_s) % (2. * math.pi)
        msg.joy_receipt_age_s = (float('nan') if self._latest_joy_receipt is None else
                                 now - self._latest_joy_receipt)
        msg.valid_joy_age_s = (float('nan') if self.controller.last_joy_time is None else
                               now - self.controller.last_joy_time)
        msg.acoustic_enabled, msg.acoustic_status = self.acoustic is not None, self._acoustic_status
        msg.has_adc, msg.adc_structurally_valid = self._latest_adc is not None, self._latest_adc_group is not None
        if self._latest_adc_ros is not None:
            msg.adc_receipt_stamp = self._latest_adc_ros
        if self._latest_adc_group is not None:
            msg.adc_sample_stamp = rospy.Time.from_sec(self._latest_adc_group['t'])
        adc = self._adcStatus(now, stamp.to_sec())
        msg.adc_source_age_s, msg.adc_source_age_at_receipt_s = adc['source_age'], adc['source_age_at_receipt']
        msg.adc_receipt_age_s, msg.adc_receipt_interval_s = adc['receipt_age'], self._adc_receipt_interval
        msg.adc_quality_flags, msg.adc_quality = adc['flags'], adc['quality']
        msg.acoustic_input_error = '' if self.acoustic is None else self.acoustic.fault or self.acoustic.latest_error
        self.motion_pub.publish(msg)

    def _formatStatus(self, state, now):
        """Describe the actual held decision; never run a new steering decision."""
        data = dict(mode=state.mode, status=self._acoustic_status, reason=state.reason,
                    raw=None, raw_means=None, means=None, historical=False,
                    difference=None, compensated_difference=None,
                    lr_voltage_offset_v=0., pending_lr_voltage_offset_v=None,
                    threshold=None,
                    bias=state.applied_bias, bias_limit=self.controller.motion.servo_range)
        data['quality'] = self._adcStatus(now, rospy.Time.now().to_sec())['quality']
        if self._latest_adc_group is not None:
            samples = sorted(self._latest_adc.samples, key=lambda s: s.channel_id)
            data['raw'] = [getattr(sample, 'volt_raw', float('nan')) for sample in samples]
        decision = None if self.acoustic is None else self.acoustic.last_decision
        active, _, direction = self._directions(state)
        if self.acoustic is not None:
            data['threshold'] = self.acoustic.settings.threshold
            data['lr_voltage_offset_v'] = self.acoustic.settings.lr_voltage_offset_v
            if self.acoustic.pending is not None:
                data['pending_lr_voltage_offset_v'] = self.acoustic.pending.lr_voltage_offset_v
        if decision is not None:
            data.update(raw_means=decision.raw_means,
                        means=(decision.head_mean, decision.left_mean, decision.right_mean),
                        historical=not active,
                        lr_voltage_offset_v=decision.lr_voltage_offset_v,
                        difference=decision.difference,
                        compensated_difference=decision.compensated_difference)
        data['direction'] = direction
        return format_status(data)

    def publish_cmd(self, state):
        self.servo_cmd[0] = state.target
        msg = ServoControlCmd()
        msg.index = [0]
        msg.cmd = [int(state.target)]
        self.servo_cmd_pub.publish(msg)

    def _acousticStep(self, state, now, stamp):
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
                    if self._pending_request is not None and self.acoustic.pending is None:
                        request_id, values = self._pending_request
                        self._applied_request_id = request_id
                        self._pending_request = None
                        self._publishEvent(request_id, 'applied', values,
                                           'effective at full-cycle decision', decision.cycle)
                        self._publishConfig()
                        rospy.loginfo('Acoustic steering applied: request=%d cycle=%d '
                                      'threshold=%.4f bias=%.1f lr_voltage_offset_v=%+.6f V',
                                      request_id, decision.cycle, values.threshold, values.bias, values.lr_voltage_offset_v)
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
        self._publishAcousticState(state, stamp, updated)
        return state

    def _publishAcousticState(self, state, stamp, updated):
        msg = self._AcousticState()
        msg.header.stamp = stamp
        msg.header.frame_id = 'fish_base_link'
        msg.enabled = True
        msg.motion_mode = 'idle' if state is None else state.mode
        msg.status, msg.decision_updated = self._acoustic_status, updated
        msg.parameters_pending = self.acoustic.pending is not None
        msg.frequency, msg.window_s = self.acoustic.frequency, self.acoustic.period
        msg.threshold, msg.turn_bias = self.acoustic.settings.threshold, self.acoustic.settings.bias
        msg.difference_reference = DIFFERENCE_REFERENCE
        msg.lr_voltage_offset_v = self.acoustic.settings.lr_voltage_offset_v
        msg.left_bias_sign = self.acoustic.left_bias_sign
        msg.command_bias = 0. if state is None else state.applied_bias
        msg.left_mean = msg.right_mean = msg.normalized_difference = float('nan')
        msg.compensated_difference = float('nan')
        msg.head_mean = msg.voltage_difference = float('nan')
        msg.compensated_left_mean = msg.compensated_right_mean = float('nan')
        msg.compensated_voltage_difference = float('nan')
        if self.acoustic.latest_stamp is not None:
            msg.sample_stamp = rospy.Time.from_sec(self.acoustic.latest_stamp)
        decision = self.acoustic.last_decision
        msg.has_decision = decision is not None
        msg.decision_current, msg.decision_direction, msg.direction = self._directions(state)
        if decision is not None:
            # Convert monotonic boundaries using the ADC/ROS clock mapping.
            offset = self.acoustic.epoch_clock_offset
            msg.window_start = rospy.Time.from_sec(max(0., decision.start - offset))
            msg.window_end = rospy.Time.from_sec(max(0., decision.end - offset))
            msg.cycle_index, msg.sample_count = decision.cycle, decision.sample_count
            msg.left_mean, msg.right_mean = decision.left_mean, decision.right_mean
            msg.head_mean = decision.head_mean
            msg.voltage_difference = decision.left_mean - decision.right_mean
            msg.compensated_left_mean, msg.compensated_right_mean = (
                decision.compensated_left_mean, decision.compensated_right_mean)
            msg.compensated_voltage_difference = decision.compensated_left_mean - decision.compensated_right_mean
            msg.normalized_difference = decision.difference
            msg.compensated_difference = decision.compensated_difference
        self.acoustic_pub.publish(msg)

    def _controlLoop(self):
        report = None
        with self.lock:
            now = time.monotonic()
            stamp = rospy.Time.now()
            state = self.controller.tick(now)
            if self.acoustic is not None:
                state = self._acousticStep(state, now, stamp)
            if state is not None:
                self.publish_cmd(state)
            self._publishMotionState(state, now, stamp)
            if state is None:
                return
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
            try:
                # Optional recording must not prevent the original shutdown center command.
                now, stamp = time.monotonic(), rospy.Time.now()
                state = self.controller.tick(now)
                if state is not None:
                    state = self.controller.stop('shutdown')
                    self.publish_cmd(state)
                if self._pending_request is not None:
                    request_id, values = self._pending_request
                    self._publishEvent(request_id, 'cancelled', values, 'motion node is shutting down')
                    self._pending_request = None
                    self.acoustic.pending = None
                    self._publishConfig()
                self._acoustic_status = 'shutdown'
                if self.acoustic is not None:
                    self.acoustic.deactivate()
                    self._publishAcousticState(state, stamp, False)
                self._publishMotionState(state, now, stamp)
            except rospy.ROSException as exc:
                rospy.logwarn('Could not publish sonic shutdown state: %s', exc)


if __name__ == '__main__':
    rospy.init_node('sonic_teleop')
    try:
        teleop_node = Teleop()
        rospy.spin()
    except (ValueError, TypeError, OverflowError) as exc:
        rospy.logfatal('Invalid sonic motion configuration: %s', exc)
        raise SystemExit(1)
