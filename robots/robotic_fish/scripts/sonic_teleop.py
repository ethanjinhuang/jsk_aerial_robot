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
from robotic_fish_sonic.channel_mapping import channel_mapping, head_left_right, layout_from_pzt_channels
from robotic_fish_sonic.cycle_steering import (
    DIFFERENCE_REFERENCE, CycleVoltageWindow, SteeringSettings, number, steering_direction,
)
from robotic_fish_sonic.msg import AdcSampleArray, SonicMotionState, RuntimeConfig, AcousticSteeringEvent
from robotic_fish_sonic.status_display import format_status, write_snapshot


class Teleop:
    def __init__(self):
        self.channel_layout = layout_from_pzt_channels(rospy.get_param('~hydrophone_channels', None))
        parameters = {
            'center': rospy.get_param('~servo_center', 2048.),
            'servo_range': rospy.get_param('~servo_range', 1024.),
            'frequency': rospy.get_param('~auto_frequency', 2.),
            'default_amplitude': rospy.get_param('~auto_default_amplitude', 512.),
            'amplitude': rospy.get_param('~auto_amplitude', None),
            'amplitude_rate': rospy.get_param('~auto_amplitude_rate', 64.),
            'joy_timeout_s': rospy.get_param('~joy_timeout_s', .5),
            'joy_timeout_action': rospy.get_param('~joy_timeout_action', 'stop'),
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
        self.polarity_toggle_button = rospy.get_param('~polarity_toggle_button', 9)
        if (isinstance(self.polarity_toggle_button, bool)
                or not isinstance(self.polarity_toggle_button, int) or self.polarity_toggle_button < 0):
            raise ValueError('polarity_toggle_button must be a nonnegative integer')
        if self.polarity_toggle_button in (self.status_button, self.controller.stop_button,
                self.controller.auto_button, self.controller.manual_button,
                self.controller.tracking_button, *self.controller.amplitude_buttons):
            raise ValueError('polarity_toggle_button must not overlap motion or status buttons')
        max_age = rospy.get_param('~acoustic_steering/max_age_s', .2)
        future_tolerance = rospy.get_param('~acoustic_steering/future_tolerance_s', .05)
        if isinstance(max_age, bool) or isinstance(future_tolerance, bool):
            raise ValueError('ADC timing limits must be numbers, not booleans')
        self._status_max_age = float(max_age)
        self._status_future_tolerance = float(future_tolerance)
        if (not math.isfinite(self._status_max_age) or self._status_max_age <= 0
                or not math.isfinite(self._status_future_tolerance) or self._status_future_tolerance < 0):
            raise ValueError('ADC age limit must be positive and future tolerance nonnegative')
        gap_s = rospy.get_param('~acoustic_steering/gap_s', .2)
        low_signal_v = rospy.get_param('~acoustic_steering/low_signal_v', .001)
        self._status_window = CycleVoltageWindow(
            self.controller.frequency, gap_s, self._status_max_age, self._status_future_tolerance,
            low_signal_v, self.channel_layout)
        self._status_threshold = number(rospy.get_param('~acoustic_steering/threshold', .15), 'threshold')
        self._status_offset = number(rospy.get_param('~acoustic_steering/lr_voltage_offset_v', 0.), 'lr_voltage_offset_v')
        self._status_ratio_offset = number(rospy.get_param('~acoustic_steering/lr_ratio_offset', 0.), 'lr_ratio_offset')
        if not 0 <= self._status_threshold <= 1:
            raise ValueError('threshold must be in [0,1]')
        if not -.5 <= self._status_ratio_offset <= .5:
            raise ValueError('lr_ratio_offset must be in [-0.5,0.5]')
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
        self._polarity_button_prev = False
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
                threshold=self._status_threshold,
                bias=rospy.get_param('~acoustic_steering/bias', 256.),
                lr_voltage_offset_v=self._status_offset,
                lr_ratio_offset=self._status_ratio_offset,
                left_bias_sign=rospy.get_param('~acoustic_steering/left_bias_sign', -1),
                gap_s=gap_s,
                max_age_s=self._status_max_age,
                future_tolerance_s=self._status_future_tolerance,
                low_signal_v=low_signal_v,
                channel_layout=self.channel_layout)
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
        # ADC telemetry is collected in every motion mode.
        self.adc_sub = rospy.Subscriber('/robotic_fish/adc/samples', AdcSampleArray,
                                        self._adcCallback, queue_size=100)
        self.status_pub = rospy.Publisher('/robotic_fish/sonic/status_snapshot', String,
                                          queue_size=10, latch=False)
        self.servo_cmd_pub = rospy.Publisher('/servo/target_states', ServoControlCmd, queue_size=1)
        self.joy_sub = rospy.Subscriber('/joy', Joy, self._joyCallback, queue_size=1)
        self._thread = threading.Thread(target=self._run, name='sonic-motion', daemon=True)
        rospy.on_shutdown(self._shutdown)
        rospy.loginfo('Sonic motion: %.1f Hz wall clock, direct joystick mapping, Joy timeout %.3f s, action=%s',
                      self.update_rate, self.controller.joy_timeout_s, self.controller.joy_timeout_action)
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
                ros_now = self._latest_adc_ros.to_sec()
                self._status_window.push(self._latest_adc_group, ros_now)
                if self.acoustic is not None:
                    self.acoustic.push(msg, self._latest_adc_receipt, ros_now)

    def _publishConfig(self):
        """Latched effective values, including pending settings for late recorders."""
        c = self.controller
        settings = dict(channel_layout=self.channel_layout, hydrophone_channels=channel_mapping(self.channel_layout),
                        servo_center=c.motion.center, servo_range=c.motion.servo_range,
                        servo_index=0, auto_frequency=c.frequency,
                        auto_default_amplitude=c.default_amplitude, auto_amplitude=self._initial_amplitude,
                        auto_amplitude_rate=c.amplitude_rate, joy_timeout_s=c.joy_timeout_s,
                        joy_timeout_action=c.joy_timeout_action,
                        joy_dead_zone=c.dead_zone, update_rate=self.update_rate,
                        steering_axis=c.steering_axis, amplitude_decrease_axis=c.decrease_axis,
                        amplitude_increase_axis=c.increase_axis, amplitude_buttons=c.amplitude_buttons,
                        manual_button=c.manual_button, auto_button=c.auto_button,
                        tracking_button=c.tracking_button, stop_button=c.stop_button,
                        status_button=self.status_button, status_local_echo=self.status_local_echo,
                        polarity_toggle_button=self.polarity_toggle_button,
                        initial_servo_cmd=self._initial_servo_cmd,
                        acoustic_steering=dict(enabled=self.acoustic is not None,
                            difference_reference=DIFFERENCE_REFERENCE, threshold_inclusive=True,
                            voltage_offset_reference='left_minus_right',
                            threshold_reference='full_bar_width',
                            ratio_offset_reference='left_plus_right_minus',
                            max_age_s=self._status_max_age, future_tolerance_s=self._status_future_tolerance))
        if self.acoustic is not None:
            a = self.acoustic
            settings['acoustic_steering'].update(vars(a.settings), left_bias_sign=a.left_bias_sign,
                gap_s=a.gap_s, low_signal_v=a.low_signal_v, max_bias=a.max_bias)
        payload = json.dumps(dict(schema_version=7, host=socket.gethostname(),
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
        msg.left_bias_sign = values.left_bias_sign
        msg.lr_ratio_offset = values.lr_ratio_offset
        msg.cycle_index = cycle
        self.event_pub.publish(msg)

    def _setAcousticSteering(self, request, source=None):
        with self.lock:
            self._request_id += 1
            request_id = self._request_id
            # Retain rejected values, including NaN/Inf in the float fields.
            requested_sign = request.left_bias_sign
            resolved_sign = self.acoustic.left_bias_sign if requested_sign == 0 else requested_sign
            ratio_offset = getattr(request, 'lr_ratio_offset', self.acoustic.settings.lr_ratio_offset)
            values = SteeringSettings(request.threshold, request.bias, request.lr_voltage_offset_v, resolved_sign, ratio_offset)
            if self._stopping.is_set():
                self._publishEvent(request_id, 'rejected', values, 'motion node is shutting down')
                return self._steeringResponse(False, 'motion node is shutting down', request_id)
            try:
                self.acoustic.configure(request.threshold, request.bias, request.lr_voltage_offset_v, resolved_sign, ratio_offset)
            except ValueError as exc:
                self._publishEvent(request_id, 'rejected', values, str(exc))
                return self._steeringResponse(False, str(exc), request_id)
            if self._pending_request is not None:
                old_id, old_values = self._pending_request
                self._publishEvent(old_id, 'superseded', old_values, 'replaced by request %d' % request_id)
            self._pending_request = (request_id, self.acoustic.pending)
            detail = 'applies at next valid full-cycle decision'
            if source is not None:
                detail = 'source=%s; %s' % (source, detail)
            self._publishEvent(request_id, 'accepted', values, detail)
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
        response.active_left_bias_sign = active.left_bias_sign
        response.active_lr_ratio_offset = active.lr_ratio_offset
        response.parameters_pending = pending is not None
        response.pending_request_id = 0 if self._pending_request is None else self._pending_request[0]
        response.pending_threshold = float('nan') if pending is None else pending.threshold
        response.pending_bias = float('nan') if pending is None else pending.bias
        response.pending_lr_voltage_offset_v = float('nan') if pending is None else pending.lr_voltage_offset_v
        response.pending_left_bias_sign = 0 if pending is None else pending.left_bias_sign
        response.pending_lr_ratio_offset = float('nan') if pending is None else pending.lr_ratio_offset
        return response

    def _joyCallback(self, msg):
        with self.lock:
            if not self._stopping.is_set():
                self._latest_joy_receipt = time.monotonic()
                self.controller.handle_joy(msg.axes, msg.buttons, self._latest_joy_receipt)
                self._polarityButton(msg)
                try:
                    pressed = len(msg.buttons) > self.status_button and msg.buttons[self.status_button] == 1
                except TypeError:
                    pressed = False
                if pressed and not self._status_button_prev:
                    self._status_requested = True
                self._status_button_prev = pressed

    def _polarityButton(self, msg):
        """Queue one polarity flip per valid press; preserve pending float edits."""
        try:
            self.controller._validate_joy(msg.axes, msg.buttons)
            if (len(msg.buttons) <= self.polarity_toggle_button
                    or msg.buttons[self.polarity_toggle_button] not in (0, 1)):
                return
        except (ValueError, TypeError, IndexError):
            # Missing/invalid frames must not re-arm a held Option on reconnect.
            return
        pressed = msg.buttons[self.polarity_toggle_button] == 1
        edge = pressed and not self._polarity_button_prev
        self._polarity_button_prev = pressed
        if not edge or self.acoustic is None:
            return
        c = self.controller
        if (c.last_joy_time != self._latest_joy_receipt or c.input_error
                or msg.buttons[c.stop_button] == 1
                or sum(msg.buttons[i] for i in (c.auto_button, c.manual_button, c.tracking_button)) > 1):
            return
        base = self.acoustic.pending or self.acoustic.settings
        request = SteeringSettings(base.threshold, base.bias, base.lr_voltage_offset_v, -base.left_bias_sign,
                                   base.lr_ratio_offset)
        response = self._setAcousticSteering(request, source='joystick_option')
        if response.success:
            rospy.loginfo('Option requested sound mode %s (left_bias_sign=%+d), request=%d; '
                          'waiting for a valid full cycle',
                          'FOLLOW' if request.left_bias_sign == -1 else 'AVOID',
                          request.left_bias_sign, response.request_id)

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

    def _cycleStatus(self, adc):
        settings = None if self.acoustic is None else self.acoustic.settings
        threshold = self._status_threshold if settings is None else settings.threshold
        offset = self._status_offset if settings is None else settings.lr_voltage_offset_v
        ratio_offset = self._status_ratio_offset if settings is None else settings.lr_ratio_offset
        stats = None
        if adc['quality'] not in ('stale_sample', 'future_sample'):
            stats = self._status_window.snapshot(offset, ratio_offset)
        return stats, threshold, offset, ratio_offset

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
        (cycle, msg.adc_cycle_threshold, msg.adc_cycle_lr_voltage_offset_v,
         msg.adc_cycle_lr_ratio_offset) = self._cycleStatus(adc)
        msg.adc_cycle_window_s = self._status_window.period
        msg.adc_cycle_ready = cycle is not None
        msg.adc_cycle_sample_count = 0
        msg.adc_cycle_quality_flags = int(Status.WARMUP) | adc['flags']
        if adc['quality'] == 'stale_sample':
            msg.adc_cycle_quality_flags |= int(Status.STALE)
        elif adc['quality'] == 'future_sample':
            msg.adc_cycle_quality_flags |= int(Status.INVALID_INPUT)
        missing = float('nan')
        msg.adc_cycle_means = msg.adc_cycle_raw_means = (missing,) * 3
        msg.adc_cycle_side_mean = msg.adc_cycle_difference = msg.adc_cycle_compensated_difference = missing
        if cycle is not None:
            msg.adc_cycle_sample_count = cycle['sample_count']
            msg.adc_cycle_quality_flags = int(cycle['flags'])
            msg.adc_cycle_means, msg.adc_cycle_raw_means = cycle['means'], cycle['raw_means']
            msg.adc_cycle_side_mean = cycle['side_mean']
            msg.adc_cycle_difference = cycle['difference']
            msg.adc_cycle_compensated_difference = cycle['compensated_difference']
        self.motion_pub.publish(msg)

    def _formatStatus(self, state, now):
        """Report the latest voltage window alongside the current motion state."""
        adc = self._adcStatus(now, rospy.Time.now().to_sec())
        cycle, threshold, offset, ratio_offset = self._cycleStatus(adc)
        data = dict(mode=state.mode, status=self._acoustic_status, reason=state.reason,
                    raw=None, raw_means=None, means=None, mean_1s=None, mean_1s_status=None,
                    difference=None, compensated_difference=None, cycle_quality=None,
                    lr_voltage_offset_v=offset, pending_lr_voltage_offset_v=None,
                    lr_ratio_offset=ratio_offset, pending_lr_ratio_offset=None,
                    left_bias_sign=None, pending_left_bias_sign=None,
                    threshold=threshold, quality=adc['quality'],
                    bias=state.applied_bias, bias_limit=self.controller.motion.servo_range)
        if self._latest_adc_group is not None:
            data['raw'] = head_left_right(self._latest_adc_group['raw_volts'], self.channel_layout)
            samples = self._latest_adc.samples
            if (adc['quality'] not in ('stale_sample', 'future_sample')
                    and all(hasattr(s, 'volt_cali_mean_1s') for s in samples)):
                by_channel = {s.channel_id: s.volt_cali_mean_1s for s in samples}
                data['mean_1s'] = head_left_right([by_channel[i] for i in range(3)], self.channel_layout)
                mean_status = []
                if not all(s.mean_1s_ready for s in samples):
                    mean_status.append('warmup')
                if any(s.mean_1s_has_calibration_fallback for s in samples):
                    mean_status.append('calibration_fallback')
                data['mean_1s_status'] = '|'.join(mean_status)
        if self.acoustic is not None:
            data['left_bias_sign'] = self.acoustic.left_bias_sign
            if self.acoustic.pending is not None:
                data['pending_left_bias_sign'] = self.acoustic.pending.left_bias_sign
                data['pending_lr_voltage_offset_v'] = self.acoustic.pending.lr_voltage_offset_v
                data['pending_lr_ratio_offset'] = self.acoustic.pending.lr_ratio_offset
        if cycle is not None:
            data.update(cycle)
            data['cycle_quality'] = '|'.join(flag.name.lower() for flag in Status if flag & cycle['flags'])
        data['direction'] = self._directions(state)[2]
        return format_status(data, self.channel_layout)

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
                                      'threshold=%.4f bias=%.1f lr_voltage_offset_v=%+.6f V '
                                      'left_bias_sign=%+d lr_ratio_offset=%+.4f',
                                      request_id, decision.cycle, values.threshold, values.bias,
                                      values.lr_voltage_offset_v, values.left_bias_sign, values.lr_ratio_offset)
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
        msg.lr_ratio_offset = self.acoustic.settings.lr_ratio_offset
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
            msg.compensated_voltage_difference = (decision.left_mean - decision.right_mean
                                                  + decision.lr_voltage_offset_v)
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
