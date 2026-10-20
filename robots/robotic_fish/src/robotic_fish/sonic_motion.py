"""Single-servo gait and guarded joystick control, without ROS or hardware access.

All times supplied to SonicController are seconds from one monotonic clock.
Positions, amplitudes and biases are servo command units, not radians/degrees.
"""
from dataclasses import dataclass
import math


def _number(value, name):
    if isinstance(value, bool):
        raise ValueError(name + ' must be a finite number')
    try:
        value = float(value)
    except (TypeError, ValueError, OverflowError) as exc:
        raise ValueError(name + ' must be a finite number') from exc
    if not math.isfinite(value):
        raise ValueError(name + ' must be a finite number')
    return value


def _index(value, name):
    if isinstance(value, bool) or not isinstance(value, int) or value < 0:
        raise ValueError(name + ' must be a nonnegative integer')
    return value


class SonicMotion:
    """The original centered sinusoid, with a separate bias input and envelope."""
    def __init__(self, center=2048., servo_range=1024.):
        self.center = _number(center, 'servo_center')
        self.servo_range = _number(servo_range, 'servo_range')
        self.lower = self.center - self.servo_range
        self.upper = self.center + self.servo_range
        if self.servo_range <= 0 or self.lower < -32768 or self.upper > 32767:
            raise ValueError('servo envelope must be positive and fit an int16 command')

    def sample(self, elapsed_s, bias, amplitude, frequency):
        """Return (position, effective amplitude); clip bias before fitting amplitude."""
        elapsed_s = _number(elapsed_s, 'elapsed_s')
        bias = _number(bias, 'bias')
        amplitude = _number(amplitude, 'amplitude')
        frequency = _number(frequency, 'frequency')
        if elapsed_s < 0 or amplitude < 0 or frequency <= 0:
            raise ValueError('elapsed/amplitude must be nonnegative and frequency positive')
        phase = 2. * math.pi * frequency * elapsed_s
        if not math.isfinite(phase):
            raise ValueError('oscillator phase overflow')
        bias = max(-self.servo_range, min(self.servo_range, bias))
        center = self.center + bias
        effective = min(amplitude, self.upper - center, center - self.lower)
        target = center + effective * math.sin(phase)
        return max(self.lower, min(self.upper, target)), effective


@dataclass(frozen=True)
class MotionState:
    mode: str
    target: float
    requested_bias: float
    applied_bias: float
    requested_amplitude: float
    effective_amplitude: float
    frequency: float
    reason: str


class SonicController:
    """Manual/auto/stopped state machine and rate-limited bias.

    Startup is manual, but produces no command before the first Joy input.
    Stop/fault recovery requires a neutral, released frame followed by a new
    manual/auto button press while neutral. Repeated messages cannot clear a stop.
    """
    def __init__(self, center=2048., servo_range=1024., frequency=2.,
                 default_amplitude=512., amplitude=None, amplitude_rate=64.,
                 bias_rate_limit=512., joy_timeout_s=.5, joy_dead_zone=.1,
                 steering_axis=2, amplitude_decrease_axis=3, amplitude_increase_axis=4,
                 stop_button=1, auto_button=3, manual_button=0, amplitude_buttons=(6, 7)):
        self.motion = SonicMotion(center, servo_range)
        self.frequency = _number(frequency, 'auto_frequency')
        self.default_amplitude = _number(default_amplitude, 'auto_default_amplitude')
        self.amplitude = _number(self.default_amplitude if amplitude is None else amplitude, 'auto_amplitude')
        self.amplitude_rate = _number(amplitude_rate, 'auto_amplitude_rate')
        self.bias_rate_limit = _number(bias_rate_limit, 'bias_rate_limit')
        self.joy_timeout_s = _number(joy_timeout_s, 'joy_timeout_s')
        self.dead_zone = _number(joy_dead_zone, 'joy_dead_zone')
        if self.frequency <= 0 or not math.isfinite(2. * math.pi * self.frequency):
            raise ValueError('auto_frequency must be positive with a finite phase rate')
        if (not 0 <= self.default_amplitude <= self.motion.servo_range or
                not 0 <= self.amplitude <= self.motion.servo_range):
            raise ValueError('amplitudes must be within [0, servo_range]')
        if self.amplitude_rate < 0 or self.bias_rate_limit <= 0 or self.joy_timeout_s <= 0:
            raise ValueError('amplitude rate must be nonnegative; bias rate and timeout positive')
        if not 0 <= self.dead_zone < 1:
            raise ValueError('joy_dead_zone must be within [0, 1)')
        self.steering_axis = _index(steering_axis, 'steering_axis')
        self.decrease_axis = _index(amplitude_decrease_axis, 'amplitude_decrease_axis')
        self.increase_axis = _index(amplitude_increase_axis, 'amplitude_increase_axis')
        self.stop_button = _index(stop_button, 'stop_button')
        self.auto_button = _index(auto_button, 'auto_button')
        self.manual_button = _index(manual_button, 'manual_button')
        if not isinstance(amplitude_buttons, (list, tuple)) or not amplitude_buttons:
            raise ValueError('amplitude_buttons must be a nonempty list of indices')
        self.amplitude_buttons = tuple(_index(x, 'amplitude_buttons') for x in amplitude_buttons)
        mode_buttons = (self.stop_button, self.auto_button, self.manual_button)
        if len(set(mode_buttons)) != 3 or len(set(self.amplitude_buttons)) != len(self.amplitude_buttons):
            raise ValueError('mode and amplitude buttons must have distinct indices')
        if set(mode_buttons) & set(self.amplitude_buttons):
            raise ValueError('amplitude buttons must not overlap mode buttons')
        if len({self.steering_axis, self.decrease_axis, self.increase_axis}) != 3:
            raise ValueError('steering and amplitude axes must have distinct indices')
        self.mode, self.reason = 'manual', 'startup'
        self.requested_bias = self.applied_bias = 0.
        self.input_error = ''
        self._last_joy = self._last_event = self._last_update = self._auto_start = None
        self._auto_prev = self._manual_prev = self._rearm_ready = False
        self._has_output = False

    def stop(self, reason='stop', now=None):
        self.mode, self.reason = 'stopped', reason
        self.requested_bias = self.applied_bias = 0.
        self._auto_start = None
        self._auto_prev = self._manual_prev = True
        self._rearm_ready = False
        self._has_output = True
        if now is not None:
            self._last_update = now
        return self._snapshot()

    def _time(self, value):
        try:
            now = _number(value, 'monotonic time')
            if now < 0:
                raise ValueError('negative monotonic time')
        except ValueError:
            self.stop('invalid_clock')
            return None
        if self._last_event is not None and now < self._last_event:
            self._last_joy = None
            self._last_event = now
            self.stop('clock_rewind', now)
            return None
        self._last_event = now
        return now

    def _advance(self, now):
        dt = 0. if self._last_update is None else now - self._last_update
        self._last_update = now
        if self.mode != 'stopped':
            difference = self.requested_bias - self.applied_bias
            step = self.bias_rate_limit * dt
            self.applied_bias += max(-step, min(step, difference))

    def _expire(self, now):
        if self.mode != 'stopped' and self._last_joy is not None and now - self._last_joy > self.joy_timeout_s:
            self.stop('joy_timeout', now)

    def _snapshot(self, now=None):
        if self.mode == 'stopped':
            return MotionState(self.mode, self.motion.center, 0., 0., self.amplitude,
                               0., self.frequency, self.reason)
        active = self.mode == 'auto'
        elapsed = now - self._auto_start if active else 0.
        try:
            target, effective = self.motion.sample(elapsed, self.applied_bias,
                                                   self.amplitude if active else 0., self.frequency)
        except ValueError as exc:
            self.input_error = str(exc)
            return self.stop('motion_error', now)
        return MotionState(self.mode, target, self.requested_bias, self.applied_bias,
                           self.amplitude, effective, self.frequency, self.reason)

    def _validate_joy(self, axes, buttons):
        indices = (self.steering_axis, self.decrease_axis, self.increase_axis)
        button_indices = (self.stop_button, self.auto_button, self.manual_button, *self.amplitude_buttons)
        if len(axes) <= max(indices) or len(buttons) <= max(button_indices):
            raise ValueError('Joy message is shorter than the configured mapping')
        values = [_number(axes[i], 'mapped Joy axis') for i in indices]
        if any(abs(x) > 1 for x in values):
            raise ValueError('mapped Joy axes must be within [-1, 1]')
        if any(buttons[i] not in (0, 1) for i in button_indices):
            raise ValueError('mapped Joy buttons must be 0 or 1')
        return values

    def handle_joy(self, axes, buttons, now):
        now = self._time(now)
        if now is None:
            return self._snapshot()
        self._expire(now)
        self._advance(now)
        # Stop has priority even when the rest of a message is malformed.
        try:
            stop_pressed = len(buttons) > self.stop_button and buttons[self.stop_button] == 1
        except TypeError:
            stop_pressed = False
        if stop_pressed:
            return self.stop('stop_button', now)
        try:
            steering, decrease, increase = self._validate_joy(axes, buttons)
        except (ValueError, TypeError, IndexError) as exc:
            self.input_error = str(exc)
            return self.stop('invalid_joy', now)
        self.input_error = ''
        dt = 0. if self._last_joy is None else max(0., min(self.joy_timeout_s, now - self._last_joy))
        self._last_joy = now
        auto, manual = bool(buttons[self.auto_button]), bool(buttons[self.manual_button])
        auto_edge, manual_edge = auto and not self._auto_prev, manual and not self._manual_prev
        self._auto_prev, self._manual_prev = auto, manual
        adjusting = any(buttons[i] for i in self.amplitude_buttons)
        neutral = steering == 0 or abs(steering) < self.dead_zone
        if auto and manual:
            return self.stop('mode_conflict', now)
        if self.mode == 'stopped':
            if neutral and not adjusting and not auto and not manual:
                self._rearm_ready = True
            if not (self._rearm_ready and neutral and not adjusting and (auto_edge or manual_edge)):
                return self._snapshot()
            self._rearm_ready = False
        if auto_edge:
            self.mode, self.reason, self._auto_start = 'auto', 'auto_button', now
            if self.amplitude <= 0:
                self.amplitude = self.default_amplitude
        elif manual_edge:
            self.mode, self.reason, self._auto_start = 'manual', 'manual_button', None
        self.requested_bias = 0. if neutral else -steering * self.motion.servo_range
        if adjusting:
            delta = .5 * (1. - increase) - .5 * (1. - decrease)
            if abs(delta) < self.dead_zone:
                delta = 0.
            self.amplitude = max(0., min(self.motion.servo_range,
                                        self.amplitude + delta * self.amplitude_rate * dt))
        self._has_output = True
        return self._snapshot(now)

    def tick(self, now):
        now = self._time(now)
        if now is None:
            return self._snapshot()
        self._expire(now)
        self._advance(now)
        return self._snapshot(now) if self._has_output else None
