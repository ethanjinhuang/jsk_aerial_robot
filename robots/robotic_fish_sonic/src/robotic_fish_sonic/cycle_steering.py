"""At most one decision per complete stable-gain cycle; no ROS or hardware access."""
from collections import deque
from dataclasses import dataclass
import math

from robotic_fish_sonic.acoustic_core import adapt, Status


DIFFERENCE_REFERENCE = 'channel_mean'


def steering_direction(difference, threshold):
    """Inclusive symmetric threshold; equality of channels always means straight.

    Decimal voltage inputs may round just below an exact threshold (e.g. 15%).
    The relative tolerance only absorbs floating-point arithmetic at that edge.
    """
    reached = abs(difference) >= threshold or math.isclose(
        abs(difference), threshold, rel_tol=1e-12, abs_tol=0.)
    return (1 if difference > 0 else -1) if difference != 0 and reached else 0


def number(value, name):
    if isinstance(value, bool):
        raise ValueError(name + ' must be finite')
    try:
        value = float(value)
    except (TypeError, ValueError, OverflowError) as exc:
        raise ValueError(name + ' must be finite') from exc
    if not math.isfinite(value):
        raise ValueError(name + ' must be finite')
    return value


class SteeringFault(ValueError):
    """Input or timing fault requiring explicit motion recovery."""


@dataclass(frozen=True)
class SteeringSettings:
    threshold: float
    bias: float
    lr_voltage_offset_v: float


@dataclass(frozen=True)
class CycleDecision:
    cycle: int
    start: float
    end: float
    head_mean: float
    left_mean: float
    right_mean: float
    difference: float
    bias: float
    sample_count: int
    lr_voltage_offset_v: float
    compensated_left_mean: float
    compensated_right_mean: float
    compensated_difference: float
    raw_means: tuple


class CycleSteering:
    """Compare calibrated voltage means from [cycle start, cycle end).

    Epochs, receipts and decisions use the motion node's monotonic clock.
    Acquisition stamps are mapped from ROS time at reception, with bounded
    clock skew/age. A full elapsed period is required, not 80% window warmup.
    Edge gaps and internal gaps must fit gap_s; samples need not hit endpoints.
    Gain changes, calibration fallback and unknown gain hold the last decision
    until a complete aligned cycle is usable, without changing the gait epoch.
    """
    def __init__(self, frequency, max_bias=1024., threshold=.1, bias=256.,
                 left_bias_sign=1, gap_s=.2, max_age_s=.2,
                 future_tolerance_s=.05, low_signal_v=.001, lr_voltage_offset_v=0.):
        self.max_bias = number(max_bias, 'max_bias')
        self.gap_s = number(gap_s, 'gap_s')
        self.max_age_s = number(max_age_s, 'max_age_s')
        self.future_tolerance_s = number(future_tolerance_s, 'future_tolerance_s')
        self.low_signal_v = number(low_signal_v, 'low_signal_v')
        if min(self.max_bias, self.gap_s, self.max_age_s, self.low_signal_v) <= 0 or self.future_tolerance_s < 0:
            raise ValueError('limits must be positive; future tolerance nonnegative')
        if isinstance(left_bias_sign, bool) or left_bias_sign not in (-1, 1):
            raise ValueError('left_bias_sign must be -1 or 1')
        self.left_bias_sign = int(left_bias_sign)
        self.settings = self._settings(threshold, bias, lr_voltage_offset_v)
        self.pending = None
        self._set_frequency(frequency)
        self.rows = deque()
        self.epoch = None
        self.epoch_clock_offset = None
        self.cycle = 0
        self.last_decision = None
        self.latest_time = self.latest_stamp = self.last_sensor_stamp = None
        self.latest_error = 'no_data'
        self.fault = ''
        self.signature = self.clock_offset = None
        self.waiting_status = self._unusable_reason = ''
        self._stable_since = None

    @property
    def gain_warmup(self):
        return self.waiting_status == 'gain_warmup'

    def _settings(self, threshold, bias, lr_voltage_offset_v):
        threshold, bias = number(threshold, 'threshold'), number(bias, 'bias')
        lr_voltage_offset_v = number(lr_voltage_offset_v, 'lr_voltage_offset_v')
        if not 0 <= threshold <= 2 or not 0 <= bias <= self.max_bias:
            raise ValueError('threshold must be in [0,2] and bias in [0,max_bias]')
        return SteeringSettings(threshold, bias, lr_voltage_offset_v)

    def configure(self, threshold, bias, lr_voltage_offset_v):
        """Validate the entire triplet now; apply it at the next valid decision."""
        self.pending = self._settings(threshold, bias, lr_voltage_offset_v)

    def _set_frequency(self, frequency):
        frequency = number(frequency, 'frequency')
        if frequency <= 0 or not math.isfinite(1. / frequency):
            raise ValueError('frequency must have a positive finite period')
        self.frequency, self.period = frequency, 1. / frequency

    def _reject(self, reason):
        self.latest_error = reason
        self.rows.clear()
        if self.epoch is not None:
            self.fault = reason
        return False

    def push(self, message, received, ros_now):
        """Check timing on every group; only usable groups enter a decision."""
        try:
            received, ros_now = number(received, 'receipt'), number(ros_now, 'ROS time')
            g = adapt(message)
            # These quality states pause decisions while fresh timing still
            # distinguishes incoming unusable data from a disconnected ADC.
            waiting = g['status'] & (Status.GAIN_TRANSITION |
                                     Status.CALIBRATION_FALLBACK | Status.GAIN_UNKNOWN)
            invalid = g['status'] & ~waiting
            if invalid:
                return self._reject('|'.join(flag.name.lower() for flag in Status if flag & invalid))
            age = ros_now - g['t']
            if age < -self.future_tolerance_s:
                return self._reject('future_sample')
            if age > self.max_age_s:
                return self._reject('stale_sample')
            if self.last_sensor_stamp is not None and g['t'] <= self.last_sensor_stamp:
                self.last_sensor_stamp = g['t']
                return self._reject('sample_time_reset')
            offset = received - ros_now
            if self.clock_offset is not None and abs(offset - self.clock_offset) > self.future_tolerance_s:
                self.clock_offset = offset
                self.last_sensor_stamp = g['t']
                return self._reject('clock_changed')
            acquisition = min(received, g['t'] + offset)
            if self.latest_time is not None and acquisition <= self.latest_time:
                self.latest_time = None
                return self._reject('receipt_time_reset')
            if not waiting:
                total = g['volts'][1] + g['volts'][2]
                if not math.isfinite(total):
                    return self._reject('invalid_signal_sum')
                if total < self.low_signal_v:
                    return self._reject('low_signal')
        except (ValueError, TypeError, AttributeError, OverflowError) as exc:
            return self._reject('invalid_input: ' + str(exc))
        # A fresh frame must not hide a missed health deadline between ticks.
        # Keep the recovered frame for explicit rearming, but latch active faults.
        continuity_error = ''
        if self.latest_time is not None:
            if received - self.latest_time > self.max_age_s + 1e-9:
                continuity_error = 'adc_timeout'
            elif acquisition - self.latest_time > self.gap_s + 1e-9:
                continuity_error = 'adc_gap'
        if continuity_error:
            self.rows.clear()
            if self.epoch is not None:
                self.fault = continuity_error
        # Compare usable signatures only: fallback IDs and unknown DAC values
        # must not turn normal quality recovery into a calibration-change fault.
        calibration_changed = (not waiting and self.signature is not None and
                               g['signature'][0] != self.signature[0])
        gain_changed = (not waiting and self.signature is not None and
                        g['signature'][1] != self.signature[1])
        recovering = self._unusable_reason
        if calibration_changed:
            self.rows.clear()
            if self.epoch is not None:
                self.fault = 'gain_or_calibration_changed'
        if waiting or gain_changed or recovering:
            self.rows.clear()
            if self.epoch is not None:
                if waiting:
                    self.waiting_status = ('gain_warmup' if waiting == Status.GAIN_TRANSITION else
                                           '|'.join(flag.name.lower() for flag in Status if flag & waiting))
                else:
                    self.waiting_status = ('data_warmup' if recovering and recovering != 'gain_transition'
                                           else 'gain_warmup')
                self._stable_since = None if waiting else acquisition
        self._unusable_reason = '|'.join(flag.name.lower() for flag in Status if flag & waiting)
        if not waiting:
            self.signature = g['signature']
        self.clock_offset = offset
        self.last_sensor_stamp = self.latest_stamp = g['t']
        self.latest_time = acquisition
        self.latest_error = ''
        if not waiting:
            # Raw voltages are diagnostics only. Missing values in older input
            # schemas must remain unavailable, without changing steering health.
            raw = {}
            for sample in message.samples:
                channel = int(sample.channel if hasattr(sample, 'calibrated_voltage') else sample.channel_id)
                try:
                    raw[channel] = number(getattr(sample, 'volt_raw', None), 'raw voltage')
                except ValueError:
                    raw[channel] = float('nan')
            self.rows.append((acquisition, *g['volts'], *(raw[i] for i in range(3))))
        while self.rows and acquisition - self.rows[0][0] > self.period + 2. * self.gap_s:
            self.rows.popleft()
        if len(self.rows) > 10000:
            return self._reject('sample_buffer_overflow')
        return True

    def _health(self, now):
        if self.latest_error:
            raise SteeringFault(self.latest_error)
        if self.latest_time is None or now - self.latest_time > self.max_age_s:
            raise SteeringFault('adc_timeout')
        if now < self.latest_time:
            raise SteeringFault('clock_rewind')

    def begin(self, epoch, frequency, now):
        now = number(now, 'start time')
        self._health(now)
        self._set_frequency(frequency)
        self.epoch = number(epoch, 'epoch')
        if self.epoch < 0 or self.epoch > now:
            raise SteeringFault('invalid_epoch')
        self.epoch_clock_offset = self.clock_offset
        self.cycle, self.last_decision, self.fault = 0, None, ''
        self.waiting_status = ('gain_warmup' if self._unusable_reason == 'gain_transition'
                               else self._unusable_reason)
        self._stable_since = None

    def deactivate(self):
        self.epoch = None
        self.fault = ''
        self.waiting_status, self._stable_since = '', None

    def evaluate(self, now):
        """Detect the next positive-going crossing and decide exactly once."""
        now = number(now, 'decision time')
        if self.epoch is None:
            return None
        if self.fault:
            raise SteeringFault(self.fault)
        self._health(now)
        elapsed = now - self.epoch
        if elapsed < 0:
            raise SteeringFault('clock_rewind')
        cycles = elapsed / self.period
        if not math.isfinite(cycles):
            raise SteeringFault('phase_overflow')
        cycle = math.floor(cycles + 1e-9)
        if cycle <= self.cycle:
            return None
        if cycle != self.cycle + 1:
            raise SteeringFault('missed_cycle')
        start, end = self.epoch + (cycle - 1) * self.period, self.epoch + cycle * self.period
        if (self.waiting_status and self.waiting_status != 'voltage_compensation_out_of_range'
                and (self._stable_since is None or start < self._stable_since - 1e-9)):
            # Advance the cycle counter without resetting the oscillator epoch.
            # Keep the held decision until an entire aligned cycle is usable.
            self.cycle = cycle
            return None
        rows = [row for row in self.rows if start <= row[0] < end]
        times = [start] + [row[0] for row in rows] + [end]
        if len(rows) < 2 or max(b - a for a, b in zip(times, times[1:])) > self.gap_s + 1e-9:
            raise SteeringFault('incomplete_cycle')
        head, left, right = [math.fsum(row[i] / len(rows) for row in rows) for i in (1, 2, 3)]
        raw_means = tuple(math.fsum(row[i] / len(rows) for row in rows) for i in (4, 5, 6))
        if not math.isfinite(left + right):
            raise SteeringFault('invalid_signal_sum')
        if left + right < self.low_signal_v:
            raise SteeringFault('low_signal')
        # Offset compensates L-R once; it does not alter the measured means.
        mean = (left + right) / 2.
        difference = (left - right) / mean
        candidate = self.settings if self.pending is None else self.pending
        # Retain symmetric diagnostic channels and their nonnegative-envelope
        # check. Half on each side changes their difference by exactly offset.
        compensated_left = left + candidate.lr_voltage_offset_v / 2.
        compensated_right = right - candidate.lr_voltage_offset_v / 2.
        if (not all(math.isfinite(v) and v >= 0 for v in (compensated_left, compensated_right))):
            # A voltage correction may not invent a negative envelope. Retry at
            # the next full cycle, keeping the last bias, gait epoch and pending
            # triplet. Fresh ADC / Joy health checks continue during this hold.
            self.cycle = cycle
            self.waiting_status = 'voltage_compensation_out_of_range'
            return None
        compensated = (left - right + candidate.lr_voltage_offset_v) / mean
        self.settings, self.pending = candidate, None
        direction = steering_direction(compensated, self.settings.threshold)
        self.cycle = cycle
        self.last_decision = CycleDecision(cycle, start, end, head, left, right, difference,
                                           direction * self.left_bias_sign * self.settings.bias, len(rows),
                                           self.settings.lr_voltage_offset_v,
                                           compensated_left, compensated_right, compensated, raw_means)
        self.waiting_status, self._stable_since = '', None
        return self.last_decision
