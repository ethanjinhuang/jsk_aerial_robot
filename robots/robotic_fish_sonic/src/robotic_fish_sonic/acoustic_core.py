"""No ROS dependencies. No inferred direction without an explicitly supplied model."""
import json
import math
from collections import deque
from enum import IntFlag


class Status(IntFlag):
    WARMUP = 1
    LOW_SIGNAL = 2
    CALIBRATION_FALLBACK = 4
    GAIN_TRANSITION = 8
    STALE = 16
    INVALID_INPUT = 32
    MODEL_UNAVAILABLE = 64
    TIME_RESET = 128
    GAIN_UNKNOWN = 256
    SATURATED = 512
    CALIBRATION_CHANGE = 1024


CLASSES = ['front_right', 'rear_right', 'rear_left', 'front_left']
FEATURES = ['head_ratio', 'left_ratio', 'lr_difference', 'head_sides_difference']


def stamp_seconds(stamp):
    if hasattr(stamp, 'to_sec'):
        return stamp.to_sec()
    sec = getattr(stamp, 'sec', getattr(stamp, 'secs', None))
    ns = getattr(stamp, 'nanosec', getattr(stamp, 'nsecs', None))
    if sec is None or ns is None or sec < 0 or not 0 <= ns < 1000000000:
        raise ValueError('invalid timestamp')
    return sec + ns / 1e9


def adapt(message):
    rows = []
    flags = Status(0)
    calibration_ids = []
    dac_voltages_v = []
    timestamp_semantics = set()
    for sample in sorted(message.samples, key=lambda sample: int(sample.channel if hasattr(sample, 'calibrated_voltage') else sample.channel_id)):
        uses_legacy_schema = hasattr(sample, 'calibrated_voltage')
        channel_id = int(sample.channel if uses_legacy_schema else sample.channel_id)
        sample_time_s = stamp_seconds(sample.header.stamp if uses_legacy_schema else sample.timestamp)
        calibrated_voltage_v = float(sample.calibrated_voltage if uses_legacy_schema else sample.volt_cali)
        if not bool(sample.calibration_applied if uses_legacy_schema else sample.status_cali):
            flags |= Status.CALIBRATION_FALLBACK
        calibration_ids.append(sample.calibration_id if uses_legacy_schema else sample.cali_id)
        dac_voltage_valid = bool(sample.gain_control_voltage_valid if uses_legacy_schema else sample.status_dac_feedback)
        dac_voltage_v = float(sample.gain_control_voltage if uses_legacy_schema else sample.dac_volt)
        if not dac_voltage_valid or not math.isfinite(dac_voltage_v):
            flags |= Status.GAIN_UNKNOWN
            dac_voltage_v = None
        dac_voltages_v.append(dac_voltage_v)
        if abs(int(sample.raw if uses_legacy_schema else sample.adc_code)) >= 32767:
            flags |= Status.SATURATED
        timestamp_semantics.add('conversion_midpoint' if uses_legacy_schema else 'read_completion')
        rows.append((channel_id, sample_time_s, calibrated_voltage_v))
    rows.sort()
    if [row[0] for row in rows] != [0, 1, 2] or len(timestamp_semantics) != 1:
        raise ValueError('expected one channel each and one timestamp schema')
    if not all(math.isfinite(sample_time_s) and sample_time_s >= 0 and math.isfinite(v) and v >= 0 for _, sample_time_s, v in rows):
        raise ValueError('nonfinite/negative sample')
    if max(row[1] for row in rows) - min(row[1] for row in rows) > .05:
        raise ValueError('ADC group spans more than 50 ms')
    if len(set(dac_voltages_v)) > 1:
        flags |= Status.GAIN_TRANSITION
    # Preserve a deterministic reference time without pretending simultaneous sampling.
    return dict(t=max(row[1] for row in rows), volts=[row[2] for row in rows],
                status=flags, signature=(tuple(calibration_ids), tuple(dac_voltages_v), int(flags & Status.CALIBRATION_FALLBACK)),
                semantics=next(iter(timestamp_semantics)))


class Model:
    def __init__(self, path):
        with open(path) as stream:
            self.data = json.load(stream)
        d = self.data
        if d['classes'] != CLASSES or d['features'] != FEATURES or d['frame'] != 'FRD':
            raise ValueError('model class/feature/frame mismatch')
        if len(d['weights']) != 4 or any(len(row) != 4 for row in d['weights']):
            raise ValueError('weights must be 4 by 4')
        if any(len(d[k]) != 4 for k in ('mean', 'scale', 'bias')):
            raise ValueError('invalid model dimensions')
        values = d['mean'] + d['scale'] + d['bias'] + sum(d['weights'], []) + [d['temperature'], d['window_s']]
        if not all(math.isfinite(v) for v in values) or min(d['scale']) <= 0 or d['temperature'] <= 0 or d['window_s'] <= 0:
            raise ValueError('invalid numeric model parameters')
        self.id = str(d['model_id'])

    def predict(self, features):
        model_data = self.data
        normalized_features = [(v - m) / s for v, m, s in zip(features, model_data['mean'], model_data['scale'])]
        logits = [(sum(w * v for w, v in zip(row, normalized_features)) + b) / model_data['temperature']
                  for row, b in zip(model_data['weights'], model_data['bias'])]
        unnormalized_probabilities = [math.exp(v - max(logits)) for v in logits]
        return [v / sum(unnormalized_probabilities) for v in unnormalized_probabilities]


class Estimator:
    def __init__(self, model=None, window_s=.5, gap_s=.2, low_signal_v=.001,
                 future_tolerance_s=.05):
        if not all(math.isfinite(v) and v > 0 for v in (window_s, gap_s, low_signal_v)):
            raise ValueError('window, gap and threshold must be positive')
        if (isinstance(future_tolerance_s, bool) or not math.isfinite(future_tolerance_s)
                or future_tolerance_s < 0):
            raise ValueError('future tolerance must be finite and nonnegative')
        if model and model.data['window_s'] != window_s:
            raise ValueError('model filter window mismatch')
        self.model, self.window_s, self.gap_s, self.low_signal_v = model, window_s, gap_s, low_signal_v
        self.future_tolerance_s = future_tolerance_s
        self.rows = deque()
        self.signature = None
        self.last_input = None
        self.last = None
        self.last_comparison = None
        self.current_status = 0

    def snapshot(self, now, status=None, updated=False, comparison_updated=False):
        if status is not None:
            self.current_status = int(status)
        # Heartbeats must invalidate a window even if no new ADC frame arrives
        # after a ROS clock rewind. Held values keep their acquisition stamps.
        if not math.isfinite(now) or now < 0:
            self.rows.clear()
            self.current_status |= int(Status.INVALID_INPUT | Status.WARMUP)
        elif self.last_input is not None and self.last_input - now > self.future_tolerance_s:
            self.rows.clear()
            self.current_status |= int(Status.TIME_RESET | Status.WARMUP)
        status = Status(self.current_status)
        if not self.model:
            status |= Status.MODEL_UNAVAILABLE
        if self.last_input is None or now - self.last_input > self.gap_s:
            status |= Status.STALE
        result = dict(self.last or dict(probabilities=None, estimate_stamp=None, window_start=None,
                                       window_end=None, valid_sample_count=0, estimate_status=0,
                                       timestamp_semantics='unknown'))
        result.update(status=int(status), updated=updated, has_estimate=self.last is not None,
                      model_id=self.model.id if self.model else 'unavailable')
        comparison = dict(self.last_comparison or {})
        comparison_status = int(status & ~Status.MODEL_UNAVAILABLE)
        comparison.update(has_comparison=self.last_comparison is not None,
                          updated=comparison_updated, status=comparison_status,
                          reliable=self.last_comparison is not None and comparison_status == 0)
        result['comparison'] = comparison
        return result

    def push(self, message, now):
        try:
            if not math.isfinite(now) or now < 0:
                raise ValueError('invalid receipt time')
            sample_group = adapt(message)
            if sample_group['t'] - now > self.future_tolerance_s:
                raise ValueError('sample timestamp is in the future')
        except (ValueError, TypeError, AttributeError, OverflowError):
            self.rows.clear()
            return self.snapshot(now, Status.INVALID_INPUT)
        sample_time_s, flags = sample_group['t'], sample_group['status']
        if self.last_input is not None:
            if sample_time_s <= self.last_input:
                self.rows.clear()
                flags |= Status.TIME_RESET
            elif sample_time_s - self.last_input > self.gap_s:
                self.rows.clear()
        if self.signature is not None and self.signature != sample_group['signature']:
            self.rows.clear()
            if self.signature[1] != sample_group['signature'][1]:
                flags |= Status.GAIN_TRANSITION
            if (self.signature[0], self.signature[2]) != (sample_group['signature'][0], sample_group['signature'][2]):
                flags |= Status.CALIBRATION_CHANGE
        self.signature, self.last_input = sample_group['signature'], sample_time_s
        self.rows.append(sample_group)
        while self.rows and sample_time_s - self.rows[0]['t'] > self.window_s:
            self.rows.popleft()
        if len(self.rows) < 2 or sample_time_s - self.rows[0]['t'] < self.window_s * .8:
            flags |= Status.WARMUP
        for row in self.rows:
            flags |= row['status']
        head_voltage_mean_v, left_voltage_mean_v, right_voltage_mean_v = [
            sum(row['volts'][i] for row in self.rows) / len(self.rows)
            for i in range(3)]
        total_voltage_v = head_voltage_mean_v + left_voltage_mean_v + right_voltage_mean_v
        side_voltage_sum_v = left_voltage_mean_v + right_voltage_mean_v
        side_voltage_mean_v = side_voltage_sum_v / 2
        head_sides_voltage_sum_v = head_voltage_mean_v + side_voltage_mean_v
        if min(total_voltage_v, side_voltage_sum_v, head_sides_voltage_sum_v) < self.low_signal_v:
            flags |= Status.LOW_SIGNAL
        # Assumed sensor mapping: ADC0=head, ADC1=left, ADC2=right.
        # Compare voltage window means, independently of any direction model.
        left_right_sum_difference_ratio = (
            (left_voltage_mean_v - right_voltage_mean_v) / side_voltage_sum_v
            if side_voltage_sum_v > 0 else 0.)
        head_sides_difference_ratio = (
            (head_voltage_mean_v - side_voltage_mean_v) / head_sides_voltage_sum_v
            if head_sides_voltage_sum_v > 0 else 0.)
        self.last_comparison = dict(sample_stamp=sample_time_s,
                                    left_right_normalized_difference=left_right_sum_difference_ratio,
                                    head_sides_normalized_difference=head_sides_difference_ratio)
        if min(total_voltage_v, side_voltage_sum_v, head_sides_voltage_sum_v) <= 0:
            return self.snapshot(now, flags | Status.INVALID_INPUT, comparison_updated=True)
        features = [head_voltage_mean_v / total_voltage_v,
                    left_voltage_mean_v / total_voltage_v,
                    left_right_sum_difference_ratio, head_sides_difference_ratio]
        result = self.snapshot(now, flags, comparison_updated=True)
        result['features'] = features
        if self.model:
            try:
                probabilities = self.model.predict(features)
                if not all(math.isfinite(p) for p in probabilities):
                    raise ValueError('nonfinite prediction')
            except (ValueError, OverflowError, ZeroDivisionError):
                return self.snapshot(now, flags | Status.INVALID_INPUT, comparison_updated=True)
            self.last = dict(probabilities=probabilities, estimate_stamp=sample_time_s,
                             window_start=self.rows[0]['t'], window_end=sample_time_s,
                             valid_sample_count=len(self.rows), estimate_status=int(flags),
                             timestamp_semantics=sample_group['semantics'])
            result = self.snapshot(now, flags, True, comparison_updated=True)
            result['features'] = features
        return result
