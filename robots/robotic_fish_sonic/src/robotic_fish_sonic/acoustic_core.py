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
    ids = []
    gains = []
    semantics = set()
    for s in sorted(message.samples, key=lambda sample: int(sample.channel if hasattr(sample, 'calibrated_voltage') else sample.channel_id)):
        old = hasattr(s, 'calibrated_voltage')
        channel = int(s.channel if old else s.channel_id)
        t = stamp_seconds(s.header.stamp if old else s.timestamp)
        value = float(s.calibrated_voltage if old else s.volt_cali)
        if not bool(s.calibration_applied if old else s.status_cali):
            flags |= Status.CALIBRATION_FALLBACK
        ids.append(s.calibration_id if old else s.cali_id)
        known = bool(s.gain_control_voltage_valid if old else s.status_dac_feedback)
        gain = float(s.gain_control_voltage if old else s.dac_volt)
        if not known or not math.isfinite(gain):
            flags |= Status.GAIN_UNKNOWN
            gain = None
        gains.append(gain)
        if abs(int(s.raw if old else s.adc_code)) >= 32767:
            flags |= Status.SATURATED
        semantics.add('conversion_midpoint' if old else 'read_completion')
        rows.append((channel, t, value))
    rows.sort()
    if [r[0] for r in rows] != [0, 1, 2] or len(semantics) != 1:
        raise ValueError('expected one channel each and one timestamp schema')
    if not all(math.isfinite(t) and t >= 0 and math.isfinite(v) and v >= 0 for _, t, v in rows):
        raise ValueError('nonfinite/negative sample')
    if max(r[1] for r in rows) - min(r[1] for r in rows) > .05:
        raise ValueError('ADC group spans more than 50 ms')
    if len(set(gains)) > 1:
        flags |= Status.GAIN_TRANSITION
    # Preserve a deterministic reference time without pretending simultaneous sampling.
    return dict(t=max(r[1] for r in rows), volts=[r[2] for r in rows],
                status=flags, signature=(tuple(ids), tuple(gains), int(flags & Status.CALIBRATION_FALLBACK)),
                semantics=next(iter(semantics)))


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
        d = self.data
        x = [(v - m) / s for v, m, s in zip(features, d['mean'], d['scale'])]
        logits = [(sum(w * v for w, v in zip(row, x)) + b) / d['temperature']
                  for row, b in zip(d['weights'], d['bias'])]
        exps = [math.exp(v - max(logits)) for v in logits]
        return [v / sum(exps) for v in exps]


class Estimator:
    def __init__(self, model=None, window_s=.5, gap_s=.2, low_signal_v=.001):
        if not all(math.isfinite(v) and v > 0 for v in (window_s, gap_s, low_signal_v)):
            raise ValueError('window, gap and threshold must be positive')
        if model and model.data['window_s'] != window_s:
            raise ValueError('model filter window mismatch')
        self.model, self.window_s, self.gap_s, self.low_signal_v = model, window_s, gap_s, low_signal_v
        self.rows = deque()
        self.signature = None
        self.last_input = None
        self.last = None
        self.last_comparison = None
        self.current_status = 0

    def snapshot(self, now, status=None, updated=False, comparison_updated=False):
        if status is not None:
            self.current_status = int(status)
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
            g = adapt(message)
        except (ValueError, TypeError, AttributeError, OverflowError):
            self.rows.clear()
            return self.snapshot(now, Status.INVALID_INPUT)
        t, flags = g['t'], g['status']
        if self.last_input is not None:
            if t <= self.last_input:
                self.rows.clear()
                flags |= Status.TIME_RESET
            elif t - self.last_input > self.gap_s:
                self.rows.clear()
        if self.signature is not None and self.signature != g['signature']:
            self.rows.clear()
            if self.signature[1] != g['signature'][1]:
                flags |= Status.GAIN_TRANSITION
            if (self.signature[0], self.signature[2]) != (g['signature'][0], g['signature'][2]):
                flags |= Status.CALIBRATION_CHANGE
        self.signature, self.last_input = g['signature'], t
        self.rows.append(g)
        while self.rows and t - self.rows[0]['t'] > self.window_s:
            self.rows.popleft()
        if len(self.rows) < 2 or t - self.rows[0]['t'] < self.window_s * .8:
            flags |= Status.WARMUP
        for row in self.rows:
            flags |= row['status']
        h, l, r = [sum(row['volts'][i] for row in self.rows) / len(self.rows) for i in range(3)]
        s = h + l + r
        if min(s, l + r, h + (l + r) / 2) < self.low_signal_v:
            flags |= Status.LOW_SIGNAL
        # Assumed sensor mapping: ADC0=head, ADC1=left, ADC2=right.
        # Compare voltage window means, independently of any direction model.
        sides = (l + r) / 2
        lr_difference = (l - r) / (l + r) if l + r > 0 else 0.
        head_sides_difference = (h - sides) / (h + sides) if h + sides > 0 else 0.
        self.last_comparison = dict(sample_stamp=t,
                                    left_right_normalized_difference=lr_difference,
                                    head_sides_normalized_difference=head_sides_difference)
        if min(s, l + r, h + sides) <= 0:
            return self.snapshot(now, flags | Status.INVALID_INPUT, comparison_updated=True)
        features = [h / s, l / s, lr_difference, head_sides_difference]
        result = self.snapshot(now, flags, comparison_updated=True)
        result['features'] = features
        if self.model:
            try:
                probabilities = self.model.predict(features)
                if not all(math.isfinite(p) for p in probabilities):
                    raise ValueError('nonfinite prediction')
            except (ValueError, OverflowError, ZeroDivisionError):
                return self.snapshot(now, flags | Status.INVALID_INPUT, comparison_updated=True)
            self.last = dict(probabilities=probabilities, estimate_stamp=t,
                             window_start=self.rows[0]['t'], window_end=t,
                             valid_sample_count=len(self.rows), estimate_status=int(flags),
                             timestamp_semantics=g['semantics'])
            result = self.snapshot(now, flags, True, comparison_updated=True)
            result['features'] = features
        return result
