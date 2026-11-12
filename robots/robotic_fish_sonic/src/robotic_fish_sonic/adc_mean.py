"""Per-channel trailing one-second raw-voltage means for operator display."""
from collections import deque
import math


class RawVoltageMean:
    def __init__(self, max_gap_s):
        self.max_gap_s = max_gap_s
        self.reset()

    def reset(self):
        self._samples = deque()
        self._window_start_s = None
        self._dac_signature = None

    def update(self, stamp, voltage, gain_valid, gain_voltage):
        if not math.isfinite(stamp) or not math.isfinite(voltage):
            self.reset()
            return float('nan'), False
        dac_voltage_known = bool(gain_valid) and math.isfinite(gain_voltage)
        dac_signature = (dac_voltage_known, gain_voltage if dac_voltage_known else None)
        if self._samples and (stamp <= self._samples[-1][0]
                          or stamp - self._samples[-1][0] > self.max_gap_s
                          or dac_signature != self._dac_signature):
            self.reset()
        if self._window_start_s is None:
            self._window_start_s = stamp
        self._dac_signature = dac_signature
        self._samples.append((stamp, voltage))
        while self._samples and stamp - self._samples[0][0] > 1.0:
            self._samples.popleft()
        raw_voltage_mean_v = math.fsum(value for _, value in self._samples) / len(self._samples)
        mean_ready = dac_voltage_known and len(self._samples) >= 2 and stamp - self._window_start_s >= 1.0
        return raw_voltage_mean_v, mean_ready
