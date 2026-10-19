"""Per-channel trailing one-second raw-voltage means for operator display."""
from collections import deque
import math


class RawVoltageMean:
    def __init__(self, max_gap_s):
        self.max_gap_s = max_gap_s
        self.reset()

    def reset(self):
        self.rows = deque()
        self.started = None
        self.signature = None

    def update(self, stamp, voltage, gain_valid, gain_voltage):
        if not math.isfinite(stamp) or not math.isfinite(voltage):
            self.reset()
            return float('nan'), False
        known = bool(gain_valid) and math.isfinite(gain_voltage)
        signature = (known, gain_voltage if known else None)
        if self.rows and (stamp <= self.rows[-1][0]
                          or stamp - self.rows[-1][0] > self.max_gap_s
                          or signature != self.signature):
            self.reset()
        if self.started is None:
            self.started = stamp
        self.signature = signature
        self.rows.append((stamp, voltage))
        while self.rows and stamp - self.rows[0][0] > 1.0:
            self.rows.popleft()
        mean = math.fsum(value for _, value in self.rows) / len(self.rows)
        ready = known and len(self.rows) >= 2 and stamp - self.started >= 1.0
        return mean, ready
