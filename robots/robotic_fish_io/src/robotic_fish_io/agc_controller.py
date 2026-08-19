"""Pure closed-loop AGC decision logic shared by the ROS node and tests."""

import math

from robotic_fish_io import dac_driver


class AgcController:
    """Adjust DAC voltage from the maximum of three uncalibrated ADC values."""

    def __init__(
        self,
        target_min_v,
        target_max_v,
        step_v,
        interval_s,
        consecutive_samples,
        dac_min_v=0.0,
        dac_max_v=5.0,
    ):
        self.target_min_v = self._finite(target_min_v, "target_min_v")
        self.target_max_v = self._finite(target_max_v, "target_max_v")
        self.step_v = float(dac_driver.normalize_voltage(step_v))
        self.interval_s = self._finite(interval_s, "interval_s")
        self.dac_min_v = float(dac_driver.normalize_voltage(dac_min_v))
        self.dac_max_v = float(dac_driver.normalize_voltage(dac_max_v))
        if not 0.0 <= self.target_min_v < self.target_max_v <= 4.096:
            raise ValueError(
                "ADC target range must satisfy 0 <= min < max <= 4.096 V"
            )
        if self.step_v <= 0.0:
            raise ValueError("AGC step_v must be positive")
        if not 0.02 <= self.interval_s <= 3600.0:
            raise ValueError("AGC interval_s must be between 0.02 and 3600 s")
        if not self.dac_min_v < self.dac_max_v:
            raise ValueError("AGC DAC minimum must be below maximum")
        if isinstance(consecutive_samples, bool):
            raise ValueError("consecutive_samples must be an integer")
        try:
            consecutive = int(consecutive_samples)
        except (TypeError, ValueError, OverflowError) as exc:
            raise ValueError("consecutive_samples must be an integer") from exc
        if consecutive != float(consecutive_samples) or not 1 <= consecutive <= 1000:
            raise ValueError("consecutive_samples must be between 1 and 1000")
        self.consecutive_samples = consecutive

        self.below_count = 0
        self.above_count = 0
        self.last_adjustment_s = 0.0
        self.current_max_raw_v = None
        self.last_action = "waiting"

    @staticmethod
    def _finite(value, name):
        if isinstance(value, bool):
            raise ValueError("{} must be finite".format(name))
        try:
            value = float(value)
        except (TypeError, ValueError, OverflowError) as exc:
            raise ValueError("{} must be finite".format(name)) from exc
        if not math.isfinite(value):
            raise ValueError("{} must be finite".format(name))
        return value

    def reset_counts(self):
        self.below_count = 0
        self.above_count = 0

    def observe(self, raw_voltages, current_dac_v, now_s):
        """Return the next DAC voltage, or ``None`` when no change is due."""
        if not isinstance(raw_voltages, (list, tuple)) or len(raw_voltages) != 3:
            self.reset_counts()
            raise ValueError("AGC requires ADC0, ADC1, and ADC2")
        values = [
            self._finite(value, "ADC{} voltage".format(channel))
            for channel, value in enumerate(raw_voltages)
        ]
        current = self._finite(current_dac_v, "current DAC voltage")
        now = self._finite(now_s, "monotonic time")
        if not self.dac_min_v <= current <= self.dac_max_v:
            self.reset_counts()
            raise ValueError("Current DAC voltage is outside the AGC limits")

        maximum = max(values)
        self.current_max_raw_v = maximum
        if maximum < self.target_min_v:
            self.below_count += 1
            self.above_count = 0
            direction = 1
            self.last_action = "waiting_low"
        elif maximum > self.target_max_v:
            self.above_count += 1
            self.below_count = 0
            direction = -1
            self.last_action = "waiting_high"
        else:
            self.reset_counts()
            self.last_action = "in_range"
            return None

        count = self.below_count if direction > 0 else self.above_count
        if count < self.consecutive_samples:
            return None
        if now - self.last_adjustment_s < self.interval_s:
            return None

        requested = current + direction * self.step_v
        bounded = min(self.dac_max_v, max(self.dac_min_v, requested))
        target = float(dac_driver.normalize_voltage(bounded))
        self.reset_counts()
        if target == current:
            self.last_action = "dac_max" if direction > 0 else "dac_min"
            return None

        self.last_action = "increase" if direction > 0 else "decrease"
        self.last_adjustment_s = now
        return target
