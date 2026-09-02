"""Pure fixed/closed-loop gain-control decisions shared by ROS and tests."""

import math

from robotic_fish_io import dac_driver


class GainController:
    """Control a DAC from the maximum of three uncalibrated ADC voltages."""

    MODES = ("off", "fixed", "closed_loop")

    def __init__(
        self,
        mode="off",
        fixed_target_v=0.0,
        target_min_v=2.5,
        target_max_v=3.0,
        step_v=0.01,
        interval_s=0.5,
        consecutive_samples=3,
        dac_min_v=0.0,
        dac_max_v=5.0,
        safety_limit_v=3.50,
        safety_recovery_v=3.30,
        safety_step_v=0.10,
        safety_interval_s=0.10,
        fixed_ramp_step_v=0.05,
        fixed_ramp_interval_s=0.10,
        recovery_settle_s=0.20,
        max_normal_step_v=0.10,
    ):
        self.mode = str(mode).strip().lower()
        if self.mode not in self.MODES:
            raise ValueError("gain mode must be one of {}".format(", ".join(self.MODES)))

        self.dac_min_v = self._dac_voltage(dac_min_v, "dac_min_v")
        self.dac_max_v = self._dac_voltage(dac_max_v, "dac_max_v")
        if not self.dac_min_v < self.dac_max_v:
            raise ValueError("DAC minimum must be below maximum")

        self.fixed_target_v = self._bounded_dac(fixed_target_v, "fixed_target_v")
        self.target_min_v = self._finite(target_min_v, "target_min_v")
        self.target_max_v = self._finite(target_max_v, "target_max_v")
        if not 0.0 <= self.target_min_v < self.target_max_v <= 4.096:
            raise ValueError("ADC target range must satisfy 0 <= min < max <= 4.096 V")

        self.max_normal_step_v = self._positive_dac_step(
            max_normal_step_v, "max_normal_step_v"
        )
        self.step_v = self._positive_dac_step(step_v, "step_v")
        self.fixed_ramp_step_v = self._positive_dac_step(
            fixed_ramp_step_v, "fixed_ramp_step_v"
        )
        if self.step_v > self.max_normal_step_v:
            raise ValueError("AGC step_v exceeds max_normal_step_v")
        if self.fixed_ramp_step_v > self.max_normal_step_v:
            raise ValueError("fixed_ramp_step_v exceeds max_normal_step_v")

        self.safety_step_v = self._positive_dac_step(
            safety_step_v, "safety_step_v"
        )
        self.interval_s = self._interval(interval_s, "interval_s")
        self.fixed_ramp_interval_s = self._interval(
            fixed_ramp_interval_s, "fixed_ramp_interval_s"
        )
        self.safety_interval_s = self._interval(
            safety_interval_s, "safety_interval_s"
        )
        self.recovery_settle_s = self._finite(
            recovery_settle_s, "recovery_settle_s"
        )
        if self.recovery_settle_s < 0.0:
            raise ValueError("recovery_settle_s must not be negative")

        self.safety_limit_v = self._finite(safety_limit_v, "safety_limit_v")
        self.safety_recovery_v = self._finite(
            safety_recovery_v, "safety_recovery_v"
        )
        if not 0.0 <= self.safety_recovery_v < self.safety_limit_v <= 4.096:
            raise ValueError(
                "ADC safety thresholds must satisfy 0 <= recovery < limit <= 4.096 V"
            )
        if self.target_max_v >= self.safety_limit_v:
            raise ValueError("ADC target maximum must be below the safety limit")

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
        self.last_adjustment_s = float("-inf")
        self.last_safety_adjustment_s = float("-inf")
        self.current_max_raw_v = None
        self.last_action = "waiting"
        self.safety_active = False
        self.fixed_limited = False
        self.settle_until_s = float("-inf")

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

    @classmethod
    def _dac_voltage(cls, value, name):
        try:
            return float(dac_driver.normalize_voltage(cls._finite(value, name)))
        except ValueError as exc:
            raise ValueError("{} must be within the DAC hardware range".format(name)) from exc

    @classmethod
    def _positive_dac_step(cls, value, name):
        normalized = cls._dac_voltage(value, name)
        if normalized <= 0.0:
            raise ValueError("{} must be positive".format(name))
        return normalized

    @classmethod
    def _interval(cls, value, name):
        interval = cls._finite(value, name)
        if not 0.02 <= interval <= 3600.0:
            raise ValueError("{} must be between 0.02 and 3600 s".format(name))
        return interval

    def _bounded_dac(self, value, name):
        voltage = self._dac_voltage(value, name)
        if not self.dac_min_v <= voltage <= self.dac_max_v:
            raise ValueError("{} is outside configured DAC limits".format(name))
        return voltage

    def reset_counts(self):
        self.below_count = 0
        self.above_count = 0

    def reset_safety(self):
        """Clear the fixed-mode safety latch; an active overrange remains active."""
        self.fixed_limited = False
        self.reset_counts()
        self.last_action = "safety_active" if self.safety_active else "reset"

    def set_mode(self, mode):
        mode = str(mode).strip().lower()
        if mode not in self.MODES:
            raise ValueError("gain mode must be one of {}".format(", ".join(self.MODES)))
        self.mode = mode
        self.reset_counts()
        self.last_action = mode

    def _target(self, requested):
        bounded = min(self.dac_max_v, max(self.dac_min_v, requested))
        return float(dac_driver.normalize_voltage(bounded))

    def _safety_decision(self, maximum, current, now):
        if maximum >= self.safety_limit_v:
            if not self.safety_active:
                self.last_safety_adjustment_s = float("-inf")
            self.safety_active = True
            self.reset_counts()

        if not self.safety_active:
            return False, None

        if maximum <= self.safety_recovery_v:
            self.safety_active = False
            self.reset_counts()
            if self.mode == "fixed":
                self.fixed_limited = True
                self.last_action = "fixed_limited"
            elif self.mode == "closed_loop":
                self.settle_until_s = now + self.recovery_settle_s
                self.last_action = "settling"
            else:
                self.last_action = "off"
            return True, None

        if current <= self.dac_min_v:
            self.last_action = "adc_overrange_at_dac_min"
            return True, None
        if now - self.last_safety_adjustment_s < self.safety_interval_s:
            self.last_action = "safety_waiting"
            return True, None

        target = self._target(current - self.safety_step_v)
        self.last_safety_adjustment_s = now
        self.last_action = "safety_decreasing"
        if self.mode == "fixed":
            self.fixed_limited = True
        return True, target

    def observe(self, raw_voltages, current_dac_v, now_s):
        """Return the next DAC voltage, or ``None`` when no change is due."""
        if not isinstance(raw_voltages, (list, tuple)) or len(raw_voltages) != 3:
            self.reset_counts()
            raise ValueError("gain control requires ADC0, ADC1, and ADC2")
        values = [
            self._finite(value, "ADC{} voltage".format(channel))
            for channel, value in enumerate(raw_voltages)
        ]
        current = self._finite(current_dac_v, "current DAC voltage")
        now = self._finite(now_s, "monotonic time")
        if not self.dac_min_v <= current <= self.dac_max_v:
            self.reset_counts()
            raise ValueError("Current DAC voltage is outside configured limits")

        maximum = max(values)
        self.current_max_raw_v = maximum
        handled, target = self._safety_decision(maximum, current, now)
        if handled:
            return target

        if self.mode == "off":
            self.reset_counts()
            self.last_action = "off"
            return None

        if self.mode == "fixed":
            self.reset_counts()
            if self.fixed_limited:
                self.last_action = "fixed_limited"
                return None
            difference = self.fixed_target_v - current
            if abs(difference) < 0.005:
                self.last_action = "fixed_holding"
                return None
            if now - self.last_adjustment_s < self.fixed_ramp_interval_s:
                self.last_action = "fixed_waiting"
                return None
            direction = 1.0 if difference > 0.0 else -1.0
            target = self._target(
                current + direction * min(abs(difference), self.fixed_ramp_step_v)
            )
            self.last_adjustment_s = now
            self.last_action = "fixed_ramping_up" if direction > 0 else "fixed_ramping_down"
            return target

        if now < self.settle_until_s:
            self.reset_counts()
            self.last_action = "settling"
            return None
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

        target = self._target(current + direction * self.step_v)
        self.reset_counts()
        if target == current:
            self.last_action = "dac_max" if direction > 0 else "dac_min"
            return None
        self.last_adjustment_s = now
        self.last_action = "increase" if direction > 0 else "decrease"
        return target


class AgcController(GainController):
    """Backward-compatible closed-loop-only controller interface."""

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
        super().__init__(
            mode="closed_loop",
            target_min_v=target_min_v,
            target_max_v=target_max_v,
            step_v=step_v,
            interval_s=interval_s,
            consecutive_samples=consecutive_samples,
            dac_min_v=dac_min_v,
            dac_max_v=dac_max_v,
        )
