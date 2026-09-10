"""Pure fixed/closed-loop gain-control decisions shared by ROS and tests."""

import math

from robotic_fish_io import dac_driver
from robotic_fish_io.adc_limits import MAX_VOLTAGE_V


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
        safety_limit_v=4.00,
        safety_recovery_v=3.30,
        safety_step_v=0.10,
        safety_interval_s=0.10,
        fixed_ramp_step_v=0.05,
        fixed_ramp_interval_s=0.10,
        recovery_settle_s=0.20,
        max_normal_step_v=0.10,
        fixed_auto_recovery=False,
        fixed_recovery_wait_s=2.0,
        fixed_recovery_step_v=0.01,
        fixed_recovery_interval_s=0.5,
        fixed_recovery_max_adc_v=1.0,
        recovery_sample_timeout_s=0.2,
        window_control=False,
        window_s=2.0,
        window_mean_min_v=2.0,
        window_peak_upper_v=3.0,
        window_high_fraction=None,
        window_cooldown_s=5.0,
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
        if not 0.0 <= self.safety_recovery_v < self.safety_limit_v <= MAX_VOLTAGE_V:
            raise ValueError(
                "ADC safety thresholds must satisfy 0 <= recovery < limit <= 4.095875 V"
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

        if not isinstance(fixed_auto_recovery, bool):
            raise ValueError("fixed_auto_recovery must be boolean")
        self.fixed_auto_recovery = fixed_auto_recovery
        self.fixed_recovery_wait_s = self._interval(fixed_recovery_wait_s, "fixed_recovery_wait_s")
        self.fixed_recovery_interval_s = self._interval(fixed_recovery_interval_s, "fixed_recovery_interval_s")
        self.recovery_sample_timeout_s = self._interval(recovery_sample_timeout_s, "recovery_sample_timeout_s")
        self.fixed_recovery_step_v = self._positive_dac_step(fixed_recovery_step_v, "fixed_recovery_step_v")
        self.fixed_recovery_max_adc_v = self._finite(fixed_recovery_max_adc_v, "fixed_recovery_max_adc_v")
        if not 0 < self.fixed_recovery_max_adc_v < self.safety_recovery_v:
            raise ValueError("fixed recovery ADC limit must be positive and below safety recovery")
        if self.fixed_recovery_step_v > self.max_normal_step_v:
            raise ValueError("fixed recovery step exceeds max_normal_step_v")
        if self.fixed_recovery_step_v / self.fixed_recovery_interval_s >= self.safety_step_v / self.safety_interval_s:
            raise ValueError("fixed recovery must be slower than safety reduction")
        self.recovery_since_s = None
        self.recovery_last_sample_s = None
        self.recovery_last_step_s = float("-inf")

        self.below_count = 0
        self.above_count = 0
        self.last_adjustment_s = float("-inf")
        self.last_safety_adjustment_s = float("-inf")
        self.current_max_raw_v = None
        self.last_action = "waiting"
        self.safety_active = False
        self.fixed_limited = False
        self.settle_until_s = float("-inf")
        if not isinstance(window_control, bool):
            raise ValueError("window_control must be boolean")
        self.window_control = window_control
        self.window_s = self._interval(window_s, "window_s")
        self.window_cooldown_s = self._interval(window_cooldown_s, "window_cooldown_s")
        self.window_mean_min_v = self._finite(window_mean_min_v, "window_mean_min_v")
        self.window_peak_upper_v = self._finite(window_peak_upper_v, "window_peak_upper_v")
        if not 0 <= self.window_mean_min_v < self.window_peak_upper_v < self.safety_limit_v:
            raise ValueError("window limits must satisfy 0 <= mean < peak < safety")
        self.window_high_fraction = None
        if window_high_fraction is not None:
            self.window_high_fraction = self._finite(window_high_fraction, "window_high_fraction")
            if not 0 <= self.window_high_fraction < 1:
                raise ValueError("window_high_fraction must be in [0, 1)")
        if window_control and self.window_high_fraction is None:
            raise ValueError("Explicit window_high_fraction required to enable window control")
        self.window_samples = []
        self.window_dac = None
        self.window_last_over_s = None

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
        self.reset_recovery()

    def reset_recovery(self):
        """Require a fresh continuous stable interval after errors or mode changes."""
        self.recovery_since_s = None
        self.recovery_last_sample_s = None
        self.window_samples = []
        self.window_dac = None

    def _window_decision(self, values, current, now):
        if self.window_last_over_s is None:
            self.window_last_over_s = now
        history = self.window_samples
        if (self.window_dac is None or abs(current - self.window_dac) > 1e-5 or
                (history and (now <= history[-1][0] or
                 now - history[-1][0] > self.recovery_sample_timeout_s))):
            history.clear()
        self.window_dac = current
        history.append((now, values[1], values[2]))
        while len(history) > 1 and history[1][0] <= now - self.window_s:
            history.pop(0)
        self.last_action = "window_waiting"
        if now - history[0][0] < self.window_s:
            return None
        mean = max(sum(row[ch] for row in history) / len(history) for ch in (1, 2))
        high_fraction = sum(max(row[1:]) > self.window_peak_upper_v for row in history) / len(history)
        peak = max(max(row[1:]) for row in history)
        if high_fraction > self.window_high_fraction:
            direction = -1
        elif peak > self.window_peak_upper_v:
            self.last_action = "window_peak_hold"
            return None
        elif mean < self.window_mean_min_v and now - self.window_last_over_s >= self.window_cooldown_s:
            direction = 1
        else:
            self.last_action = "window_holding"
            return None
        if now - self.last_adjustment_s < self.interval_s:
            return None
        target = self._target(current + direction * self.step_v)
        self.reset_counts()
        self.last_adjustment_s = now
        self.last_action = "window_increase" if direction > 0 else "window_decrease"
        return target if target != current else None

    def _fixed_recovery(self, maximum, current, now):
        if not self.fixed_auto_recovery:
            self.last_action = "fixed_limited"
            return None
        if current >= self.fixed_target_v - 0.005:
            self.reset_recovery()
            self.last_action = "fixed_recovery_holding"
            return None
        previous = self.recovery_last_sample_s
        if previous is None or now <= previous or now - previous > self.recovery_sample_timeout_s:
            self.recovery_since_s = None
        self.recovery_last_sample_s = now
        if maximum > self.fixed_recovery_max_adc_v:
            self.recovery_since_s = None
            self.last_action = "fixed_recovery_holding"
            return None
        if self.recovery_since_s is None:
            self.recovery_since_s = now
        if now - self.recovery_since_s < self.fixed_recovery_wait_s:
            self.last_action = "fixed_recovery_waiting"
            return None
        if now - self.recovery_last_step_s < self.fixed_recovery_interval_s:
            self.last_action = "fixed_recovery_waiting"
            return None
        self.recovery_last_step_s = now
        self.reset_recovery()
        self.last_action = "fixed_recovering"
        return self._target(min(self.fixed_target_v, current + self.fixed_recovery_step_v))

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
            self.window_last_over_s = now
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

    def observe(self, raw_voltages, current_dac_v, now_s, data_valid=True):
        try:
            return self._observe(raw_voltages, current_dac_v, now_s, data_valid)
        except (ValueError, TypeError, OverflowError):
            self.reset_counts()
            raise

    def _observe(self, raw_voltages, current_dac_v, now_s, data_valid=True):
        """Return the next DAC voltage, or ``None`` when no change is due."""
        if not isinstance(raw_voltages, (list, tuple)) or len(raw_voltages) != 3:
            self.reset_counts()
            raise ValueError("gain control requires ADC0, ADC1, and ADC2")
        values = []
        for value in raw_voltages:
            try:
                values.append(self._finite(value, "ADC voltage"))
            except ValueError:
                values.append(float("nan"))
        current = self._finite(current_dac_v, "current DAC voltage")
        now = self._finite(now_s, "monotonic time")
        if not self.dac_min_v <= current <= self.dac_max_v:
            self.reset_counts()
            raise ValueError("Current DAC voltage is outside configured limits")
        valid_values = [v for v in values if math.isfinite(v) and v >= 0]
        maximum = max(valid_values, default=float("nan"))
        self.current_max_raw_v = maximum
        if maximum >= self.safety_limit_v:
            return self._safety_decision(maximum, current, now)[1]
        if len(valid_values) != 3 or not data_valid:
            self.reset_counts()
            self.last_action = "invalid_adc"
            if len(valid_values) != 3:
                raise ValueError("ADC voltages must be finite and nonnegative")
            return None
        handled, target = self._safety_decision(maximum, current, now)
        if handled:
            return target

        if self.mode == "off":
            self.reset_counts()
            self.last_action = "off"
            return None

        if self.mode == "fixed":
            self.below_count = self.above_count = 0
            if self.fixed_limited:
                return self._fixed_recovery(maximum, current, now)
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
        if self.window_control:
            return self._window_decision(values, current, now)
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
