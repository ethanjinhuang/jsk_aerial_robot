"""Edge-triggered joystick input for manual DAC gain control."""
import math

from robotic_fish_io import dac_driver


class ManualGainInput:
    def __init__(self, step_v=0.01, max_step_v=0.10):
        self.step_v = float(dac_driver.normalize_voltage(step_v))
        self.max_step_v = float(dac_driver.normalize_voltage(max_step_v))
        if not 0.01 <= self.step_v <= self.max_step_v:
            raise ValueError("manual step must be between 0.01 V and max_normal_step_v")
        self.previous = (0, 0)

    def events(self, axes):
        if len(axes) <= 10 or not all(math.isfinite(v) for v in (axes[9], axes[10])):
            # Do not re-arm a held button on a malformed message.
            raise ValueError("manual gain requires finite axes[9] and axes[10]")
        current = tuple(1 if v > 0.5 else -1 if v < -0.5 else 0
                        for v in (axes[9], axes[10]))
        events = tuple(value if value != old else 0
                       for value, old in zip(current, self.previous))
        self.previous = current
        return events

    def adjust_step(self, direction):
        self.step_v = round(min(self.max_step_v, max(0.01,
                            self.step_v + direction * 0.01)), 2)
        return self.step_v
