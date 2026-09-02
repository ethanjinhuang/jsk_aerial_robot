#!/usr/bin/env python3
"""Fixed/closed-loop LNA gain control with raw-ADC overrange protection."""

import math
import threading
import time

import rospy
from diagnostic_msgs.msg import DiagnosticArray, DiagnosticStatus, KeyValue
from std_msgs.msg import Float32
from std_srvs.srv import SetBool, SetBoolResponse, Trigger, TriggerResponse

from robotic_fish_io import dac_driver
from robotic_fish_io.agc_controller import GainController
from robotic_fish_io.msg import AdcSampleArray, GainControlState
from robotic_fish_io.srv import SetDacVoltage


class GainControlNode:
    def __init__(self):
        self.mode = self._configured_mode()
        self.start_voltage = float(
            dac_driver.normalize_voltage(self._param("start_voltage", 0.0))
        )
        self.dac_channel = int(
            self._param("dac_channel", dac_driver.DEFAULT_CHANNEL)
        )
        self.service_timeout = float(self._param("service_timeout", 1.0))
        self.adc_timeout = float(self._param("adc_timeout", 1.0))
        self.adc_topic = str(
            self._param("adc_samples_topic", "/robotic_fish/adc/samples")
        )
        self.dac_state_topic = str(
            self._param("dac_state_topic", "/robotic_fish/dac/state")
        )
        self.dac_service_name = str(
            self._param("set_voltage_service", "/robotic_fish/dac/set_voltage")
        )
        self.enable_service_name = str(
            self._param("enable_service", "/robotic_fish/agc/enable")
        )
        self.reset_service_name = str(
            self._param(
                "reset_safety_service", "/robotic_fish/gain_control/reset_safety"
            )
        )
        self.state_topic = str(
            self._param("state_topic", "/robotic_fish/gain_control/state")
        )

        self.controller = GainController(
            mode=self.mode,
            fixed_target_v=self._param("fixed_voltage", 0.0),
            target_min_v=self._param("target_min_v", 2.5),
            target_max_v=self._param("target_max_v", 3.0),
            step_v=self._param("step_v", 0.01),
            interval_s=self._param("interval_s", 0.5),
            consecutive_samples=self._param("consecutive_samples", 3),
            dac_min_v=self._param("dac_min_v", 0.0),
            dac_max_v=self._param("dac_max_v", 5.0),
            safety_limit_v=self._param("adc_safety_limit_v", 3.50),
            safety_recovery_v=self._param("adc_safety_recovery_v", 3.30),
            safety_step_v=self._param("safety_step_v", 0.10),
            safety_interval_s=self._param("safety_interval_s", 0.10),
            fixed_ramp_step_v=self._param("fixed_ramp_step_v", 0.05),
            fixed_ramp_interval_s=self._param("fixed_ramp_interval_s", 0.10),
            recovery_settle_s=self._param("recovery_settle_s", 0.20),
            max_normal_step_v=self._param("max_normal_step_v", 0.10),
        )
        dac_driver.validate_channel(self.dac_channel)
        if not math.isfinite(self.service_timeout) or self.service_timeout <= 0.0:
            raise ValueError("gain-control service_timeout must be positive")
        if not math.isfinite(self.adc_timeout) or self.adc_timeout <= 0.0:
            raise ValueError("gain-control adc_timeout must be positive")
        if not self.controller.dac_min_v <= self.start_voltage <= self.controller.dac_max_v:
            raise ValueError("start_voltage is outside configured DAC limits")

        self.lock = threading.RLock()
        self.command_lock = threading.Lock()
        self.current_dac_voltage = None
        self.commanded_dac_voltage = None
        self.last_adc_monotonic_s = None
        self.last_sample_stamp = rospy.Time(0)
        self.adjustments = 0
        self.error_count = 0
        self.last_error = ""

        self.dac_client = rospy.ServiceProxy(self.dac_service_name, SetDacVoltage)
        self.state_pub = rospy.Publisher(
            self.state_topic, GainControlState, queue_size=10, latch=True
        )
        self.diagnostic_pub = rospy.Publisher(
            "/diagnostics", DiagnosticArray, queue_size=10
        )
        self.adc_sub = rospy.Subscriber(
            self.adc_topic, AdcSampleArray, self._adc_callback, queue_size=10
        )
        self.dac_state_sub = rospy.Subscriber(
            self.dac_state_topic, Float32, self._dac_state_callback, queue_size=10
        )
        self.enable_service = rospy.Service(
            self.enable_service_name, SetBool, self._enable_callback
        )
        self.reset_service = rospy.Service(
            self.reset_service_name, Trigger, self._reset_safety_callback
        )
        self.diagnostic_timer = rospy.Timer(
            rospy.Duration(1.0), self._diagnostic_callback
        )
        rospy.loginfo("Gain control started in '%s' mode", self.mode)

    @staticmethod
    def _legacy_param(name, default):
        return rospy.get_param("~agc/{}".format(name), default)

    @classmethod
    def _param(cls, name, default):
        full_name = "~gain_control/{}".format(name)
        if rospy.has_param(full_name):
            return rospy.get_param(full_name)
        return cls._legacy_param(name, default)

    @staticmethod
    def _configured_mode():
        mode_name = "~gain_control/mode"
        legacy_name = "~agc/enabled"
        mode_explicit = rospy.has_param(mode_name)
        mode = str(rospy.get_param(mode_name, "off")).strip().lower()
        legacy_enabled = bool(rospy.get_param(legacy_name, False))
        if mode_explicit and legacy_enabled and mode not in ("off", "closed_loop"):
            raise ValueError(
                "gain_control/mode conflicts with deprecated agc/enabled=true"
            )
        return "closed_loop" if legacy_enabled else mode

    @staticmethod
    def _ordered_raw_voltages(msg):
        by_channel = {}
        for sample in msg.samples:
            channel = int(sample.channel)
            if channel in by_channel:
                raise ValueError("ADC batch contains a duplicate channel")
            by_channel[channel] = float(sample.voltage)
        if set(by_channel) != {0, 1, 2}:
            raise ValueError("gain control requires one ADC0, ADC1, and ADC2 sample")
        return [by_channel[channel] for channel in range(3)]

    def _dac_state_callback(self, msg):
        voltage = float(msg.data)
        if not math.isfinite(voltage):
            rospy.logwarn_throttle(5.0, "Ignoring non-finite DAC state")
            return
        if not self.controller.dac_min_v <= voltage <= self.controller.dac_max_v:
            self._record_error("DAC state is outside configured limits")
            return
        with self.lock:
            self.current_dac_voltage = voltage

    def _enable_callback(self, request):
        # Backward-compatible service: enabling selects closed-loop mode.
        with self.lock:
            self.controller.set_mode("closed_loop" if request.data else "off")
            self.mode = self.controller.mode
            self.last_error = ""
        self._publish_state()
        return SetBoolResponse(True, "gain mode is {}".format(self.mode))

    def _reset_safety_callback(self, _request):
        with self.lock:
            if self.controller.safety_active:
                return TriggerResponse(False, "ADC overrange protection is still active")
            self.controller.reset_safety()
            self.last_error = ""
        self._publish_state()
        return TriggerResponse(True, "gain-control safety latch reset")

    def _set_dac_voltage(self, target):
        try:
            rospy.wait_for_service(self.dac_service_name, timeout=self.service_timeout)
            response = self.dac_client(channel=self.dac_channel, voltage=target)
        except (rospy.ROSException, rospy.ServiceException) as exc:
            raise OSError("DAC service unavailable: {}".format(exc)) from exc
        if not response.success:
            raise OSError("DAC rejected gain-control command: {}".format(response.message))
        applied = float(response.applied_voltage)
        if not math.isfinite(applied):
            raise OSError("DAC returned a non-finite applied voltage")
        if not self.controller.dac_min_v <= applied <= self.controller.dac_max_v:
            raise OSError("DAC returned an applied voltage outside configured limits")
        expected = float(dac_driver.normalize_voltage(target))
        if abs(applied - expected) >= 0.005:
            raise OSError(
                "DAC applied {:.2f} V instead of requested {:.2f} V".format(
                    applied, expected
                )
            )
        return applied

    def _record_error(self, message):
        with self.lock:
            self.controller.reset_counts()
            self.error_count += 1
            self.last_error = str(message)
        rospy.logerr_throttle(2.0, "Gain control: %s", message)

    def _adc_callback(self, msg):
        try:
            raw_voltages = self._ordered_raw_voltages(msg)
        except ValueError as exc:
            self._record_error(exc)
            self._publish_state()
            return

        now = time.monotonic()
        with self.command_lock:
            with self.lock:
                self.last_sample_stamp = msg.header.stamp
                self.last_adc_monotonic_s = now
                current = self.current_dac_voltage
                mode = self.controller.mode

            if current is None:
                maximum = max(raw_voltages)
                # Off mode normally observes external DAC commands without taking
                # ownership. An ADC overrange is the exception: command the known
                # safe DAC minimum even when no prior DAC state was received.
                if mode == "off" and maximum < self.controller.safety_limit_v:
                    with self.lock:
                        self.controller.current_max_raw_v = maximum
                        self.controller.last_action = "waiting_for_dac_state"
                    self._publish_state()
                    return
                try:
                    initial_target = (
                        self.controller.dac_min_v
                        if maximum >= self.controller.safety_limit_v
                        else self.start_voltage
                    )
                    applied = self._set_dac_voltage(initial_target)
                except OSError as exc:
                    self._record_error(exc)
                    self._publish_state()
                    return
                with self.lock:
                    self.current_dac_voltage = applied
                    self.commanded_dac_voltage = initial_target
                    self.adjustments += 1
                    self.controller.current_max_raw_v = maximum
                    if maximum >= self.controller.safety_limit_v:
                        self.controller.safety_active = True
                        self.controller.last_action = "adc_overrange_at_dac_min"
                    else:
                        self.controller.last_action = "initialized_dac"
                    self.last_error = ""
                self._publish_state()
                return

            try:
                target = self.controller.observe(raw_voltages, current, now)
            except ValueError as exc:
                self._record_error(exc)
                self._publish_state()
                return

            if target is not None:
                try:
                    applied = self._set_dac_voltage(target)
                except OSError as exc:
                    self._record_error(exc)
                    self._publish_state()
                    return
                with self.lock:
                    self.current_dac_voltage = applied
                    self.commanded_dac_voltage = target
                    self.adjustments += 1
                    self.last_error = ""
                rospy.loginfo(
                    "Gain control %s: DAC %.2f V, raw ADC max %.6f V",
                    self.controller.last_action,
                    applied,
                    self.controller.current_max_raw_v,
                )
            else:
                with self.lock:
                    self.last_error = ""
            self._publish_state()

    def _adc_is_valid(self):
        return (
            self.last_adc_monotonic_s is not None
            and time.monotonic() - self.last_adc_monotonic_s <= self.adc_timeout
        )

    def _state_message(self):
        msg = GainControlState()
        msg.header.stamp = rospy.Time.now()
        msg.mode = self.controller.mode
        msg.state = "adc_stale" if not self._adc_is_valid() else self.controller.last_action
        msg.reason = self.last_error
        msg.current_dac_voltage = (
            float("nan") if self.current_dac_voltage is None else self.current_dac_voltage
        )
        msg.commanded_dac_voltage = (
            float("nan")
            if self.commanded_dac_voltage is None
            else self.commanded_dac_voltage
        )
        msg.fixed_target_voltage = self.controller.fixed_target_v
        msg.adc_max_raw_voltage = (
            float("nan")
            if self.controller.current_max_raw_v is None
            else self.controller.current_max_raw_v
        )
        msg.target_min_voltage = self.controller.target_min_v
        msg.target_max_voltage = self.controller.target_max_v
        msg.adc_safety_limit_voltage = self.controller.safety_limit_v
        msg.adc_safety_recovery_voltage = self.controller.safety_recovery_v
        msg.safety_active = self.controller.safety_active
        msg.adc_valid = self._adc_is_valid()
        msg.dac_state_valid = self.current_dac_voltage is not None
        msg.low_sample_count = self.controller.below_count
        msg.high_sample_count = self.controller.above_count
        msg.adjustment_count = self.adjustments
        msg.error_count = self.error_count
        return msg

    def _publish_state(self):
        with self.lock:
            self.state_pub.publish(self._state_message())

    def _diagnostic_callback(self, _event):
        with self.lock:
            msg = self._state_message()
            status = DiagnosticStatus()
            status.name = "robotic_fish_io/gain_control"
            status.hardware_id = "adc-max-to-dac"
            if self.last_error or msg.state == "adc_overrange_at_dac_min":
                status.level = DiagnosticStatus.ERROR
                status.message = self.last_error or msg.state
            elif msg.safety_active or not msg.adc_valid or not msg.dac_state_valid:
                status.level = DiagnosticStatus.WARN
                status.message = msg.state
            else:
                status.level = DiagnosticStatus.OK
                status.message = msg.state
            status.values = [
                KeyValue("mode", msg.mode),
                KeyValue("state", msg.state),
                KeyValue("adc_valid", str(msg.adc_valid)),
                KeyValue("adc_max_raw_v", str(msg.adc_max_raw_voltage)),
                KeyValue("dac_state_valid", str(msg.dac_state_valid)),
                KeyValue("dac_voltage_v", str(msg.current_dac_voltage)),
                KeyValue("safety_active", str(msg.safety_active)),
                KeyValue("adjustments", str(msg.adjustment_count)),
                KeyValue("errors", str(msg.error_count)),
            ]
            array = DiagnosticArray()
            array.header.stamp = msg.header.stamp
            array.status = [status]
            self.diagnostic_pub.publish(array)
            self.state_pub.publish(msg)


# Preserve the old class name for code importing the ROS node directly.
AgcNode = GainControlNode


def main():
    rospy.init_node("gain_control")
    try:
        GainControlNode()
        rospy.spin()
    except (ValueError, TypeError) as exc:
        rospy.logfatal("Invalid gain-control configuration: %s", exc)
        raise


if __name__ == "__main__":
    main()
