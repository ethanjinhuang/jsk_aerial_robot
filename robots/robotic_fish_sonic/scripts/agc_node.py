#!/usr/bin/env python3
"""Manual/closed-loop LNA gain control with raw-ADC overrange protection."""

import math
import threading
import time

import rospy
from diagnostic_msgs.msg import DiagnosticArray, DiagnosticStatus, KeyValue
from robotic_fish_sonic.msg import DacState
from robotic_fish_sonic.runtime_config import ConfigRecorder
from robotic_fish_sonic.parameters import get_compatible_param
from sensor_msgs.msg import Joy
from robotic_fish_sonic.manual_gain import ManualGainInput
from std_srvs.srv import SetBool, SetBoolResponse, Trigger, TriggerResponse

from robotic_fish_sonic import dac_driver
from robotic_fish_sonic.adc_limits import MAX_RAW, MAX_VOLTAGE_V
from robotic_fish_sonic.agc_controller import GainController
from robotic_fish_sonic.msg import AdcSampleArray, GainControlState
from robotic_fish_sonic.srv import SetDacVoltage


class GainControlNode:
    def __init__(self):
        self.mode = self._configured_mode()
        self.start_voltage_v = float(
            dac_driver.normalize_voltage(self._param("start_voltage", 0.0))
        )
        self.dac_channel_id = int(
            self._param("dac_channel", dac_driver.DEFAULT_CHANNEL)
        )
        self.service_timeout_s = float(self._param("service_timeout", 1.0))
        self.adc_timeout_s = float(self._param("adc_timeout", 1.0))
        self.adc_samples_topic = str(
            self._param("adc_samples_topic", "/robotic_fish/adc/samples")
        )
        self.dac_state_topic = str(
            self._param("dac_state_topic", "/robotic_fish/dac/state")
        )
        self.set_dac_voltage_service_name = str(
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
            manual_target_v=self._param("manual_voltage", 0.0),
            target_min_v=self._param("target_min_v", 2.5),
            target_max_v=self._param("target_max_v", 3.0),
            step_v=self._param("step_v", 0.01),
            interval_s=self._param("interval_s", 0.5),
            consecutive_samples=self._param("consecutive_samples", 3),
            dac_min_v=self._param("dac_min_v", 0.0),
            dac_max_v=self._param("dac_max_v", 5.0),
            safety_limit_v=self._param("adc_safety_limit_v", 4.00),
            safety_recovery_v=self._param("adc_safety_recovery_v", 3.30),
            safety_step_v=self._param("safety_step_v", 0.10),
            safety_interval_s=self._param("safety_interval_s", 0.10),
            manual_ramp_step_v=self._param("manual_ramp_step_v", 0.05),
            manual_ramp_interval_s=self._param("manual_ramp_interval_s", 0.10),
            recovery_settle_s=self._param("recovery_settle_s", 0.20),
            max_normal_step_v=self._param("max_normal_step_v", 0.10),
            recovery_sample_timeout_s=self._param("recovery_sample_timeout_s", 0.2),
            window_control=self._param("window_control", False),
            window_s=self._param("window_s", 2.0),
            window_mean_min_v=self._param("window_mean_min_v", 2.0),
            window_peak_upper_v=self._param("window_peak_upper_v", 3.0),
            window_high_fraction=self._param("window_high_fraction", None),
            window_cooldown_s=self._param("window_cooldown_s", 5.0),
        )
        dac_driver.validate_channel(self.dac_channel_id)
        if not math.isfinite(self.service_timeout_s) or self.service_timeout_s <= 0.0:
            raise ValueError("gain-control service_timeout must be positive")
        if not math.isfinite(self.adc_timeout_s) or self.adc_timeout_s <= 0.0:
            raise ValueError("gain-control adc_timeout must be positive")
        if not self.controller.dac_min_v <= self.start_voltage_v <= self.controller.dac_max_v:
            raise ValueError("start_voltage is outside configured DAC limits")

        self.joy_topic = str(self._param("joy_topic", "/joy"))
        self.manual_input = ManualGainInput(self._param("manual_step_v", 0.01),
                                            self.controller.max_normal_step_v)

        self.lock = threading.RLock()
        self.command_lock = threading.Lock()
        self.confirmed_dac_voltage_v = None
        self.requested_dac_voltage_v = None
        self.last_valid_adc_received_monotonic_s = None
        self.last_accepted_sample_stamps = {}
        self.last_state_key = None
        self.latest_sample_group_valid = False
        self.adjustment_count = 0
        self.error_count = 0
        self.last_error = ""

        self.config_recorder = ConfigRecorder("gain_control")
        self._publish_config()
        self.set_dac_voltage_client = rospy.ServiceProxy(self.set_dac_voltage_service_name, SetDacVoltage)
        self.state_publisher = rospy.Publisher(
            self.state_topic, GainControlState, queue_size=10, latch=True
        )
        self.diagnostics_publisher = rospy.Publisher(
            "/diagnostics", DiagnosticArray, queue_size=10
        )
        self.adc_samples_subscriber = rospy.Subscriber(
            self.adc_samples_topic, AdcSampleArray, self._on_adc_samples, queue_size=10
        )
        self.dac_state_subscriber = rospy.Subscriber(
            self.dac_state_topic, DacState, self._on_dac_state, queue_size=10
        )
        self.joy_subscriber = rospy.Subscriber(self.joy_topic, Joy, self._on_joy, queue_size=1)
        self.enable_service = rospy.Service(
            self.enable_service_name, SetBool, self._on_set_enabled
        )
        self.reset_service = rospy.Service(
            self.reset_service_name, Trigger, self._on_reset_safety
        )
        self.diagnostic_timer = rospy.Timer(
            rospy.Duration(1.0), self._diagnostic_callback
        )
        self._publish_state()
        rospy.loginfo("Gain control started in '%s' mode", self.mode)
        rospy.loginfo("Raw ADC protection: trigger %.6f V, release %.6f V",
                      self.controller.safety_limit_v, self.controller.safety_recovery_v)

    @classmethod
    def _param(cls, name, default):
        canonical_names = {
            "start_voltage": "start_voltage_v", "dac_channel": "dac_channel_id",
            "service_timeout": "service_timeout_s", "adc_timeout": "adc_timeout_s",
            "manual_voltage": "manual_target_voltage_v",
            "manual_step_v": "manual_voltage_step_v",
            "target_min_v": "target_min_voltage_v", "target_max_v": "target_max_voltage_v",
            "step_v": "voltage_step_v", "consecutive_samples": "consecutive_sample_group_count",
            "dac_min_v": "dac_min_voltage_v", "dac_max_v": "dac_max_voltage_v",
            "max_normal_step_v": "max_normal_voltage_step_v",
            "adc_safety_limit_v": "adc_safety_trigger_voltage_v",
            "adc_safety_recovery_v": "adc_safety_recovery_voltage_v",
            "safety_step_v": "safety_voltage_step_v",
            "manual_ramp_step_v": "manual_ramp_voltage_step_v",
            "window_control": "window_control_enabled",
            "window_mean_min_v": "window_mean_min_voltage_v",
            "window_peak_upper_v": "window_peak_max_voltage_v",
            "window_high_fraction": "window_high_sample_ratio",
        }
        return get_compatible_param(
            "~gain_control/" + canonical_names.get(name, name), default,
            "~gain_control/" + name, "~agc/" + name)

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
        raw_voltage_by_channel_v = {}
        for sample in msg.samples:
            channel_id = int(sample.channel_id)
            if channel_id in raw_voltage_by_channel_v:
                raise ValueError("ADC batch contains a duplicate channel")
            raw_voltage_by_channel_v[channel_id] = (MAX_VOLTAGE_V if sample.adc_code == MAX_RAW
                                   else float(sample.volt_raw))
        if set(raw_voltage_by_channel_v) != {0, 1, 2}:
            raise ValueError("gain control requires one ADC0, ADC1, and ADC2 sample")
        return [raw_voltage_by_channel_v[channel_id] for channel_id in range(3)]

    def _on_dac_state(self, msg):
        dac_voltage_v = float(msg.dac_volt)
        if not msg.status_dac_feedback or not math.isfinite(dac_voltage_v):
            with self.lock:
                self.controller.reset_counts()
                self.confirmed_dac_voltage_v = None
            rospy.logwarn_throttle(5.0, "Ignoring non-finite DAC state")
            self._publish_state()
            return
        if not self.controller.dac_min_v <= dac_voltage_v <= self.controller.dac_max_v:
            with self.lock:
                self.confirmed_dac_voltage_v = None
            self._record_error("DAC state is outside configured limits")
            self._publish_state()
            return
        with self.lock:
            if self.confirmed_dac_voltage_v is None or abs(dac_voltage_v - self.confirmed_dac_voltage_v) > 1e-5:
                self.controller.reset_window()
            self.confirmed_dac_voltage_v = dac_voltage_v
        self._publish_state()

    def _on_joy(self, msg):
        # Serialize target edits with ADC decisions and service execution.
        with self.command_lock, self.lock:
            try:
                step_direction, voltage_direction = self.manual_input.events(msg.axes)
            except ValueError as exc:
                if self.controller.mode == "manual":
                    rospy.logwarn_throttle(5.0, "%s", exc)
                return
            if self.controller.mode != "manual":
                return
            if step_direction:
                self.manual_input.adjust_step(step_direction)
                rospy.loginfo("Manual gain %s: step=%.2f V, target=%.2f V, DAC=%s",
                              "LEFT" if step_direction > 0 else "RIGHT",
                              self.manual_input.step_v, self.controller.manual_target_v,
                              self.confirmed_dac_voltage_v)
            if voltage_direction:
                key = "UP" if voltage_direction > 0 else "DOWN"
                if (self.controller.safety_active or not self._adc_is_valid() or
                        self.confirmed_dac_voltage_v is None or self.last_error):
                    rospy.loginfo("Manual gain %s blocked: safety=%s, ADC valid=%s, DAC=%s, error=%s",
                                  key, self.controller.safety_active, self._adc_is_valid(),
                                  self.confirmed_dac_voltage_v, self.last_error)
                else:
                    base_voltage_v = (self.confirmed_dac_voltage_v if self.controller.manual_limited
                            else self.controller.manual_target_v)
                    self.controller.manual_target_v = self.controller._target(
                        base_voltage_v + voltage_direction * self.manual_input.step_v)
                    self.controller.manual_limited = False
                    self.controller.reset_counts()
                    self.controller.last_action = "manual_target_changed"
                    rospy.loginfo("Manual gain %s: step=%.2f V, target=%.2f V, DAC=%.2f V",
                                  key, self.manual_input.step_v,
                                  self.controller.manual_target_v, self.confirmed_dac_voltage_v)
            if step_direction or voltage_direction:
                self._publish_config()
                self._publish_state()

    def _on_set_enabled(self, request):
        # Backward-compatible service: enabling selects closed-loop mode.
        # A successful response also confirms that earlier DAC work has finished.
        with self.command_lock, self.lock:
            self.controller.set_mode("closed_loop" if request.data else "off")
            self.mode = self.controller.mode
            self.last_error = ""
            self._publish_config()
            self._publish_state()
            return SetBoolResponse(True, "gain mode is {}".format(self.mode))

    def _on_reset_safety(self, _request):
        with self.command_lock, self.lock:
            if self.controller.safety_active:
                return TriggerResponse(False, "ADC overrange protection is still active")
            self.controller.reset_safety()
            self.last_error = ""
            self._publish_state()
            return TriggerResponse(True, "gain-control safety latch reset")

    def _set_dac_voltage(self, target_voltage_v):
        try:
            rospy.wait_for_service(self.set_dac_voltage_service_name, timeout=self.service_timeout_s)
            response = self.set_dac_voltage_client(channel=self.dac_channel_id, voltage=target_voltage_v)
        except (rospy.ROSException, rospy.ServiceException) as exc:
            raise OSError("DAC service unavailable: {}".format(exc)) from exc
        if not response.success:
            raise OSError("DAC rejected gain-control command: {}".format(response.message))
        applied_voltage_v = float(response.applied_voltage)
        if not math.isfinite(applied_voltage_v):
            raise OSError("DAC returned a non-finite applied voltage")
        if not self.controller.dac_min_v <= applied_voltage_v <= self.controller.dac_max_v:
            raise OSError("DAC returned an applied voltage outside configured limits")
        expected_voltage_v = float(dac_driver.normalize_voltage(target_voltage_v))
        if abs(applied_voltage_v - expected_voltage_v) >= 0.005:
            raise OSError(
                "DAC applied {:.2f} V instead of requested {:.2f} V".format(
                    applied_voltage_v, expected_voltage_v
                )
            )
        return applied_voltage_v

    def _record_error(self, message):
        with self.lock:
            self.controller.reset_counts()
            self.error_count += 1
            self.last_error = str(message)
        rospy.logerr_throttle(2.0, "Gain control: %s", message)

    def _on_adc_samples(self, msg):
        try:
            raw_voltages_v = self._ordered_raw_voltages(msg)
        except ValueError as exc:
            self._record_error(exc)
            # A malformed batch must not hide an independently observed overrange.
            observed_raw_voltages_v = [MAX_VOLTAGE_V if s.adc_code == MAX_RAW else float(s.volt_raw)
                        for s in msg.samples if s.channel_id in (0, 1, 2)]
            overrange_voltages_v = [v for v in observed_raw_voltages_v if math.isfinite(v) and v >= self.controller.safety_limit_v]
            raw_voltages_v = [max(overrange_voltages_v) if overrange_voltages_v else float("nan"), float("nan"), float("nan")]

        with self.command_lock:
            received_monotonic_s = time.monotonic()
            with self.lock:
                # Stale/repeated batches cannot accumulate recovery dwell time.
                recovery_batch_valid = True
                received_stamp = rospy.Time.now()
                for sample in msg.samples:
                    if (sample.timestamp <= self.last_accepted_sample_stamps.get(sample.channel_id, rospy.Time(0)) or
                            sample.timestamp > received_stamp or
                            (received_stamp - sample.timestamp).to_sec() > self.adc_timeout_s):
                        recovery_batch_valid = False
                recovery_batch_valid = recovery_batch_valid and all(
                    math.isfinite(v) and v >= 0 for v in raw_voltages_v)
                if self.confirmed_dac_voltage_v is not None and self.controller.window_control:
                    recovery_batch_valid = recovery_batch_valid and all(
                        sample.status_dac_feedback and
                        math.isfinite(sample.dac_volt) and
                        abs(sample.dac_volt - self.confirmed_dac_voltage_v) < 1e-5
                        for sample in msg.samples)
                if not recovery_batch_valid:
                    self.controller.reset_counts()
                    self.last_error = "Invalid or stale ADC batch; normal control blocked"
                else:
                    self.last_accepted_sample_stamps = {sample.channel_id: sample.timestamp for sample in msg.samples}
                    self.last_valid_adc_received_monotonic_s = received_monotonic_s
                self.latest_sample_group_valid = recovery_batch_valid
                confirmed_voltage_v = self.confirmed_dac_voltage_v
                mode = self.controller.mode

            if confirmed_voltage_v is None:
                maximum_raw_voltage_v = max((v for v in raw_voltages_v if math.isfinite(v)), default=float("nan"))
                if not recovery_batch_valid and not maximum_raw_voltage_v >= self.controller.safety_limit_v:
                    self._record_error("Invalid or stale ADC; initialization blocked")
                    self._publish_state()
                    return
                # Off mode normally observes external DAC commands without taking
                # ownership. An ADC overrange is the exception: command the known
                # safe DAC minimum even when no prior DAC state was received.
                if mode == "off" and maximum_raw_voltage_v < self.controller.safety_limit_v:
                    with self.lock:
                        self.controller.current_max_raw_v = maximum_raw_voltage_v
                        self.controller.last_action = "waiting_for_dac_state"
                    self._publish_state()
                    return
                try:
                    initial_voltage_v = (
                        self.controller.dac_min_v
                        if maximum_raw_voltage_v >= self.controller.safety_limit_v
                        else self.start_voltage_v
                    )
                    applied_voltage_v = self._set_dac_voltage(initial_voltage_v)
                except OSError as exc:
                    self._record_error(exc)
                    self._publish_state()
                    return
                with self.lock:
                    self.confirmed_dac_voltage_v = applied_voltage_v
                    self.requested_dac_voltage_v = initial_voltage_v
                    self.adjustment_count += 1
                    self.controller.current_max_raw_v = maximum_raw_voltage_v
                    if maximum_raw_voltage_v >= self.controller.safety_limit_v:
                        self.controller.safety_active = True
                        self.controller.last_action = "adc_overrange_at_dac_min"
                    else:
                        self.controller.last_action = "initialized_dac"
                    if recovery_batch_valid:
                        self.last_error = ""
                self._publish_state()
                return

            try:
                with self.lock:
                    requested_voltage_v = self.controller.observe(raw_voltages_v, confirmed_voltage_v, received_monotonic_s,
                                                     data_valid=recovery_batch_valid)
            except ValueError as exc:
                self._record_error(exc)
                self._publish_state()
                return

            if requested_voltage_v is not None:
                try:
                    applied_voltage_v = self._set_dac_voltage(requested_voltage_v)
                except OSError as exc:
                    self._record_error(exc)
                    self._publish_state()
                    return
                with self.lock:
                    self.confirmed_dac_voltage_v = applied_voltage_v
                    self.requested_dac_voltage_v = requested_voltage_v
                    self.adjustment_count += 1
                    if recovery_batch_valid:
                        self.last_error = ""
                rospy.loginfo(
                    "Gain control %s: DAC %.2f V, raw ADC max %.6f V",
                    self.controller.last_action,
                    applied_voltage_v,
                    self.controller.current_max_raw_v,
                )
            else:
                with self.lock:
                    if recovery_batch_valid:
                        self.last_error = ""
            self._publish_state()

    def _adc_is_valid(self):
        return (
            self.latest_sample_group_valid and self.last_valid_adc_received_monotonic_s is not None
            and time.monotonic() - self.last_valid_adc_received_monotonic_s <= self.adc_timeout_s
        )

    def _publish_config(self):
        # Constructor attributes are the normalized, effective controller settings.
        import inspect
        settings = {name: getattr(self.controller, name) for name in
                    inspect.signature(GainController.__init__).parameters if name != "self"}
        settings.update(start_voltage=self.start_voltage_v, dac_channel=self.dac_channel_id,
                        service_timeout=self.service_timeout_s, adc_timeout=self.adc_timeout_s,
                        adc_topic=self.adc_samples_topic, dac_state_topic=self.dac_state_topic,
                        dac_service=self.set_dac_voltage_service_name)
        settings.update(joy_topic=self.joy_topic, manual_step_v=self.manual_input.step_v)
        self.config_recorder.publish(settings)

    def _state_message(self):
        msg = GainControlState()
        msg.timestamp = rospy.Time.now()
        msg.mode = self.controller.mode
        # Priority: active protection > invalid/stale input > DAC unknown > error > action.
        details = []
        if not self._adc_is_valid():
            details.append("ADC invalid or stale")
        if self.confirmed_dac_voltage_v is None:
            details.append("DAC state unknown")
        if self.last_error:
            details.append(self.last_error)
        if self.controller.safety_active:
            msg.state = ("adc_overrange_at_dac_min" if self.controller.last_action ==
                         "adc_overrange_at_dac_min" else "safety_active")
        elif not self._adc_is_valid():
            msg.state = "adc_stale"
        elif self.confirmed_dac_voltage_v is None:
            msg.state = "waiting_for_dac_state"
        elif self.last_error:
            msg.state = "error"
        else:
            msg.state = self.controller.last_action
        msg.log = "; ".join(details)
        return msg

    def _publish_state(self):
        with self.lock:
            msg = self._state_message()
            key = (msg.mode, msg.state, msg.log)
            if key != self.last_state_key:
                self.state_publisher.publish(msg)
                self.last_state_key = key

    def _diagnostic_callback(self, _event):
        with self.lock:
            msg = self._state_message()
            status = DiagnosticStatus()
            status.name = "robotic_fish_sonic/gain_control"
            status.hardware_id = "adc-max-to-dac"
            status.level = (DiagnosticStatus.ERROR if self.last_error or
                msg.state == "adc_overrange_at_dac_min" else DiagnosticStatus.WARN if
                self.controller.safety_active or not self._adc_is_valid() or
                self.confirmed_dac_voltage_v is None else DiagnosticStatus.OK)
            status.message = msg.log or msg.state
            status.values = [KeyValue(key, str(value)) for key, value in dict(
                mode=msg.mode, state=msg.state, status_adc=self._adc_is_valid(),
                adc_volt_raw_max=self.controller.current_max_raw_v,
                status_dac_feedback=self.confirmed_dac_voltage_v is not None,
                dac_volt=self.confirmed_dac_voltage_v, status_safety=self.controller.safety_active,
                count_low=self.controller.below_count, count_high=self.controller.above_count,
                count_adjustment=self.adjustment_count, count_error=self.error_count).items()]
            array = DiagnosticArray()
            array.header.stamp = msg.timestamp
            array.status = [status]
            self.diagnostics_publisher.publish(array)
            self._publish_state()


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
