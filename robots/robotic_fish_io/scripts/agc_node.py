#!/usr/bin/env python3
"""Closed-loop AGC using raw ADC batches and the DAC voltage service."""

import math
import threading
import time

import rospy
from diagnostic_msgs.msg import DiagnosticArray, DiagnosticStatus, KeyValue
from std_msgs.msg import Float32
from std_srvs.srv import SetBool, SetBoolResponse

from robotic_fish_io.agc_controller import AgcController
from robotic_fish_io import dac_driver
from robotic_fish_io.msg import AdcSampleArray
from robotic_fish_io.srv import SetDacVoltage


class AgcNode:
    def __init__(self):
        self.enabled = bool(rospy.get_param("~agc/enabled", True))
        self.start_voltage = float(
            dac_driver.normalize_voltage(rospy.get_param("~agc/start_voltage", 0.0))
        )
        self.dac_channel = int(
            rospy.get_param("~agc/dac_channel", dac_driver.DEFAULT_CHANNEL)
        )
        self.service_timeout = float(
            rospy.get_param("~agc/service_timeout", 1.0)
        )
        self.adc_topic = str(
            rospy.get_param("~agc/adc_samples_topic", "/robotic_fish/adc/samples")
        )
        self.dac_state_topic = str(
            rospy.get_param("~agc/dac_state_topic", "/robotic_fish/dac/state")
        )
        self.dac_service_name = str(
            rospy.get_param(
                "~agc/set_voltage_service", "/robotic_fish/dac/set_voltage"
            )
        )
        self.enable_service_name = str(
            rospy.get_param("~agc/enable_service", "/robotic_fish/agc/enable")
        )
        self.controller = AgcController(
            target_min_v=rospy.get_param("~agc/target_min_v", 2.5),
            target_max_v=rospy.get_param("~agc/target_max_v", 3.0),
            step_v=rospy.get_param("~agc/step_v", 0.01),
            interval_s=rospy.get_param("~agc/interval_s", 0.5),
            consecutive_samples=rospy.get_param("~agc/consecutive_samples", 3),
            dac_min_v=rospy.get_param("~agc/dac_min_v", 0.0),
            dac_max_v=rospy.get_param("~agc/dac_max_v", 5.0),
        )
        dac_driver.validate_channel(self.dac_channel)
        if not math.isfinite(self.service_timeout) or self.service_timeout <= 0:
            raise ValueError("AGC service_timeout must be positive")

        self.lock = threading.RLock()
        self.command_lock = threading.Lock()
        self.current_dac_voltage = None
        self.adjustments = 0
        self.error_count = 0
        self.last_error = ""
        self.status = "waiting for DAC state"
        self.last_sample_stamp = rospy.Time(0)

        self.dac_client = rospy.ServiceProxy(self.dac_service_name, SetDacVoltage)
        self.adc_sub = rospy.Subscriber(
            self.adc_topic, AdcSampleArray, self._adc_callback, queue_size=10
        )
        self.dac_state_sub = rospy.Subscriber(
            self.dac_state_topic, Float32, self._dac_state_callback, queue_size=10
        )
        self.enable_service = rospy.Service(
            self.enable_service_name, SetBool, self._enable_callback
        )
        self.diagnostic_pub = rospy.Publisher(
            "/diagnostics", DiagnosticArray, queue_size=10
        )
        self.diagnostic_timer = rospy.Timer(
            rospy.Duration(1.0), self._diagnostic_callback
        )

    def _dac_state_callback(self, msg):
        voltage = float(msg.data)
        if not math.isfinite(voltage):
            rospy.logwarn_throttle(5.0, "Ignoring non-finite DAC state")
            return
        with self.lock:
            self.current_dac_voltage = voltage
            if self.status == "waiting for DAC state":
                self.status = "waiting for ADC samples"

    def _enable_callback(self, request):
        with self.lock:
            self.enabled = bool(request.data)
            self.controller.reset_counts()
            self.last_error = ""
            self.status = "enabled" if self.enabled else "disabled"
        return SetBoolResponse(True, "AGC {}".format(self.status))

    @staticmethod
    def _ordered_raw_voltages(msg):
        by_channel = {}
        for sample in msg.samples:
            channel = int(sample.channel)
            if channel in by_channel:
                raise ValueError("ADC batch contains a duplicate channel")
            by_channel[channel] = float(sample.voltage)
        if set(by_channel) != {0, 1, 2}:
            raise ValueError("AGC requires one ADC0, ADC1, and ADC2 sample")
        return [by_channel[channel] for channel in range(3)]

    def _set_dac_voltage(self, target):
        try:
            rospy.wait_for_service(self.dac_service_name, timeout=self.service_timeout)
            response = self.dac_client(channel=self.dac_channel, voltage=target)
        except (rospy.ROSException, rospy.ServiceException) as exc:
            raise OSError("DAC service unavailable: {}".format(exc)) from exc
        if not response.success:
            raise OSError("DAC rejected AGC command: {}".format(response.message))
        return float(response.applied_voltage)

    def _record_error(self, message, disable=False):
        with self.lock:
            self.controller.reset_counts()
            self.error_count += 1
            self.last_error = str(message)
            self.status = "error"
            if disable:
                self.enabled = False
        rospy.logerr_throttle(2.0, "AGC: %s", message)

    def _adc_callback(self, msg):
        try:
            raw_voltages = self._ordered_raw_voltages(msg)
        except ValueError as exc:
            self._record_error(exc)
            return

        with self.command_lock:
            with self.lock:
                self.last_sample_stamp = msg.header.stamp
                if not self.enabled:
                    self.controller.reset_counts()
                    self.status = "disabled"
                    return
                current = self.current_dac_voltage

            if current is None:
                try:
                    applied = self._set_dac_voltage(self.start_voltage)
                except OSError as exc:
                    self._record_error(exc, disable=True)
                    return
                with self.lock:
                    self.current_dac_voltage = applied
                    self.status = "initialized DAC"
                rospy.loginfo("AGC initialized DAC to %.2f V", applied)
                return

            try:
                target = self.controller.observe(
                    raw_voltages, current, time.monotonic()
                )
            except ValueError as exc:
                self._record_error(exc)
                return

            with self.lock:
                self.status = self.controller.last_action
                self.last_error = ""
            if target is None:
                return

            try:
                applied = self._set_dac_voltage(target)
            except OSError as exc:
                self._record_error(exc, disable=True)
                return
            with self.lock:
                self.current_dac_voltage = applied
                self.adjustments += 1
                self.status = self.controller.last_action
                self.last_error = ""
            rospy.loginfo(
                "AGC %s DAC to %.2f V (raw ADC max %.6f V)",
                self.controller.last_action,
                applied,
                self.controller.current_max_raw_v,
            )

    def _diagnostic_callback(self, _event):
        with self.lock:
            status = DiagnosticStatus()
            status.name = "robotic_fish_io/agc"
            status.hardware_id = "adc-max-to-dac"
            if self.last_error:
                status.level = DiagnosticStatus.ERROR
                status.message = self.last_error
            elif not self.enabled:
                status.level = DiagnosticStatus.OK
                status.message = "AGC disabled"
            elif self.current_dac_voltage is None:
                status.level = DiagnosticStatus.WARN
                status.message = "AGC waiting for DAC initialization"
            else:
                status.level = DiagnosticStatus.OK
                status.message = self.status
            status.values = [
                KeyValue("enabled", str(self.enabled)),
                KeyValue(
                    "adc_max_raw_v",
                    "unknown"
                    if self.controller.current_max_raw_v is None
                    else "{:.6f}".format(self.controller.current_max_raw_v),
                ),
                KeyValue(
                    "dac_voltage_v",
                    "unknown"
                    if self.current_dac_voltage is None
                    else "{:.2f}".format(self.current_dac_voltage),
                ),
                KeyValue("action", self.controller.last_action),
                KeyValue("below_count", str(self.controller.below_count)),
                KeyValue("above_count", str(self.controller.above_count)),
                KeyValue("adjustments", str(self.adjustments)),
                KeyValue("errors", str(self.error_count)),
            ]
            array = DiagnosticArray()
            array.header.stamp = rospy.Time.now()
            array.status = [status]
        self.diagnostic_pub.publish(array)


def main():
    rospy.init_node("agc")
    try:
        AgcNode()
        rospy.spin()
    except (ValueError, TypeError) as exc:
        rospy.logfatal("Invalid AGC configuration: %s", exc)
        raise


if __name__ == "__main__":
    main()
