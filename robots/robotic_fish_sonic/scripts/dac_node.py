#!/usr/bin/env python3
"""Expose safe serial DAC control through a ROS service and execution-result topic."""

import threading
import time

import rospy
import serial
from diagnostic_msgs.msg import DiagnosticArray, DiagnosticStatus, KeyValue
from robotic_fish_sonic.msg import DacState
from robotic_fish_sonic.runtime_config import ConfigRecorder
from robotic_fish_sonic.parameters import get_compatible_param

from robotic_fish_sonic import dac_driver
from robotic_fish_sonic.srv import SetDacVoltage, SetDacVoltageResponse


class DacNode:
    def __init__(self):
        self.serial_port = str(get_compatible_param(
            "~dac/serial_port", dac_driver.DEFAULT_PORT, "~dac/port"))
        self.serial_baud_rate = int(get_compatible_param(
            "~dac/serial_baud_rate", dac_driver.DEFAULT_BAUD, "~dac/baud"))
        self.dac_channel_id = int(
            get_compatible_param("~dac/channel_id", dac_driver.DEFAULT_CHANNEL, "~dac/channel")
        )
        self.serial_timeout_s = float(get_compatible_param("~dac/serial_timeout_s", 0.1, "~dac/timeout"))
        self.verify_echo = bool(rospy.get_param("~dac/verify_echo", True))
        self.safe_voltage_v = dac_driver.normalize_voltage(
            get_compatible_param("~dac/safe_voltage_v", 0.0, "~dac/safe_voltage")
        )
        self.zero_on_shutdown = bool(
            rospy.get_param("~dac/zero_on_shutdown", True)
        )
        self.command_timeout_s = float(
            get_compatible_param("~dac/command_timeout_s", 0.0, "~dac/command_timeout")
        )
        self.reconnect_interval_s = float(
            get_compatible_param("~dac/reconnect_interval_s", 1.0, "~dac/reconnect_interval")
        )
        state_topic = rospy.get_param(
            "~dac/state_topic", "/robotic_fish/dac/state"
        )
        service_name = rospy.get_param(
            "~dac/set_voltage_service", "/robotic_fish/dac/set_voltage"
        )

        dac_driver.validate_channel(self.dac_channel_id)
        if self.serial_timeout_s <= 0 or self.reconnect_interval_s <= 0:
            raise ValueError("DAC timeout and reconnect_interval must be positive")
        if self.command_timeout_s < 0:
            raise ValueError("DAC command_timeout must not be negative")

        self.lock = threading.RLock()
        self.dac = None
        self.last_open_attempt_monotonic_ns = 0
        self.last_command_monotonic_ns = 0
        self.watchdog_fired = False
        self.applied_voltage_v = None
        self.error_count = 0
        self.last_error = "not connected"

        self.state_publisher = rospy.Publisher(
            state_topic, DacState, queue_size=10, latch=True
        )
        self.diagnostics_publisher = rospy.Publisher(
            "/diagnostics", DiagnosticArray, queue_size=10
        )
        self.service = rospy.Service(
            service_name, SetDacVoltage, self._set_voltage_service
        )
        self.watchdog_timer = rospy.Timer(
            rospy.Duration(0.1), self._watchdog_callback
        )
        self.diagnostic_timer = rospy.Timer(
            rospy.Duration(1.0), self._diagnostic_callback
        )
        rospy.on_shutdown(self._shutdown)
        self.config_recorder = ConfigRecorder("dac")
        self.config_recorder.publish(dict(
            port=self.serial_port, baud=self.serial_baud_rate, default_channel=self.dac_channel_id,
            timeout=self.serial_timeout_s, verify_echo=self.verify_echo,
            safe_voltage=float(self.safe_voltage_v), zero_on_shutdown=self.zero_on_shutdown,
            command_timeout=self.command_timeout_s, reconnect_interval=self.reconnect_interval_s),
            topics=dict(state=state_topic, service=service_name))

        with self.lock:
            try:
                self._open_locked()
            except (OSError, serial.SerialException) as exc:
                self.last_error = str(exc)
                rospy.logwarn("DAC is not connected yet: %s", exc)

    def _open_locked(self):
        if self.dac is not None and self.dac.is_open:
            return
        now_monotonic_ns = time.monotonic_ns()
        elapsed_s = (now_monotonic_ns - self.last_open_attempt_monotonic_ns) / 1e9
        if self.last_open_attempt_monotonic_ns and elapsed_s < self.reconnect_interval_s:
            raise OSError("DAC reconnect backoff is active")
        self.last_open_attempt_monotonic_ns = now_monotonic_ns
        self.dac = dac_driver.open_dac(self.serial_port, self.serial_baud_rate, self.serial_timeout_s)
        self.last_error = ""
        rospy.loginfo("DAC connected on %s at %d baud", self.serial_port, self.serial_baud_rate)

    def _close_locked(self):
        if self.dac is not None:
            try:
                self.dac.close()
            except Exception:
                pass
            self.dac = None

    def _apply_voltage_locked(self, voltage_v, channel_id, mark_command=True):
        # A rejected request has not changed the hardware or its confirmed state.
        requested_voltage_v = dac_driver.normalize_voltage(voltage_v)
        dac_driver.validate_channel(channel_id)
        if channel_id != self.dac_channel_id:
            raise ValueError("Only the configured common-gain DAC channel is allowed")
        try:
            self._open_locked()
            applied_voltage_v, _, _ = dac_driver.set_voltage(
                self.dac, requested_voltage_v, channel=channel_id, verify_echo=self.verify_echo)
        except (ValueError, OSError, serial.SerialException) as exc:
            self.applied_voltage_v = None
            self._publish_result(voltage_v, float("nan"), False, str(exc))
            self._close_locked()
            raise

        self.applied_voltage_v = float(applied_voltage_v)
        self.last_error = ""
        if mark_command:
            self.last_command_monotonic_ns = time.monotonic_ns()
            self.watchdog_fired = False
        self._publish_result(voltage_v, self.applied_voltage_v, True, "")
        return self.applied_voltage_v

    def _publish_result(self, requested_voltage_v, applied_voltage_v, dac_voltage_valid, error_message):
        msg = DacState()
        msg.timestamp = rospy.Time.now()
        msg.dac_volt_target = float(requested_voltage_v)
        msg.dac_volt = applied_voltage_v
        msg.status_dac_feedback = dac_voltage_valid
        msg.log = error_message
        self.state_publisher.publish(msg)

    def _set_voltage(self, voltage_v, channel_id):
        with self.lock:
            try:
                applied_voltage_v = self._apply_voltage_locked(voltage_v, channel_id)
                return True, applied_voltage_v, "DAC voltage applied"
            except (ValueError, OSError, serial.SerialException) as exc:
                self.error_count += 1
                self.last_error = str(exc)
                rospy.logerr_throttle(2.0, "DAC command failed: %s", exc)
                return False, float("nan"), str(exc)

    def _set_voltage_service(self, request):
        success, applied_voltage_v, message = self._set_voltage(
            request.voltage, request.channel
        )
        return SetDacVoltageResponse(success, applied_voltage_v, message)

    def _watchdog_callback(self, _event):
        with self.lock:
            # A command may have completed while this callback waited for serial IO.
            if self.command_timeout_s <= 0 or self.last_command_monotonic_ns == 0:
                return
            elapsed_s = (time.monotonic_ns() - self.last_command_monotonic_ns) / 1e9
            if elapsed_s <= self.command_timeout_s or self.watchdog_fired:
                return
            try:
                self._apply_voltage_locked(
                    self.safe_voltage_v, self.dac_channel_id, mark_command=False
                )
                self.watchdog_fired = True
                rospy.logwarn(
                    "DAC command watchdog expired; applied safe voltage %.2f V",
                    float(self.safe_voltage_v),
                )
            except (ValueError, OSError, serial.SerialException) as exc:
                self.error_count += 1
                self.last_error = str(exc)
                rospy.logerr_throttle(2.0, "DAC watchdog failed: %s", exc)

    def _diagnostic_callback(self, _event):
        with self.lock:
            status = DiagnosticStatus()
            status.name = "robotic_fish_sonic/dac"
            status.hardware_id = self.serial_port
            if self.dac is None or not self.dac.is_open:
                status.level = DiagnosticStatus.ERROR
                status.message = self.last_error or "DAC disconnected"
            else:
                status.level = DiagnosticStatus.OK
                status.message = "DAC connected"
            status.values = [
                KeyValue("port", self.serial_port),
                KeyValue("baud", str(self.serial_baud_rate)),
                KeyValue("channel", str(self.dac_channel_id)),
                KeyValue(
                    "applied_voltage",
                    "unknown"
                    if self.applied_voltage_v is None
                    else "{:.2f}".format(self.applied_voltage_v),
                ),
                KeyValue("verify_echo", str(self.verify_echo)),
                KeyValue("errors", str(self.error_count)),
            ]
            array = DiagnosticArray()
            array.header.stamp = rospy.Time.now()
            array.status = [status]
            self.diagnostics_publisher.publish(array)

    def _shutdown(self):
        with self.lock:
            if self.zero_on_shutdown:
                try:
                    self._apply_voltage_locked(
                        self.safe_voltage_v,
                        self.dac_channel_id,
                        mark_command=False,
                    )
                    rospy.loginfo(
                        "DAC shutdown safe voltage applied: %.2f V",
                        float(self.safe_voltage_v),
                    )
                except (ValueError, OSError, serial.SerialException) as exc:
                    rospy.logwarn("Could not apply DAC shutdown voltage: %s", exc)
            self._close_locked()


def main():
    rospy.init_node("dac")
    try:
        DacNode()
        rospy.spin()
    except (ValueError, TypeError) as exc:
        rospy.logfatal("Invalid DAC configuration: %s", exc)
        raise


if __name__ == "__main__":
    main()
