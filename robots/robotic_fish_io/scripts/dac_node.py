#!/usr/bin/env python3
"""Expose safe serial DAC control through a ROS service and command topic."""

import threading
import time

import rospy
import serial
from diagnostic_msgs.msg import DiagnosticArray, DiagnosticStatus, KeyValue
from std_msgs.msg import Float32

from robotic_fish_io import dac_driver
from robotic_fish_io.srv import SetDacVoltage, SetDacVoltageResponse


class DacNode:
    def __init__(self):
        self.port = str(rospy.get_param("~dac/port", dac_driver.DEFAULT_PORT))
        self.baud = int(rospy.get_param("~dac/baud", dac_driver.DEFAULT_BAUD))
        self.default_channel = int(
            rospy.get_param("~dac/channel", dac_driver.DEFAULT_CHANNEL)
        )
        self.timeout = float(rospy.get_param("~dac/timeout", 0.1))
        self.verify_echo = bool(rospy.get_param("~dac/verify_echo", True))
        self.safe_voltage = dac_driver.normalize_voltage(
            rospy.get_param("~dac/safe_voltage", 0.0)
        )
        self.zero_on_shutdown = bool(
            rospy.get_param("~dac/zero_on_shutdown", True)
        )
        self.command_timeout = float(
            rospy.get_param("~dac/command_timeout", 0.0)
        )
        self.reconnect_interval = float(
            rospy.get_param("~dac/reconnect_interval", 1.0)
        )
        command_topic = rospy.get_param(
            "~dac/command_topic", "/robotic_fish/dac/command"
        )
        state_topic = rospy.get_param(
            "~dac/state_topic", "/robotic_fish/dac/state"
        )
        service_name = rospy.get_param(
            "~dac/set_voltage_service", "/robotic_fish/dac/set_voltage"
        )

        dac_driver.validate_channel(self.default_channel)
        if self.timeout <= 0 or self.reconnect_interval <= 0:
            raise ValueError("DAC timeout and reconnect_interval must be positive")
        if self.command_timeout < 0:
            raise ValueError("DAC command_timeout must not be negative")

        self.lock = threading.RLock()
        self.dac = None
        self.last_open_attempt_ns = 0
        self.last_command_ns = 0
        self.watchdog_fired = False
        self.applied_voltage = None
        self.error_count = 0
        self.last_error = "not connected"

        self.state_pub = rospy.Publisher(
            state_topic, Float32, queue_size=10, latch=True
        )
        self.diagnostic_pub = rospy.Publisher(
            "/diagnostics", DiagnosticArray, queue_size=10
        )
        self.command_sub = rospy.Subscriber(
            command_topic, Float32, self._command_callback, queue_size=10
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

        with self.lock:
            try:
                self._open_locked()
            except (OSError, serial.SerialException) as exc:
                self.last_error = str(exc)
                rospy.logwarn("DAC is not connected yet: %s", exc)

    def _open_locked(self):
        if self.dac is not None and self.dac.is_open:
            return
        now_ns = time.monotonic_ns()
        elapsed = (now_ns - self.last_open_attempt_ns) / 1e9
        if self.last_open_attempt_ns and elapsed < self.reconnect_interval:
            raise OSError("DAC reconnect backoff is active")
        self.last_open_attempt_ns = now_ns
        self.dac = dac_driver.open_dac(self.port, self.baud, self.timeout)
        self.last_error = ""
        rospy.loginfo("DAC connected on %s at %d baud", self.port, self.baud)

    def _close_locked(self):
        if self.dac is not None:
            try:
                self.dac.close()
            except Exception:
                pass
            self.dac = None

    def _apply_voltage_locked(self, voltage, channel, mark_command=True):
        normalized = dac_driver.normalize_voltage(voltage)
        dac_driver.validate_channel(channel)
        self._open_locked()
        try:
            applied, _, _ = dac_driver.set_voltage(
                self.dac,
                normalized,
                channel=channel,
                verify_echo=self.verify_echo,
            )
        except (OSError, serial.SerialException):
            self._close_locked()
            raise

        self.applied_voltage = float(applied)
        self.last_error = ""
        if mark_command:
            self.last_command_ns = time.monotonic_ns()
            self.watchdog_fired = False
        self.state_pub.publish(Float32(data=self.applied_voltage))
        return self.applied_voltage

    def _set_voltage(self, voltage, channel):
        with self.lock:
            try:
                applied = self._apply_voltage_locked(voltage, channel)
                return True, applied, "DAC voltage applied"
            except (ValueError, OSError, serial.SerialException) as exc:
                self.error_count += 1
                self.last_error = str(exc)
                rospy.logerr_throttle(2.0, "DAC command failed: %s", exc)
                return False, 0.0, str(exc)

    def _set_voltage_service(self, request):
        success, applied, message = self._set_voltage(
            request.voltage, request.channel
        )
        return SetDacVoltageResponse(success, applied, message)

    def _command_callback(self, msg):
        self._set_voltage(msg.data, self.default_channel)

    def _watchdog_callback(self, _event):
        if self.command_timeout <= 0 or self.last_command_ns == 0:
            return
        elapsed = (time.monotonic_ns() - self.last_command_ns) / 1e9
        if elapsed <= self.command_timeout or self.watchdog_fired:
            return
        with self.lock:
            try:
                self._apply_voltage_locked(
                    self.safe_voltage, self.default_channel, mark_command=False
                )
                self.watchdog_fired = True
                rospy.logwarn(
                    "DAC command watchdog expired; applied safe voltage %.2f V",
                    float(self.safe_voltage),
                )
            except (ValueError, OSError, serial.SerialException) as exc:
                self.error_count += 1
                self.last_error = str(exc)
                rospy.logerr_throttle(2.0, "DAC watchdog failed: %s", exc)

    def _diagnostic_callback(self, _event):
        with self.lock:
            status = DiagnosticStatus()
            status.name = "robotic_fish_io/dac"
            status.hardware_id = self.port
            if self.dac is None or not self.dac.is_open:
                status.level = DiagnosticStatus.ERROR
                status.message = self.last_error or "DAC disconnected"
            else:
                status.level = DiagnosticStatus.OK
                status.message = "DAC connected"
            status.values = [
                KeyValue("port", self.port),
                KeyValue("baud", str(self.baud)),
                KeyValue("channel", str(self.default_channel)),
                KeyValue(
                    "applied_voltage",
                    "unknown"
                    if self.applied_voltage is None
                    else "{:.2f}".format(self.applied_voltage),
                ),
                KeyValue("verify_echo", str(self.verify_echo)),
                KeyValue("errors", str(self.error_count)),
            ]
            array = DiagnosticArray()
            array.header.stamp = rospy.Time.now()
            array.status = [status]
            self.diagnostic_pub.publish(array)

    def _shutdown(self):
        with self.lock:
            if self.zero_on_shutdown:
                try:
                    self._apply_voltage_locked(
                        self.safe_voltage,
                        self.default_channel,
                        mark_command=False,
                    )
                    rospy.loginfo(
                        "DAC shutdown safe voltage applied: %.2f V",
                        float(self.safe_voltage),
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
