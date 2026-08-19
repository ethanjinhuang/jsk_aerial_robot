#!/usr/bin/env python3
"""Publish timestamped ADS1115 conversions with optional transfer calibration."""

from pathlib import Path
import time

import rospkg
import rospy
from diagnostic_msgs.msg import DiagnosticArray, DiagnosticStatus, KeyValue

from robotic_fish_io.adc_calibration import AdcCalibration
from robotic_fish_io import adc_driver
from robotic_fish_io.msg import AdcSample, AdcSampleArray


class AdcNode:
    def __init__(self):
        self.bus_number = int(rospy.get_param("~adc/bus", adc_driver.DEFAULT_BUS))
        self.address = int(rospy.get_param("~adc/address", adc_driver.DEFAULT_ADDRESS))
        self.channels = [int(value) for value in rospy.get_param("~adc/channels", [0, 1, 2])]
        self.data_rate = int(
            rospy.get_param("~adc/data_rate", adc_driver.DEFAULT_DATA_RATE)
        )
        self.channel_rate = float(rospy.get_param("~adc/channel_rate", 20.0))
        self.timeout = float(
            rospy.get_param(
                "~adc/conversion_timeout",
                adc_driver.conversion_timeout(self.data_rate),
            )
        )
        self.poll_interval = float(
            rospy.get_param(
                "~adc/ready_poll_interval",
                adc_driver.DEFAULT_READY_POLL_INTERVAL_S,
            )
        )
        self.reconnect_interval = float(
            rospy.get_param("~adc/reconnect_interval", 1.0)
        )
        self.frame_id = str(rospy.get_param("~adc/frame_id", "ads1115"))
        self.publish_array = bool(rospy.get_param("~adc/publish_array", True))
        self.calibration_enabled = bool(
            rospy.get_param("~adc/calibration/enabled", False)
        )
        self.calibration_required = bool(
            rospy.get_param("~adc/calibration/required", True)
        )
        self.calibration_file = str(
            rospy.get_param("~adc/calibration/file", "")
        )
        sample_topic = rospy.get_param(
            "~adc/sample_topic", "/robotic_fish/adc/sample"
        )
        samples_topic = rospy.get_param(
            "~adc/samples_topic", "/robotic_fish/adc/samples"
        )

        self._validate_parameters()
        self.calibration = None
        self.calibration_path = None
        self.calibration_status = "disabled"
        self.last_calibration_applied = False
        self._configure_calibration()

        self.sample_pub = rospy.Publisher(sample_topic, AdcSample, queue_size=50)
        self.samples_pub = rospy.Publisher(
            samples_topic, AdcSampleArray, queue_size=10
        )
        self.diagnostic_pub = rospy.Publisher(
            "/diagnostics", DiagnosticArray, queue_size=10
        )
        self.bus = None
        self.sample_sequence = 0
        self.error_count = 0
        self.timeout_count = 0
        self.last_error = "not connected"
        self.last_sample_stamp = rospy.Time(0)
        self.last_conversion_ms = 0.0
        self.last_diagnostic_ns = 0

    def _validate_parameters(self):
        adc_driver.validate_address(self.address)
        if not self.channels:
            raise ValueError("ADC channels must not be empty")
        for channel in self.channels:
            adc_driver.build_config(channel, self.data_rate)
        if self.channel_rate <= 0:
            raise ValueError("ADC channel_rate must be positive")
        adc_driver.validate_timeout(self.timeout)
        adc_driver.validate_timeout(self.poll_interval)
        adc_driver.validate_timeout(self.reconnect_interval)
        if self.calibration_enabled and self.channels != [0, 1, 2]:
            raise ValueError(
                "Independent ADC calibration requires channels [0, 1, 2] in order"
            )

        minimum_cycle = len(self.channels) / float(self.data_rate)
        requested_cycle = 1.0 / self.channel_rate
        if requested_cycle < minimum_cycle:
            rospy.logwarn(
                "Requested %.1f Hz per ADC channel exceeds the nominal %.1f Hz "
                "limit for %d channels at %d SPS",
                self.channel_rate,
                self.data_rate / float(len(self.channels)),
                len(self.channels),
                self.data_rate,
            )

    def _resolve_calibration_path(self):
        if not self.calibration_file:
            raise ValueError("ADC calibration file must not be empty")
        path = Path(self.calibration_file).expanduser()
        if not path.is_absolute():
            package_path = Path(rospkg.RosPack().get_path("robotic_fish_io"))
            path = package_path / "config" / path
        return path.resolve()

    def _configure_calibration(self):
        if not self.calibration_enabled:
            return
        try:
            self.calibration_path = self._resolve_calibration_path()
            self.calibration = AdcCalibration.load(self.calibration_path)
        except (OSError, ValueError, rospkg.ResourceNotFound) as exc:
            self.calibration_status = "load failed: {}".format(exc)
            if self.calibration_required:
                raise ValueError(self.calibration_status) from exc
            rospy.logerr("ADC calibration disabled: %s", exc)
            self.calibration_enabled = False
            return

        self.calibration_status = "ready"
        rospy.loginfo(
            "Loaded DAC-independent ADC calibration %s from %s",
            self.calibration.calibration_id,
            self.calibration_path,
        )

    def _apply_calibration(self, samples):
        for sample in samples:
            sample.calibrated_voltage = sample.voltage
            sample.calibration_gain = 1.0
            sample.calibration_applied = False
            sample.calibration_id = ""

        if not self.calibration_enabled or self.calibration is None:
            self.last_calibration_applied = False
            return

        try:
            ordered_voltages = [sample.voltage for sample in samples]
            calibrated, gains = self.calibration.apply(ordered_voltages)
        except ValueError as exc:
            self.calibration_status = str(exc)
            self.last_calibration_applied = False
            return

        for sample, calibrated_voltage, gain in zip(samples, calibrated, gains):
            sample.calibrated_voltage = calibrated_voltage
            sample.calibration_gain = gain
            sample.calibration_applied = True
            sample.calibration_id = self.calibration.calibration_id
        self.calibration_status = "active"
        self.last_calibration_applied = True

    def _open_bus(self):
        self.bus = adc_driver.open_adc(self.bus_number)
        self.last_error = ""
        rospy.loginfo(
            "ADS1115 connected on /dev/i2c-%d at 0x%02X",
            self.bus_number,
            self.address,
        )

    def _close_bus(self):
        if self.bus is not None:
            try:
                self.bus.close()
            except Exception:
                pass
            self.bus = None

    @staticmethod
    def _ros_time_from_anchor(anchor_ros, anchor_ns, timestamp_ns):
        return anchor_ros + rospy.Duration.from_sec(
            (timestamp_ns - anchor_ns) / 1e9
        )

    def _sample_channel(self, channel):
        anchor_ros = rospy.Time.now()
        anchor_ns = time.monotonic_ns()
        reading = adc_driver.read_timed_channel(
            self.bus,
            channel,
            address=self.address,
            data_rate=self.data_rate,
            timeout=self.timeout,
            poll_interval=self.poll_interval,
        )

        start = self._ros_time_from_anchor(
            anchor_ros, anchor_ns, reading.started_ns
        )
        end = self._ros_time_from_anchor(
            anchor_ros, anchor_ns, reading.completed_ns
        )
        midpoint = start + rospy.Duration.from_sec(
            (reading.completed_ns - reading.started_ns) / 2e9
        )

        msg = AdcSample()
        msg.header.seq = self.sample_sequence
        msg.header.stamp = midpoint
        msg.header.frame_id = self.frame_id
        msg.channel = reading.channel
        msg.raw = reading.raw
        msg.voltage = reading.voltage
        msg.conversion_start = start
        msg.conversion_end = end
        self.sample_sequence += 1
        self.last_sample_stamp = midpoint
        self.last_conversion_ms = (
            reading.completed_ns - reading.started_ns
        ) / 1e6
        return msg

    def _publish_diagnostic(self, force=False):
        now_ns = time.monotonic_ns()
        if not force and now_ns - self.last_diagnostic_ns < 1_000_000_000:
            return
        self.last_diagnostic_ns = now_ns

        status = DiagnosticStatus()
        status.name = "robotic_fish_io/adc"
        status.hardware_id = "i2c-{}:0x{:02x}".format(
            self.bus_number, self.address
        )
        if self.bus is None:
            status.level = DiagnosticStatus.ERROR
            status.message = self.last_error or "ADC disconnected"
        elif self.calibration_enabled and not self.last_calibration_applied:
            status.level = DiagnosticStatus.WARN
            status.message = "ADC sampling; calibration inactive: {}".format(
                self.calibration_status
            )
        else:
            status.level = DiagnosticStatus.OK
            status.message = "ADC sampling"
        status.values = [
            KeyValue("channels", ",".join(str(v) for v in self.channels)),
            KeyValue("data_rate_sps", str(self.data_rate)),
            KeyValue("target_channel_rate_hz", str(self.channel_rate)),
            KeyValue("last_conversion_ms", "{:.3f}".format(self.last_conversion_ms)),
            KeyValue("errors", str(self.error_count)),
            KeyValue("timeouts", str(self.timeout_count)),
            KeyValue("calibration_enabled", str(self.calibration_enabled)),
            KeyValue(
                "calibration_id",
                "" if self.calibration is None else self.calibration.calibration_id,
            ),
            KeyValue("calibration_status", self.calibration_status),
        ]
        array = DiagnosticArray()
        array.header.stamp = rospy.Time.now()
        array.status = [status]
        self.diagnostic_pub.publish(array)

    def run(self):
        cycle_period = 1.0 / self.channel_rate
        next_cycle = time.monotonic()

        while not rospy.is_shutdown():
            if self.bus is None:
                try:
                    self._open_bus()
                except (OSError, IOError) as exc:
                    self.error_count += 1
                    self.last_error = str(exc)
                    rospy.logerr_throttle(5.0, "Cannot open ADS1115: %s", exc)
                    self._publish_diagnostic(force=True)
                    rospy.sleep(self.reconnect_interval)
                    next_cycle = time.monotonic()
                    continue

            samples = []
            try:
                for channel in self.channels:
                    sample = self._sample_channel(channel)
                    samples.append(sample)
                self._apply_calibration(samples)
                for sample in samples:
                    self.sample_pub.publish(sample)
            except TimeoutError as exc:
                self.timeout_count += 1
                self.error_count += 1
                self.last_error = str(exc)
                rospy.logerr_throttle(5.0, "%s", exc)
                self._close_bus()
                self._publish_diagnostic(force=True)
                rospy.sleep(self.reconnect_interval)
                next_cycle = time.monotonic()
                continue
            except (OSError, IOError) as exc:
                self.error_count += 1
                self.last_error = str(exc)
                rospy.logerr_throttle(5.0, "ADS1115 I2C error: %s", exc)
                self._close_bus()
                self._publish_diagnostic(force=True)
                rospy.sleep(self.reconnect_interval)
                next_cycle = time.monotonic()
                continue

            if self.publish_array:
                batch = AdcSampleArray()
                batch.header.stamp = rospy.Time.now()
                batch.header.frame_id = self.frame_id
                batch.samples = samples
                self.samples_pub.publish(batch)
            self._publish_diagnostic()

            next_cycle += cycle_period
            delay = next_cycle - time.monotonic()
            if delay > 0:
                rospy.sleep(delay)
            else:
                next_cycle = time.monotonic()

        self._close_bus()


def main():
    rospy.init_node("adc")
    try:
        AdcNode().run()
    except (ValueError, TypeError) as exc:
        rospy.logfatal("Invalid ADC configuration: %s", exc)
        raise


if __name__ == "__main__":
    main()
