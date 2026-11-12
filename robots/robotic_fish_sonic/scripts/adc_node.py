#!/usr/bin/env python3
"""Publish timestamped ADS1115 conversions with optional transfer calibration."""

from pathlib import Path
import math
import time

import rospkg
import rospy
from diagnostic_msgs.msg import DiagnosticArray, DiagnosticStatus, KeyValue
from robotic_fish_sonic.msg import DacState
from robotic_fish_sonic.runtime_config import ConfigRecorder
from robotic_fish_sonic.parameters import get_compatible_param

from robotic_fish_sonic.adc_mean import RawVoltageMean
from robotic_fish_sonic.adc_calibration import AdcCalibration
from robotic_fish_sonic import adc_driver
from robotic_fish_sonic.msg import AdcSample, AdcSampleArray


class AdcNode:
    def __init__(self):
        self.i2c_bus_id = int(get_compatible_param(
            "~adc/i2c_bus_id", adc_driver.DEFAULT_BUS, "~adc/bus"))
        self.i2c_address = int(get_compatible_param(
            "~adc/i2c_address", adc_driver.DEFAULT_ADDRESS, "~adc/address"))
        self.channel_ids = [int(value) for value in get_compatible_param(
            "~adc/channel_ids", [0, 1, 2], "~adc/channels")]
        self.conversion_rate_sps = int(
            get_compatible_param("~adc/conversion_rate_sps", adc_driver.DEFAULT_DATA_RATE,
                                 "~adc/data_rate")
        )
        self.sample_group_rate_hz = float(get_compatible_param(
            "~adc/sample_group_rate_hz", 20.0, "~adc/channel_rate"))
        self.conversion_timeout_s = float(
            get_compatible_param(
                "~adc/conversion_timeout_s",
                adc_driver.conversion_timeout(self.conversion_rate_sps),
                "~adc/conversion_timeout",
            )
        )
        self.ready_poll_interval_s = float(
            get_compatible_param(
                "~adc/ready_poll_interval_s",
                adc_driver.DEFAULT_READY_POLL_INTERVAL_S,
                "~adc/ready_poll_interval",
            )
        )
        self.reconnect_interval_s = float(
            get_compatible_param("~adc/reconnect_interval_s", 1.0, "~adc/reconnect_interval")
        )
        self.device_id = str(get_compatible_param("~adc/device_id", "ads1115", "~adc/frame_id"))
        if not rospy.get_param("~adc/publish_array", True):
            rospy.logwarn("adc/publish_array is obsolete; ADC now always publishes sample groups")
        self.calibration_enabled = bool(
            rospy.get_param("~adc/calibration/enabled", False)
        )
        self.calibration_required = bool(
            rospy.get_param("~adc/calibration/required", True)
        )
        self.calibration_file = str(
            rospy.get_param("~adc/calibration/file", "")
        )
        samples_topic = rospy.get_param(
            "~adc/samples_topic", "/robotic_fish/adc/samples"
        )
        dac_state_topic = get_compatible_param(
            "~adc/dac_state_topic", "/robotic_fish/dac/state", "~adc/gain_control_topic")

        self._validate_parameters()
        self.raw_voltage_means = {channel: RawVoltageMean(max(.2, 3.0 / self.sample_group_rate_hz))
                          for channel in self.channel_ids}
        self.calibration = None
        self.calibration_path = None
        self.calibration_status = "disabled"
        self.last_calibration_applied = False
        self._configure_calibration()

        self.samples_publisher = rospy.Publisher(
            samples_topic, AdcSampleArray, queue_size=10
        )
        self.diagnostics_publisher = rospy.Publisher(
            "/diagnostics", DiagnosticArray, queue_size=10
        )
        # Replacing this tuple is atomic in CPython, so sampling always observes
        # a consistent validity/value pair from the subscriber callback.
        self.dac_voltage_state = (False, 0.0)
        self.dac_state_subscriber = rospy.Subscriber(
            dac_state_topic,
            DacState,
            self._on_dac_state,
            queue_size=10,
        )
        self.bus = None
        self.sample_sequence = 0
        self.error_count = 0
        self.timeout_count = 0
        self.last_error = "not connected"
        self.last_sample_stamp = rospy.Time(0)
        self.last_conversion_ms = 0.0
        self.last_diagnostic_monotonic_ns = 0
        self.config_recorder = ConfigRecorder("adc")
        # Wire/config keys are a compatibility boundary for existing bag tools.
        self.config_recorder.publish(dict(
            bus_number=self.i2c_bus_id, address=self.i2c_address, channels=self.channel_ids,
            data_rate=self.conversion_rate_sps, channel_rate=self.sample_group_rate_hz,
            timeout=self.conversion_timeout_s, poll_interval=self.ready_poll_interval_s,
            reconnect_interval=self.reconnect_interval_s, frame_id=self.device_id,
            publish_array=True, calibration_enabled=self.calibration_enabled,
            calibration_required=self.calibration_required, calibration_file=self.calibration_file),
            calibration=None if self.calibration is None else self.calibration.document,
            topics=dict(samples=samples_topic, dac_state=dac_state_topic))

    def _on_dac_state(self, msg):
        dac_voltage_v = float(msg.dac_volt)
        if not msg.status_dac_feedback or not math.isfinite(dac_voltage_v):
            self.dac_voltage_state = (False, 0.0)
            rospy.logwarn_throttle(
                5.0, "Ignoring non-finite DAC gain-control voltage"
            )
            return
        self.dac_voltage_state = (True, dac_voltage_v)

    def _validate_parameters(self):
        adc_driver.validate_address(self.i2c_address)
        if not self.channel_ids:
            raise ValueError("ADC channels must not be empty")
        for channel_id in self.channel_ids:
            adc_driver.build_config(channel_id, self.conversion_rate_sps)
        if self.sample_group_rate_hz <= 0:
            raise ValueError("ADC sample_group_rate_hz must be positive")
        adc_driver.validate_timeout(self.conversion_timeout_s)
        adc_driver.validate_timeout(self.ready_poll_interval_s)
        adc_driver.validate_timeout(self.reconnect_interval_s)
        if self.calibration_enabled and self.channel_ids != [0, 1, 2]:
            raise ValueError(
                "Independent ADC calibration requires channels [0, 1, 2] in order"
            )

        minimum_cycle_s = len(self.channel_ids) / float(self.conversion_rate_sps)
        requested_cycle_s = 1.0 / self.sample_group_rate_hz
        if requested_cycle_s < minimum_cycle_s:
            rospy.logwarn(
                "Requested %.1f Hz per ADC channel exceeds the nominal %.1f Hz "
                "limit for %d channels at %d SPS",
                self.sample_group_rate_hz,
                self.conversion_rate_sps / float(len(self.channel_ids)),
                len(self.channel_ids),
                self.conversion_rate_sps,
            )

    def _resolve_calibration_path(self):
        if not self.calibration_file:
            raise ValueError("ADC calibration file must not be empty")
        path = Path(self.calibration_file).expanduser()
        if not path.is_absolute():
            package_path = Path(rospkg.RosPack().get_path("robotic_fish_sonic"))
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
            sample.volt_cali = sample.volt_raw
            sample.diff_cali = float("nan")
            sample.status_cali = False
            sample.cali_id = ""

        if not self.calibration_enabled or self.calibration is None:
            self.last_calibration_applied = False
            return

        try:
            raw_voltages_v = [sample.volt_raw for sample in samples]
            calibrated_voltages_v, _gains = self.calibration.apply(raw_voltages_v)
        except ValueError as exc:
            self.calibration_status = str(exc)
            self.last_calibration_applied = False
            return

        for sample, calibrated_voltage_v in zip(samples, calibrated_voltages_v):
            sample.volt_cali = calibrated_voltage_v
            sample.diff_cali = calibrated_voltage_v - sample.volt_raw
            sample.status_cali = True
            sample.cali_id = self.calibration.calibration_id
        self.calibration_status = "active"
        self.last_calibration_applied = True

    def _open_bus(self):
        self.bus = adc_driver.open_adc(self.i2c_bus_id)
        self.last_error = ""
        rospy.loginfo(
            "ADS1115 connected on /dev/i2c-%d at 0x%02X",
            self.i2c_bus_id,
            self.i2c_address,
        )

    def _close_bus(self):
        for mean in self.raw_voltage_means.values():
            mean.reset()
        if self.bus is not None:
            try:
                self.bus.close()
            except Exception:
                pass
            self.bus = None

    @staticmethod
    def _ros_time_from_anchor(anchor_stamp, anchor_monotonic_ns, conversion_completed_monotonic_ns):
        return anchor_stamp + rospy.Duration.from_sec(
            (conversion_completed_monotonic_ns - anchor_monotonic_ns) / 1e9
        )

    def _sample_channel(self, channel_id):
        anchor_stamp = rospy.Time.now()
        anchor_monotonic_ns = time.monotonic_ns()
        dac_voltage_valid, dac_voltage_v = self.dac_voltage_state
        reading = adc_driver.read_timed_channel(
            self.bus,
            channel_id,
            address=self.i2c_address,
            data_rate=self.conversion_rate_sps,
            timeout=self.conversion_timeout_s,
            poll_interval=self.ready_poll_interval_s,
        )

        sample_stamp = self._ros_time_from_anchor(
            anchor_stamp, anchor_monotonic_ns, reading.completed_ns
        )

        msg = AdcSample()
        msg.serial_num = self.sample_sequence
        msg.timestamp = sample_stamp
        msg.device = self.device_id
        msg.channel_id = reading.channel
        msg.adc_code = reading.raw
        msg.volt_raw = reading.voltage
        msg.dac_volt = dac_voltage_v if dac_voltage_valid else float("nan")
        msg.status_dac_feedback = dac_voltage_valid
        self.sample_sequence = (self.sample_sequence + 1) % (1 << 32)
        self.last_sample_stamp = sample_stamp
        self.last_conversion_ms = (
            reading.completed_ns - reading.started_ns
        ) / 1e6
        return msg

    def _update_means(self, samples):
        for sample in samples:
            sample.volt_raw_mean_1s, sample.mean_1s_ready = self.raw_voltage_means[sample.channel_id].update(
                sample.timestamp.to_sec(), sample.volt_raw,
                sample.status_dac_feedback, sample.dac_volt)

    def _read_sample_group(self):
        """Only a complete, calibrated group reaches the publication boundary."""
        samples = [self._sample_channel(channel_id) for channel_id in self.channel_ids]
        self._update_means(samples)
        self._apply_calibration(samples)
        sample_group = AdcSampleArray()
        sample_group.samples = samples
        return sample_group

    def _publish_diagnostic(self, force=False):
        now_ns = time.monotonic_ns()
        if not force and now_ns - self.last_diagnostic_monotonic_ns < 1_000_000_000:
            return
        self.last_diagnostic_monotonic_ns = now_ns

        status = DiagnosticStatus()
        status.name = "robotic_fish_sonic/adc"
        status.hardware_id = "i2c-{}:0x{:02x}".format(
            self.i2c_bus_id, self.i2c_address
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
            KeyValue("channels", ",".join(str(v) for v in self.channel_ids)),
            KeyValue("data_rate_sps", str(self.conversion_rate_sps)),
            KeyValue("target_channel_rate_hz", str(self.sample_group_rate_hz)),
            KeyValue("last_conversion_ms", "{:.3f}".format(self.last_conversion_ms)),
            KeyValue("errors", str(self.error_count)),
            KeyValue("timeouts", str(self.timeout_count)),
            KeyValue("calibration_enabled", str(self.calibration_enabled)),
            KeyValue(
                "calibration_id",
                "" if self.calibration is None else self.calibration.calibration_id,
            ),
            KeyValue("calibration_status", self.calibration_status),
            KeyValue(
                "gain_control_voltage_v",
                "unknown"
                if not self.dac_voltage_state[0]
                else "{:.2f}".format(self.dac_voltage_state[1]),
            ),
        ]
        array = DiagnosticArray()
        array.header.stamp = rospy.Time.now()
        array.status = [status]
        self.diagnostics_publisher.publish(array)

    def run(self):
        sample_group_period_s = 1.0 / self.sample_group_rate_hz
        next_cycle_monotonic_s = time.monotonic()

        while not rospy.is_shutdown():
            if self.bus is None:
                try:
                    self._open_bus()
                except (OSError, IOError) as exc:
                    self.error_count += 1
                    self.last_error = str(exc)
                    rospy.logerr_throttle(5.0, "Cannot open ADS1115: %s", exc)
                    self._publish_diagnostic(force=True)
                    rospy.sleep(self.reconnect_interval_s)
                    next_cycle_monotonic_s = time.monotonic()
                    continue

            try:
                sample_group = self._read_sample_group()
            except TimeoutError as exc:
                self.timeout_count += 1
                self.error_count += 1
                self.last_error = str(exc)
                rospy.logerr_throttle(5.0, "%s", exc)
                self._close_bus()
                self._publish_diagnostic(force=True)
                rospy.sleep(self.reconnect_interval_s)
                next_cycle_monotonic_s = time.monotonic()
                continue
            except (OSError, IOError) as exc:
                self.error_count += 1
                self.last_error = str(exc)
                rospy.logerr_throttle(5.0, "ADS1115 I2C error: %s", exc)
                self._close_bus()
                self._publish_diagnostic(force=True)
                rospy.sleep(self.reconnect_interval_s)
                next_cycle_monotonic_s = time.monotonic()
                continue

            self.samples_publisher.publish(sample_group)
            self._publish_diagnostic()

            next_cycle_monotonic_s += sample_group_period_s
            delay_s = next_cycle_monotonic_s - time.monotonic()
            if delay_s > 0:
                rospy.sleep(delay_s)
            else:
                next_cycle_monotonic_s = time.monotonic()

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
