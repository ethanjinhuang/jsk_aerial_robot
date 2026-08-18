#!/usr/bin/env python3
"""Hardware-independent regression tests for robotic_fish_io drivers."""

from decimal import Decimal
from pathlib import Path
import sys
import unittest

import serial


PACKAGE_ROOT = Path(__file__).resolve().parents[1]
sys.path.insert(0, str(PACKAGE_ROOT / "src"))

from robotic_fish_io import adc_driver, dac_driver  # noqa: E402


class FakeAdcBus:
    def __init__(self, raw_by_channel=None, ready=True, short_read=False):
        self.raw_by_channel = raw_by_channel or {
            0: 1000,
            1: 2000,
            2: 0xFC18,
        }
        self.ready = ready
        self.short_read = short_read
        self.channel = 0
        self.configs = []

    def write_i2c_block_data(self, address, register, data):
        config = (data[0] << 8) | data[1]
        self.configs.append((address, register, config))
        self.channel = ((config >> 12) & 0x7) - 4

    def read_i2c_block_data(self, _address, register, _length):
        if self.short_read:
            return [0x80]
        if register == adc_driver.REG_CONFIG:
            return [0x80 if self.ready else 0x00, 0x00]
        raw = self.raw_by_channel[self.channel]
        return [(raw >> 8) & 0xFF, raw & 0xFF]


class FakeDac:
    def __init__(self, echo=None, written=None):
        self.echo = echo
        self.written = written
        self.command = None
        self.input_reset = False

    def reset_input_buffer(self):
        self.input_reset = True

    def write(self, command):
        self.command = command
        return len(command) if self.written is None else self.written

    def flush(self):
        pass

    def read(self, size):
        if self.echo is None:
            return self.command[:size]
        return self.echo[:size]


class AdcDriverTests(unittest.TestCase):
    def test_configuration_words(self):
        self.assertEqual(adc_driver.build_config(0, 128), 0xC383)
        self.assertEqual(adc_driver.build_config(1, 128), 0xD383)
        self.assertEqual(adc_driver.build_config(2, 128), 0xE383)
        self.assertEqual(adc_driver.build_config(0, 860), 0xC3E3)

    def test_timeout_scales_with_data_rate(self):
        self.assertEqual(adc_driver.conversion_timeout(8), 0.25)
        self.assertEqual(adc_driver.conversion_timeout(16), 0.125)
        self.assertEqual(adc_driver.conversion_timeout(128), 0.1)

    def test_three_channels_and_signed_conversion(self):
        bus = FakeAdcBus()
        values = adc_driver.read_channels(bus)
        self.assertEqual([value[0] for value in values], [1000, 2000, -1000])
        self.assertEqual(
            [entry[2] for entry in bus.configs],
            [0xC383, 0xD383, 0xE383],
        )

    def test_each_conversion_has_independent_timing_bounds(self):
        bus = FakeAdcBus()
        timestamps = iter((1_000_000, 9_000_000, 20_000_000, 28_000_000))

        first = adc_driver.read_timed_channel(
            bus, 0, clock_ns=lambda: next(timestamps)
        )
        second = adc_driver.read_timed_channel(
            bus, 1, clock_ns=lambda: next(timestamps)
        )

        self.assertEqual((first.started_ns, first.completed_ns), (1_000_000, 9_000_000))
        self.assertEqual(
            (second.started_ns, second.completed_ns),
            (20_000_000, 28_000_000),
        )
        self.assertLess(first.completed_ns, second.started_ns)

    def test_invalid_configuration_is_rejected(self):
        with self.assertRaises(ValueError):
            adc_driver.build_config(4)
        with self.assertRaises(ValueError):
            adc_driver.build_config(0, 100)
        with self.assertRaises(ValueError):
            adc_driver.read_channel(FakeAdcBus(), 0, address=0x80)
        with self.assertRaises(ValueError):
            adc_driver.read_channel(FakeAdcBus(), 0, timeout=float("nan"))

    def test_short_read_and_conversion_timeout_are_reported(self):
        with self.assertRaises(OSError):
            adc_driver.read_channel(FakeAdcBus(short_read=True), 0)
        with self.assertRaises(TimeoutError):
            adc_driver.read_channel(FakeAdcBus(ready=False), 0, timeout=0.002)


class DacDriverTests(unittest.TestCase):
    def test_voltage_normalization_and_bcd_commands(self):
        expected = {
            "0.00": "5A 01 00 00 A5",
            "0.50": "5A 01 00 50 A5",
            "0.99": "5A 01 00 99 A5",
            "1.00": "5A 01 01 00 A5",
            "2.50": "5A 01 02 50 A5",
            "5.00": "5A 01 05 00 A5",
        }
        for value, expected_hex in expected.items():
            with self.subTest(value=value):
                command = dac_driver.voltage_to_command(value)
                self.assertEqual(command.hex(" ").upper(), expected_hex)
        self.assertEqual(dac_driver.normalize_voltage("2.345"), Decimal("2.35"))

    def test_invalid_voltage_and_channel_are_rejected(self):
        for value in ("invalid", "NaN", "-0.01", "5.01"):
            with self.subTest(value=value), self.assertRaises(ValueError):
                dac_driver.normalize_voltage(value)
        with self.assertRaises(ValueError):
            dac_driver.voltage_to_command(1, channel=256)

    def test_echo_can_be_verified(self):
        port = FakeDac()
        voltage, command, echo = dac_driver.set_voltage(
            port, 0.99, verify_echo=True
        )
        self.assertEqual(voltage, Decimal("0.99"))
        self.assertEqual(echo, command)
        self.assertTrue(port.input_reset)

        with self.assertRaises(dac_driver.DacEchoError):
            dac_driver.set_voltage(
                FakeDac(echo=b"\xFF" * 5), 1, verify_echo=True
            )

    def test_short_serial_write_is_rejected(self):
        with self.assertRaises(serial.SerialTimeoutException):
            dac_driver.set_voltage(FakeDac(written=3), 1)


if __name__ == "__main__":
    unittest.main()
