"""Serial DAC driver for the 5-byte BCD voltage protocol."""

from decimal import Decimal, InvalidOperation, ROUND_HALF_UP

import serial


DEFAULT_PORT = "/dev/robotic_fish_dac"
DEFAULT_BAUD = 19200
DEFAULT_CHANNEL = 0x01
VOLTAGE_MIN = Decimal("0.00")
VOLTAGE_MAX = Decimal("5.00")
VOLTAGE_RESOLUTION = Decimal("0.01")


class DacEchoError(OSError):
    """The DAC echo did not match the command that was sent."""


def normalize_voltage(value):
    try:
        voltage_v = Decimal(str(value)).quantize(
            VOLTAGE_RESOLUTION, rounding=ROUND_HALF_UP
        )
    except (InvalidOperation, ValueError) as exc:
        raise ValueError("DAC voltage must be numeric") from exc

    if not voltage_v.is_finite():
        raise ValueError("DAC voltage must be finite")
    if not VOLTAGE_MIN <= voltage_v <= VOLTAGE_MAX:
        raise ValueError(
            "DAC voltage must be between {} and {} V".format(
                VOLTAGE_MIN, VOLTAGE_MAX
            )
        )
    return voltage_v


def validate_channel(channel):
    if not isinstance(channel, int) or not 0 <= channel <= 0xFF:
        raise ValueError("DAC channel must be between 0x00 and 0xFF")


def voltage_to_command(value, channel=DEFAULT_CHANNEL):
    validate_channel(channel)
    voltage_v = normalize_voltage(value)
    voltage_hundredths = int(voltage_v * 100)
    whole_volts, fractional_hundredths = divmod(voltage_hundredths, 100)
    decimal_bcd = ((fractional_hundredths // 10) << 4) | (fractional_hundredths % 10)
    return bytes((0x5A, channel, whole_volts, decimal_bcd, 0xA5))


def open_dac(port=DEFAULT_PORT, baudrate=DEFAULT_BAUD, timeout=0.1):
    return serial.Serial(
        port=port,
        baudrate=baudrate,
        bytesize=serial.EIGHTBITS,
        parity=serial.PARITY_NONE,
        stopbits=serial.STOPBITS_ONE,
        timeout=timeout,
        write_timeout=timeout,
    )


def set_voltage(
    dac,
    value,
    channel=DEFAULT_CHANNEL,
    read_echo=False,
    verify_echo=False,
):
    voltage_v = normalize_voltage(value)
    command_bytes = voltage_to_command(voltage_v, channel)
    read_echo = read_echo or verify_echo
    if read_echo:
        dac.reset_input_buffer()
    written_byte_count = dac.write(command_bytes)
    if written_byte_count is not None and written_byte_count != len(command_bytes):
        raise serial.SerialTimeoutException(
            "DAC command wrote only {}/{} bytes".format(written_byte_count, len(command_bytes))
        )
    dac.flush()
    echo_bytes = dac.read(len(command_bytes)) if read_echo else b""
    if verify_echo and echo_bytes != command_bytes:
        raise DacEchoError(
            "DAC echo mismatch: sent={}, received={}".format(
                command_bytes.hex(" ").upper(), echo_bytes.hex(" ").upper() or "none"
            )
        )
    return voltage_v, command_bytes, echo_bytes
