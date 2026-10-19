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
        voltage = Decimal(str(value)).quantize(
            VOLTAGE_RESOLUTION, rounding=ROUND_HALF_UP
        )
    except (InvalidOperation, ValueError) as exc:
        raise ValueError("DAC voltage must be numeric") from exc

    if not voltage.is_finite():
        raise ValueError("DAC voltage must be finite")
    if not VOLTAGE_MIN <= voltage <= VOLTAGE_MAX:
        raise ValueError(
            "DAC voltage must be between {} and {} V".format(
                VOLTAGE_MIN, VOLTAGE_MAX
            )
        )
    return voltage


def validate_channel(channel):
    if not isinstance(channel, int) or not 0 <= channel <= 0xFF:
        raise ValueError("DAC channel must be between 0x00 and 0xFF")


def voltage_to_command(value, channel=DEFAULT_CHANNEL):
    validate_channel(channel)
    voltage = normalize_voltage(value)
    hundredths = int(voltage * 100)
    integer_part, decimal_part = divmod(hundredths, 100)
    decimal_bcd = ((decimal_part // 10) << 4) | (decimal_part % 10)
    return bytes((0x5A, channel, integer_part, decimal_bcd, 0xA5))


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
    voltage = normalize_voltage(value)
    command = voltage_to_command(voltage, channel)
    read_echo = read_echo or verify_echo
    if read_echo:
        dac.reset_input_buffer()
    written = dac.write(command)
    if written is not None and written != len(command):
        raise serial.SerialTimeoutException(
            "DAC command wrote only {}/{} bytes".format(written, len(command))
        )
    dac.flush()
    echo = dac.read(len(command)) if read_echo else b""
    if verify_echo and echo != command:
        raise DacEchoError(
            "DAC echo mismatch: sent={}, received={}".format(
                command.hex(" ").upper(), echo.hex(" ").upper() or "none"
            )
        )
    return voltage, command, echo
