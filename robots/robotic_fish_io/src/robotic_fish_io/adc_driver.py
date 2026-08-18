"""ADS1115 I2C driver with per-conversion monotonic timing."""

from collections import namedtuple
import math
import time

from smbus2 import SMBus


DEFAULT_BUS = 0
DEFAULT_ADDRESS = 0x48
DEFAULT_DATA_RATE = 128
DEFAULT_CHANNELS = (0, 1, 2)
DEFAULT_READY_POLL_INTERVAL_S = 0.001
MIN_CONVERSION_TIMEOUT_S = 0.1
CONVERSION_TIMEOUT_CYCLES = 2.0

REG_CONVERSION = 0x00
REG_CONFIG = 0x01

MUX_BITS_BY_CHANNEL = {
    0: 0b100,
    1: 0b101,
    2: 0b110,
    3: 0b111,
}
DATA_RATE_BITS = {
    8: 0b000,
    16: 0b001,
    32: 0b010,
    64: 0b011,
    128: 0b100,
    250: 0b101,
    475: 0b110,
    860: 0b111,
}
FULL_SCALE_V = 4.096
LSB_V = FULL_SCALE_V / 32768.0

AdcReading = namedtuple(
    "AdcReading", ("channel", "raw", "voltage", "started_ns", "completed_ns")
)


def validate_address(address):
    if not isinstance(address, int) or not 0 <= address <= 0x7F:
        raise ValueError("ADC address must be a valid 7-bit I2C address")


def validate_timeout(timeout):
    if (
        not isinstance(timeout, (int, float))
        or not math.isfinite(timeout)
        or timeout <= 0
    ):
        raise ValueError("ADC conversion timeout must be a positive finite value")


def build_config(channel, data_rate=DEFAULT_DATA_RATE):
    """Build a single-ended, single-shot ADS1115 configuration word."""
    if channel not in MUX_BITS_BY_CHANNEL:
        raise ValueError("ADC channel must be between 0 and 3")
    if data_rate not in DATA_RATE_BITS:
        choices = ", ".join(str(value) for value in DATA_RATE_BITS)
        raise ValueError("ADC data rate must be one of: {} SPS".format(choices))

    return (
        0x8000
        | (MUX_BITS_BY_CHANNEL[channel] << 12)
        | (0b001 << 9)  # PGA: +/-4.096 V
        | (1 << 8)  # single-shot mode
        | (DATA_RATE_BITS[data_rate] << 5)
        | 0x0003  # comparator disabled
    )


def conversion_timeout(data_rate=DEFAULT_DATA_RATE):
    if data_rate not in DATA_RATE_BITS:
        choices = ", ".join(str(value) for value in DATA_RATE_BITS)
        raise ValueError("ADC data rate must be one of: {} SPS".format(choices))
    return max(MIN_CONVERSION_TIMEOUT_S, CONVERSION_TIMEOUT_CYCLES / data_rate)


def open_adc(bus_number=DEFAULT_BUS):
    return SMBus(bus_number)


def read_register(bus, address, register):
    validate_address(address)
    data = bus.read_i2c_block_data(address, register, 2)
    if len(data) != 2:
        raise OSError(
            "ADC register 0x{:02X} returned {} bytes instead of 2".format(
                register, len(data)
            )
        )
    return (data[0] << 8) | data[1]


def start_conversion(bus, channel, address=DEFAULT_ADDRESS, data_rate=DEFAULT_DATA_RATE):
    """Select a channel and start one ADS1115 conversion."""
    validate_address(address)
    config = build_config(channel, data_rate)
    bus.write_i2c_block_data(
        address,
        REG_CONFIG,
        [(config >> 8) & 0xFF, config & 0xFF],
    )


def conversion_ready(bus, address=DEFAULT_ADDRESS):
    return bool(read_register(bus, address, REG_CONFIG) & 0x8000)


def wait_conversion_ready(
    bus,
    channel,
    address=DEFAULT_ADDRESS,
    data_rate=DEFAULT_DATA_RATE,
    timeout=None,
    poll_interval=DEFAULT_READY_POLL_INTERVAL_S,
):
    if timeout is None:
        timeout = conversion_timeout(data_rate)
    validate_timeout(timeout)
    validate_timeout(poll_interval)

    deadline = time.monotonic() + timeout
    while time.monotonic() < deadline:
        if conversion_ready(bus, address):
            return
        time.sleep(poll_interval)
    raise TimeoutError("AIN{} ADC conversion timed out".format(channel))


def read_conversion(bus, address=DEFAULT_ADDRESS):
    raw = read_register(bus, address, REG_CONVERSION)
    if raw & 0x8000:
        raw -= 1 << 16
    return raw, raw * LSB_V


def read_timed_channel(
    bus,
    channel,
    address=DEFAULT_ADDRESS,
    data_rate=DEFAULT_DATA_RATE,
    timeout=None,
    poll_interval=DEFAULT_READY_POLL_INTERVAL_S,
    clock_ns=time.monotonic_ns,
):
    """Read one channel and retain separate timing bounds for that conversion."""
    started_ns = clock_ns()
    start_conversion(bus, channel, address=address, data_rate=data_rate)
    wait_conversion_ready(
        bus,
        channel,
        address=address,
        data_rate=data_rate,
        timeout=timeout,
        poll_interval=poll_interval,
    )
    raw, voltage = read_conversion(bus, address=address)
    completed_ns = clock_ns()
    return AdcReading(channel, raw, voltage, started_ns, completed_ns)


def read_channel(
    bus,
    channel,
    address=DEFAULT_ADDRESS,
    data_rate=DEFAULT_DATA_RATE,
    timeout=None,
    poll_interval=DEFAULT_READY_POLL_INTERVAL_S,
):
    reading = read_timed_channel(
        bus,
        channel,
        address=address,
        data_rate=data_rate,
        timeout=timeout,
        poll_interval=poll_interval,
    )
    return reading.raw, reading.voltage


def read_channels(
    bus,
    channels=DEFAULT_CHANNELS,
    address=DEFAULT_ADDRESS,
    data_rate=DEFAULT_DATA_RATE,
    timeout=None,
    poll_interval=DEFAULT_READY_POLL_INTERVAL_S,
):
    return [
        read_channel(
            bus,
            channel,
            address=address,
            data_rate=data_rate,
            timeout=timeout,
            poll_interval=poll_interval,
        )
        for channel in channels
    ]
