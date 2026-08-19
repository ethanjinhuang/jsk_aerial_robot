# robotic_fish_io

ROS 1 hardware interface for the robotic fish ADS1115 ADC and serial DAC.

## ADC timing

The ADS1115 multiplexes its input channels and converts them sequentially. The
node records a separate conversion start, conversion end, and midpoint timestamp
for every `AdcSample`. Samples are published after the three-channel calibration
decision, but their timestamps still describe the individual hardware
conversions accurately. The optional `/robotic_fish/adc/samples` batch retains
the same per-sample timestamps.

Each sample contains both `voltage` (the direct ADS1115 conversion) and
`calibrated_voltage`. Calibration is loaded from the JSON file selected under
`adc.calibration.file` in `config/io.yaml`. The packaged default is the validated
three-channel independent transfer table
`adc_independent_transfer_20260818_183014_calbri01_raw.json`.

ADC0, ADC1, and ADC2 each use their own raw ADC voltage to query a piecewise
linear curve. Calibration has no runtime dependency on the DAC voltage. If any
channel is outside its observed input range (including the configured endpoint
tolerance), the complete three-channel group falls back to raw voltages and
`calibration_applied` is false. The node never extrapolates or mixes calibrated
and raw values within a group.

Select a packaged calibration file relative to the package `config` directory:

```yaml
adc:
  calibration:
    enabled: true
    required: true
    file: calibration/adc_independent_transfer_20260818_183014_calbri01_raw.json
```

An absolute path is also accepted. With `required: true`, a missing, malformed,
DAC-dependent, or otherwise incompatible calibration file prevents the ADC node
from starting instead of silently using an invalid calibration.

Default ADC configuration:

- I2C bus: `0`
- address: `0x48`
- channels: `0, 1, 2`
- ADS1115 data rate: `128 SPS`
- target update rate per channel: `20 Hz`

## Starting the nodes

```bash
roslaunch robotic_fish_io sensor_io.launch
```

ADC only:

```bash
roslaunch robotic_fish_io sensor_io.launch enable_dac:=false
```

DAC only:

```bash
roslaunch robotic_fish_io sensor_io.launch enable_adc:=false
```

Inspect ADC samples:

```bash
rostopic echo /robotic_fish/adc/sample
rostopic hz /robotic_fish/adc/sample
```

Set DAC channel 1 to 1.25 V:

```bash
rosservice call /robotic_fish/dac/set_voltage "channel: 1
voltage: 1.25"
```

The DAC defaults to `/dev/robotic_fish_dac`. Create a stable udev symlink for
the actual USB serial adapter instead of relying on `/dev/ttyUSB0`, which may
also be used by the spinal bridge.

## Tests

```bash
python3 -m unittest discover -s test -p 'test_*.py' -v
```
