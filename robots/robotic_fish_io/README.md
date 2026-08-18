# robotic_fish_io

ROS 1 hardware interface for the robotic fish ADS1115 ADC and serial DAC.

## ADC timing

The ADS1115 multiplexes its input channels and converts them sequentially. The
node therefore publishes every completed conversion immediately on
`/robotic_fish/adc/sample`. Each `AdcSample` contains its own conversion start,
conversion end, and midpoint sample timestamp. The optional
`/robotic_fish/adc/samples` batch retains those per-sample timestamps.

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
