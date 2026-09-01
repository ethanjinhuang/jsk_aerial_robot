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
`adc_independent_transfer_20260901_171408_calbri05_raw.json`.

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
    file: calibration/adc_independent_transfer_20260901_171408_calbri05_raw.json
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

## Closed-loop AGC

The optional AGC node reads complete `/robotic_fish/adc/samples` batches and
always makes its decision from the maximum of the three raw `voltage` fields.
It deliberately ignores `calibrated_voltage`, so calibration or filtering does
not alter clipping protection or gain control.

The default configuration keeps the maximum raw ADC voltage between 2.50 V and
3.00 V. Three consecutive groups below the range increase the DAC by 0.01 V;
three consecutive groups above the range decrease it by 0.01 V. Adjustments are
at least 0.50 seconds apart and are clamped to the configured 0.00-5.00 V DAC
range. A group inside the target range or an invalid/incomplete group clears the
consecutive counters.

AGC hardware control is opt-in at launch:

```bash
roslaunch robotic_fish_io sensor_io.launch enable_agc:=true
```

When no latched DAC state exists, the AGC node first applies `start_voltage`
(0.00 V by default) through `/robotic_fish/dac/set_voltage`. Subsequent changes
also use this service and only accept its confirmed applied voltage. Runtime
control is available through:

```bash
rosservice call /robotic_fish/agc/enable "data: false"
rosservice call /robotic_fish/agc/enable "data: true"
```

AGC status, current raw ADC maximum, DAC voltage, consecutive counters,
adjustment count, and errors are published through `/diagnostics`. Avoid other
DAC command publishers while AGC is enabled, and leave the DAC command watchdog
disabled (`command_timeout: 0.0`) unless its timeout is intentionally coordinated
with the AGC update behavior.

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

## Stable DAC device name

The DAC configuration uses `/dev/robotic_fish_dac`. On the VIM4 used for this
project, the DAC is a CH340 adapter (`1a86:7523`) connected to the dedicated USB
hub port `1-1.2`. Do not configure `/dev/ttyUSB0` directly: its number can
change, and another CH340 adapter may belong to the spinal bridge.

Install the packaged physical-port-specific udev rule:

```bash
source /home/khadas/ros/jsk_aerial_robot_ws/devel/setup.bash
sudo install -m 0644 \
  "$(rospack find robotic_fish_io)/config/udev/99-robotic-fish-dac.rules" \
  /etc/udev/rules.d/99-robotic-fish-dac.rules
sudo udevadm control --reload-rules
sudo udevadm trigger --action=add --subsystem-match=tty \
  --sysname-match=ttyUSB0
sudo udevadm settle
```

The user running ROS must belong to `dialout`. If needed, run the following and
then log out and back in before starting ROS again:

```bash
sudo usermod -aG dialout "$USER"
```

Verify the stable link and permissions:

```bash
ls -l /dev/robotic_fish_dac
test -r /dev/robotic_fish_dac && test -w /dev/robotic_fish_dac
```

The link should resolve to the current `ttyUSB` device. Keep the DAC adapter in
USB port `1-1.2`; moving it intentionally prevents this rule from matching, so
the wrong serial adapter is not driven. To validate the hardware with AGC off,
start only the DAC node and send a safe 0 V command:

```bash
roslaunch robotic_fish_io sensor_io.launch \
  enable_adc:=false enable_dac:=true enable_agc:=false

rosservice call /robotic_fish/dac/set_voltage "channel: 1
voltage: 0.0"
```

A successful response contains `success: True` and `applied_voltage: 0.0`.

## Tests

```bash
python3 -m unittest discover -s test -p 'test_*.py' -v
```
