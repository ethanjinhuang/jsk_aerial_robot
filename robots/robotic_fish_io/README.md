# robotic_fish_io

`robotic_fish_io` is the ROS 1 hardware-interface package for the robotic fish acoustic acquisition system. It provides:

- timestamped acquisition of ADC0, ADC1, and ADC2 through an ADS1115;
- independent transfer-curve calibration for each ADC channel;
- serial DAC output for the LNA gain-control voltage;
- fixed-voltage and closed-loop gain control;
- prioritized DAC reduction when a raw ADC voltage becomes unsafe;
- diagnostics and observable state suitable for rosbag recording.

This package is responsible only for sensor IO and gain control. Robotic-fish motion control belongs to `robotic_fish`, and communication with the embedded controller belongs to `spinal`.

## 1. System relationship

```text
ADS1115 ──I2C──> /adc ──AdcSampleArray──> /gain_control
                   │                            │
                   │                            └──SetDacVoltage──> /dac ──serial──> DAC/LNA
                   │                                                   │
                   └──snapshot the latest confirmed DAC voltage <─────┘

/joy ──> /sonic_teleop_node ──> /servo/target_states ──> /rosserial_server ──> MCU
```

The gain-control node never accesses the DAC serial port directly. `/dac` is the only node that writes to the DAC hardware. `/gain_control` requests voltages through a ROS service and updates its state using the confirmed value published on `/robotic_fish/dac/state`.

## 2. ROS nodes

| Node | Executable | Responsibility |
| --- | --- | --- |
| `/adc` | `adc_node.py` | ADC acquisition, calibration, timestamps, and diagnostics |
| `/dac` | `dac_node.py` | DAC serial access, voltage validation, state feedback, and shutdown safety |
| `/gain_control` | `agc_node.py` | Off, fixed, and closed-loop modes plus ADC overrange protection |

The default command is:

```bash
roslaunch robotic_fish_io sensor_io.launch
```

It starts all three nodes. `/gain_control` starts in `off` mode, but its ADC overrange protection remains active. To omit the controller completely:

```bash
roslaunch robotic_fish_io sensor_io.launch enable_gain_control:=false
```

## 3. Launch files

### 3.1 Package-only launch: `sensor_io.launch`

Use this launch file to test the ADC, DAC, and gain controller without starting spinal:

```bash
roslaunch robotic_fish_io sensor_io.launch gain_mode:=off
```

ADC only:

```bash
roslaunch robotic_fish_io sensor_io.launch \
  enable_adc:=true enable_dac:=false
```

DAC only:

```bash
roslaunch robotic_fish_io sensor_io.launch \
  enable_adc:=false enable_dac:=true
```

`/gain_control` is not started if either the ADC or DAC node is disabled.

### 3.2 IO and embedded bridge: `io.launch`

```bash
roslaunch robotic_fish_io io.launch dev:=vim4 gain_mode:=off
```

In addition to the three package nodes, this launch file includes:

```text
spinal/launch/bridge.launch
```

Serial mode on the VIM4 starts `/rosserial_server` with:

```text
/dev/ttyS4
921600 baud
```

Use `sensor_io.launch` when spinal is not needed. Otherwise, an unavailable embedded controller will cause repeated `Sync with device lost` warnings even if the ADC and DAC nodes are operating correctly.

### 3.3 Common launch arguments

| Argument | Default | Description |
| --- | ---: | --- |
| `enable_adc` | `true` | Start the ADC node |
| `enable_dac` | `true` | Start the DAC node |
| `enable_gain_control` | `true` | Start gain control when both ADC and DAC are enabled |
| `gain_mode` | `off` | Select `off`, `fixed`, or `closed_loop` |
| `fixed_gain_voltage` | `0.0` | Target DAC voltage in fixed mode |
| `agc_target_min_v` | `2.5` | Lower raw-ADC target in closed-loop mode |
| `agc_target_max_v` | `3.0` | Upper raw-ADC target in closed-loop mode |
| `agc_step_v` | `0.01` | Normal closed-loop DAC adjustment step |
| `adc_safety_limit_v` | `3.50` | Raw-ADC software safety threshold |
| `enable_agc` | `false` | Deprecated compatibility argument; `true` selects closed-loop mode |

New launch commands should use `gain_mode`. Do not combine `enable_agc:=true` with fixed mode.

## 4. ADC acquisition and calibration

### 4.1 Default hardware configuration

| Setting | Default |
| --- | --- |
| I2C device | `/dev/i2c-0` |
| ADS1115 address | `0x48` |
| Channels | `0, 1, 2` |
| Data rate | `128 SPS` |
| Target group rate per channel | `20 Hz` |
| Full-scale range | ADS1115 driver configuration of `±4.096 V` |

The ADS1115 multiplexes and converts the three channels sequentially. A complete three-channel group is published at approximately 20 Hz, while the combined rate of the individual-sample topic is approximately 60 Hz.

### 4.2 Timestamps

Every `AdcSample` has its own:

- `conversion_start`: start of that channel's conversion;
- `conversion_end`: completion of that channel's conversion;
- `header.stamp`: midpoint of the conversion window.

Although three samples are published together in an `AdcSampleArray`, they retain their individual hardware timing. The three channels must not be interpreted as simultaneous conversions.

### 4.3 Calibration file

The current default calibration is:

```text
config/calibration/adc_independent_transfer_20260901_171408_calbri05_raw.json
```

It is selected in `config/io.yaml`:

```yaml
adc:
  calibration:
    enabled: true
    required: true
    file: calibration/adc_independent_transfer_20260901_171408_calbri05_raw.json
```

A relative path is resolved against this package's `config/` directory. An absolute path is also accepted. With `required: true`, a missing, malformed, or incompatible calibration file prevents the ADC node from starting instead of silently using invalid calibration data.

ADC0, ADC1, and ADC2 each use their own piecewise-linear transfer curve. The calibration does not use DAC voltage as an input and never extrapolates beyond its observed range. If any channel in a group is outside the calibration range, the entire group falls back to raw voltages:

```text
calibration_applied = false
calibrated_voltage = voltage
```

This prevents calibrated and uncalibrated values from being mixed within one three-channel group.

### 4.4 `AdcSample` fields

| Field | Meaning |
| --- | --- |
| `channel` | ADC channel number |
| `raw` | Signed ADS1115 conversion count |
| `voltage` | Voltage directly converted from the ADS1115 count |
| `calibrated_voltage` | Transfer-curve result; equal to `voltage` when calibration is inactive |
| `calibration_gain` | Calibration ratio at the current interpolation point |
| `calibration_applied` | Whether calibration was applied to the complete three-channel group |
| `calibration_id` | Identifier of the active calibration |
| `gain_control_voltage` | Latest confirmed DAC control voltage when this conversion started |
| `gain_control_voltage_valid` | Whether a valid `/robotic_fish/dac/state` has been received |
| `conversion_start` | Conversion start time |
| `conversion_end` | Conversion completion time |

`gain_control_voltage` is the actual DAC voltage applied to the analog gain-control input. It is not an LNA gain ratio. A separate voltage-to-gain calibration is required before physical gain can be reported.

## 5. DAC control

Default configuration:

| Setting | Default |
| --- | --- |
| Serial device | `/dev/robotic_fish_dac` |
| Baud rate | `19200` |
| Channel | `1` |
| Voltage range | `0.00–5.00 V` |
| Resolution | `0.01 V` |
| Echo verification | Enabled |
| Safe shutdown voltage | `0.00 V` |

Set DAC channel 1 to 0 V:

```bash
rosservice call /robotic_fish/dac/set_voltage \
  "channel: 1
voltage: 0.0"
```

Example successful response:

```text
success: True
applied_voltage: 0.0
```

A `std_msgs/Float32` command can also be published to `/robotic_fish/dac/command`. Do not use another DAC command source while fixed or closed-loop control is active, because the sources would overwrite one another.

Non-numeric values, NaN, Inf, voltages below 0 V, and voltages above 5 V are rejected. On a normal shutdown, `zero_on_shutdown: true` makes the node attempt to apply `safe_voltage`, which defaults to 0 V.

## 6. LNA gain control

All gain-control decisions use the maximum raw voltage in a complete ADC group:

```text
adc_max_raw_voltage = max(ADC0.voltage, ADC1.voltage, ADC2.voltage)
```

The controller deliberately ignores `calibrated_voltage`, so clipping protection does not depend on calibration validity or calibration-file contents.

### 6.1 `off` mode

```bash
roslaunch robotic_fish_io sensor_io.launch gain_mode:=off
```

Normal fixed or closed-loop adjustment is disabled, but the controller continues to monitor the ADC safety threshold. If the DAC state is known and an overrange occurs, safety logic can still lower the DAC. If no DAC state is available and an overrange is detected, the controller requests the configured DAC minimum.

### 6.2 `fixed` mode

```bash
roslaunch robotic_fish_io sensor_io.launch \
  gain_mode:=fixed \
  fixed_gain_voltage:=0.20
```

The default ramp changes the DAC by 0.05 V every 0.10 seconds instead of jumping directly to the target:

```text
0.00 → 0.05 → 0.10 → 0.15 → 0.20 V
```

The state becomes `fixed_holding` after reaching the target. Start real-hardware tests at a low voltage; do not begin by commanding 5 V.

### 6.3 `closed_loop` mode

```bash
roslaunch robotic_fish_io sensor_io.launch \
  gain_mode:=closed_loop \
  agc_target_min_v:=2.50 \
  agc_target_max_v:=3.00 \
  agc_step_v:=0.01
```

Default behavior:

```text
maximum raw ADC < 2.50 V for 3 consecutive groups: increase DAC by 0.01 V
2.50 V <= maximum raw ADC <= 3.00 V: hold
maximum raw ADC > 3.00 V for 3 consecutive groups: decrease DAC by 0.01 V
minimum interval between normal adjustments: 0.50 s
```

The upper ADC target must be strictly below the safety threshold. Both the normal closed-loop step and the fixed-mode ramp step must not exceed `max_normal_step_v`, which defaults to 0.10 V.

### 6.4 ADC software safety protection

Default thresholds:

| Parameter | Default |
| --- | ---: |
| Trigger threshold `adc_safety_limit_v` | `3.50 V` |
| Recovery threshold `adc_safety_recovery_v` | `3.30 V` |
| Safety reduction step `safety_step_v` | `0.10 V` |
| Safety adjustment interval `safety_interval_s` | `0.10 s` |

When any raw ADC channel reaches or exceeds 3.50 V:

1. normal consecutive-sample counting and the normal AGC interval are bypassed;
2. the DAC is reduced by 0.10 V every 0.10 seconds;
3. reduction continues until the maximum raw ADC voltage is at or below 3.30 V;
4. every command remains clamped to the 0–5 V DAC range.

After an overrange in fixed mode, the controller enters `fixed_limited` and does not automatically ramp back to the original fixed target. Reset the latch only after the cause of the overrange has been checked:

```bash
rosservice call /robotic_fish/gain_control/reset_safety
```

Closed-loop mode waits 0.20 seconds after recovery, clears its counters, and then resumes normal control. If the DAC has reached 0 V while the ADC remains above the recovery threshold, the state becomes:

```text
adc_overrange_at_dac_min
```

Voltage increases remain blocked, and an error is reported through `/diagnostics`.

The controller never increases DAC voltage when ADC data is stale, incomplete, non-finite, or when the DAC state or DAC service is invalid.

> Software protection can react only after an overrange has been sampled. It cannot prevent electrical transients between samples. Hardware clamping or input protection is still required at the ADC input.

### 6.5 Advanced parameters

Advanced settings are under `gain_control` in `config/io.yaml`:

| Parameter | Default | Description |
| --- | ---: | --- |
| `dac_min_v` | `0.0` | Minimum DAC voltage allowed by the controller |
| `dac_max_v` | `5.0` | Maximum DAC voltage allowed by the controller |
| `max_normal_step_v` | `0.10` | Maximum allowed non-safety adjustment step |
| `start_voltage` | `0.0` | Safe initialization voltage when no DAC state exists |
| `fixed_ramp_step_v` | `0.05` | Fixed-mode ramp step |
| `fixed_ramp_interval_s` | `0.10` | Fixed-mode ramp interval |
| `interval_s` | `0.50` | Closed-loop normal adjustment interval |
| `consecutive_samples` | `3` | Consecutive groups required for a closed-loop adjustment |
| `recovery_settle_s` | `0.20` | Settling delay after overrange recovery |
| `adc_timeout` | `1.0` | ADC sample-group validity timeout |
| `service_timeout` | `1.0` | DAC service wait timeout |

## 7. ROS interfaces

### 7.1 Topics

| Topic | Type | Publisher/subscriber | Content |
| --- | --- | --- | --- |
| `/robotic_fish/adc/sample` | `robotic_fish_io/AdcSample` | Published by `/adc` | One channel conversion |
| `/robotic_fish/adc/samples` | `robotic_fish_io/AdcSampleArray` | Published by `/adc` | Complete three-channel group |
| `/robotic_fish/dac/command` | `std_msgs/Float32` | Subscribed by `/dac` | External DAC voltage command |
| `/robotic_fish/dac/state` | `std_msgs/Float32` | Published by `/dac` | Confirmed applied DAC voltage |
| `/robotic_fish/gain_control/state` | `robotic_fish_io/GainControlState` | Published by `/gain_control` | Mode, action, ADC/DAC, and safety state |
| `/diagnostics` | `diagnostic_msgs/DiagnosticArray` | Published by all three nodes | Connection, sampling, calibration, control, and error status |

`/robotic_fish/gain_control/state` is latched, so a new subscriber immediately receives the latest state. Unknown numeric values are represented by NaN and must be interpreted together with `adc_valid` and `dac_state_valid`.

### 7.2 Services

| Service | Type | Description |
| --- | --- | --- |
| `/robotic_fish/dac/set_voltage` | `robotic_fish_io/SetDacVoltage` | Request and confirm a DAC voltage |
| `/robotic_fish/gain_control/reset_safety` | `std_srvs/Trigger` | Clear the fixed-mode safety latch; rejected while overrange remains active |
| `/robotic_fish/agc/enable` | `std_srvs/SetBool` | Compatibility API: true selects closed-loop and false selects off |

### 7.3 Useful inspection commands

```bash
rostopic hz /robotic_fish/adc/samples
rostopic echo -n 1 /robotic_fish/adc/samples
rostopic echo /robotic_fish/dac/state
rostopic echo /robotic_fish/gain_control/state
rostopic echo /diagnostics
```

## 8. Full-system startup

The current system uses three independent launch files. Run them in separate terminals.

Terminal 1 — embedded bridge, ADC, DAC, and gain control:

```bash
source /home/khadas/ros/jsk_aerial_robot_ws/devel/setup.bash
roslaunch robotic_fish_io io.launch \
  dev:=vim4 \
  gain_mode:=fixed \
  fixed_gain_voltage:=0.20
```

Terminal 2 — joystick and sonic motion control:

```bash
source /home/khadas/ros/jsk_aerial_robot_ws/devel/setup.bash
roslaunch robotic_fish sonic_teleop.launch
```

Terminal 3 — rosbag recording:

```bash
source /home/khadas/ros/jsk_aerial_robot_ws/devel/setup.bash
roslaunch robotic_fish record.launch
```

`sonic_teleop` can publish a servo target as soon as joystick messages arrive. Secure the fish, keep the tail motion area clear, and verify stable spinal communication before starting the full system.

## 9. Rosbag recording

`robots/robotic_fish/launch/record.launch` currently records:

```text
/robotic_fish/adc/samples
/robotic_fish/dac/command
/robotic_fish/dac/state
/robotic_fish/gain_control/state
/diagnostics
/joy
/imu
/servo/target_states
/servo/states
```

Start recording with:

```bash
roslaunch robotic_fish record.launch
```

The default output filename is similar to:

```text
/home/khadas/sonic_2026-09-02-19-30-15.bag
```

While recording, the file has a `.bag.active` suffix. Press Ctrl+C to close it normally and produce the final `.bag` file. Do not power off the computer while the bag is active.

Inspect a recording:

```bash
rosbag info /home/khadas/sonic_TIMESTAMP.bag
rosbag check /home/khadas/sonic_TIMESTAMP.bag
```

Replay it with:

```bash
rosbag play /home/khadas/sonic_TIMESTAMP.bag
```

Gain control uses a service to command the DAC, and ROS service requests are not stored as rosbag topics. Reconstruct gain-control behavior primarily from:

```text
/robotic_fish/dac/state
/robotic_fish/gain_control/state
/robotic_fish/adc/samples[*].gain_control_voltage
```

`/robotic_fish/dac/command` contains only commands sent through the command topic and may have no messages when all voltage changes come from the gain-control service.

## 10. Stable DAC device name

The default device is `/dev/robotic_fish_dac`, avoiding direct dependence on a changing `/dev/ttyUSB0` number. The current VIM4 rule identifies the CH340 adapter (`1a86:7523`) attached to the dedicated USB hub port `1-1.2`.

Install the rule:

```bash
source /home/khadas/ros/jsk_aerial_robot_ws/devel/setup.bash
sudo install -m 0644 \
  "$(rospack find robotic_fish_io)/config/udev/99-robotic-fish-dac.rules" \
  /etc/udev/rules.d/99-robotic-fish-dac.rules
sudo udevadm control --reload-rules
sudo udevadm trigger --action=add --subsystem-match=tty
sudo udevadm settle
```

The ROS user must belong to `dialout`:

```bash
groups
sudo usermod -aG dialout "$USER"
```

Log out and back in after changing group membership. Verify the device and permissions:

```bash
ls -l /dev/robotic_fish_dac
readlink -f /dev/robotic_fish_dac
test -r /dev/robotic_fish_dac && test -w /dev/robotic_fish_dac
```

Keep the DAC adapter connected to the physical USB port encoded in the rule. This prevents the system from accidentally driving another serial adapter used by spinal.

## 11. Troubleshooting

### 11.1 `Sync with device lost`

This warning comes from `/rosserial_server`, not from the ADC, DAC, or gain-control nodes. It means `/dev/ttyS4` could be opened, but a valid rosserial handshake was not received.

Check:

- embedded-controller power and reset state;
- whether the current spinal firmware is installed;
- that both host and firmware use 921600 baud;
- crossed UART TX/RX and a shared ground;
- the VIM4 UART pinmux/overlay;
- whether another process owns `/dev/ttyS4`.

Use the package-only launch while testing sensors:

```bash
roslaunch robotic_fish_io sensor_io.launch
```

### 11.2 `/dev/robotic_fish_dac` does not exist

```bash
lsusb
ls -l /dev/ttyUSB*
sudo udevadm control --reload-rules
sudo udevadm trigger --action=add --subsystem-match=tty
```

Confirm that the CH340 is connected to the physical port expected by the udev rule and that the user belongs to `dialout`.

### 11.3 `calibration_applied: false`

Common causes are:

- at least one channel is outside the calibration file's observed range;
- calibration is disabled;
- the current group does not satisfy the required three-channel configuration.

Inspect `calibration_status` and `calibration_id` in `/diagnostics`.

### 11.4 `adc_stale`

No complete ADC sample group has arrived within `adc_timeout`. The controller will not increase DAC voltage in this state. Check the ADC node, I2C device, and `/robotic_fish/adc/samples` frequency.

### 11.5 `fixed_limited`

Fixed mode previously triggered ADC safety protection. After confirming that the signal is safe, reset the latch:

```bash
rosservice call /robotic_fish/gain_control/reset_safety
```

### 11.6 `adc_overrange_at_dac_min`

The DAC has reached 0 V, but the raw ADC voltage remains above the recovery threshold. Do not increase gain. Check input amplitude, analog biasing, the signal chain, and hardware input protection.

## 12. Build and test

Build the package:

```bash
cd /home/khadas/ros/jsk_aerial_robot_ws
catkin build robotic_fish_io --no-deps
source devel/setup.bash
```

Run the unit tests:

```bash
cd /home/khadas/ros/jsk_aerial_robot_ws/src/jsk_aerial_robot
python3 -m unittest discover \
  -s robots/robotic_fish_io/test \
  -p 'test_*.py' -v
```

The current tests cover:

- ADS1115 configuration, conversion, and timeout behavior;
- DAC 0–5 V limits, BCD commands, and echo verification;
- independent three-channel calibration and whole-group fallback;
- fixed-voltage ramping;
- closed-loop counters, step size, interval, and DAC bounds;
- 3.50/3.30 V safety triggering and recovery;
- fixed-mode safety latching;
- overrange behavior at the minimum DAC voltage.

Recommended first-hardware-test sequence:

1. start with `gain_mode:=off` and inspect ADC data and diagnostics;
2. explicitly apply 0 V through the DAC service;
3. begin fixed-mode testing at a low voltage such as 0.20 V;
4. confirm that reducing DAC voltage actually reduces ADC amplitude during an overrange;
5. proceed to closed-loop and full-system motion tests;
6. record a rosbag and inspect the message count of every expected topic after stopping.
